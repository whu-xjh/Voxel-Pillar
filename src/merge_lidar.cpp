#include <ros/ros.h>
#include <ros/package.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <fstream>
#include <sys/stat.h>
#include <chrono>
#include <iomanip>

// Merges the point clouds of three Livox LiDARs (IP last octet 159/160/161)
// into one timestamp-sorted CustomMsg stream.
class LivoxLidarMerger {
public:
    LivoxLidarMerger(ros::NodeHandle& nh) : nh_(nh) {
        // Get package path and create the per-session log file
        std::string package_path = ros::package::getPath("voxel_pillar");
        log_dir_ = package_path + "/Log/launch_log";
        mkdir(log_dir_.c_str(), 0777);

        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << log_dir_ << "/" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".log";
        log_file_path_ = ss.str();

        log_file_.open(log_file_path_, std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            ROS_ERROR("Failed to open log file: %s", log_file_path_.c_str());
        } else {
            ROS_INFO("Log file created: %s", log_file_path_.c_str());
        }

        // Get parameters: three input topics and one output topic
        nh_.param<std::string>("output_topic", output_topic_, "/livox/multi_lidar");
        nh_.param<std::string>("input_topic_159", input_topics_[0], "/livox/lidar_192_168_1_159");
        nh_.param<std::string>("input_topic_160", input_topics_[1], "/livox/lidar_192_168_1_160");
        nh_.param<std::string>("input_topic_161", input_topics_[2], "/livox/lidar_192_168_1_161");

        // Create ROS publisher for merged point cloud, queue size = 10
        merged_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>(output_topic_, 10);

        // Create subscribers (one per LiDAR, sharing a single gated callback)
        for (int i = 0; i < kNumLidars; ++i) {
            boost::function<void(const livox_ros_driver::CustomMsg::ConstPtr&)> cb =
                [this, i](const livox_ros_driver::CustomMsg::ConstPtr& msg) { lidarCallback(i, msg); };
            subs_[i] = nh_.subscribe(input_topics_[i], 100, cb);
            has_data_[i] = false;
        }

        writeLog("Livox Multi Lidar initialized");
        writeLog("Input topics: " + input_topics_[0] + ", " + input_topics_[1] + ", " + input_topics_[2]);
        writeLog("Output topic: " + output_topic_);
    }

    ~LivoxLidarMerger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    static constexpr int kNumLidars = 3;
    static constexpr int kLidarIds[kNumLidars] = {159, 160, 161};

    void writeLog(const std::string& message) {
        if (log_file_.is_open()) {
            // Add timestamp
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

            log_file_ << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                     << "." << std::setfill('0') << std::setw(3) << ms.count()
                     << " - " << message << std::endl;
            log_file_.flush();
        }
    }

    // Gated message reception: LiDAR i is only accepted once LiDAR i-1 of the
    // current round has arrived (159 -> 160 -> 161); 161 triggers the merge.
    void lidarCallback(int idx, const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        if (idx > 0 && !has_data_[idx - 1]) {
            writeLog("Received " + std::to_string(kLidarIds[idx]) +
                     " LiDAR out of sequence, waiting for " + std::to_string(kLidarIds[idx - 1]) + " data first");
            return;
        }

        msgs_[idx] = msg;
        has_data_[idx] = true;
        writeLog("Received " + std::to_string(kLidarIds[idx]) + " LiDAR message with " +
                 std::to_string(msg->point_num) + " points");

        // Try to merge and publish once the last LiDAR of the round has arrived
        if (idx == kNumLidars - 1) {
            mergeAndPublish();
        }
    }

    // Merge the three buffered scans into one timestamp-sorted message
    void mergeAndPublish() {
        for (int i = 0; i < kNumLidars; ++i) {
            if (!has_data_[i]) return;
        }
        writeLog("Starting timestamp-based merging");

        size_t total_points = 0;
        for (int i = 0; i < kNumLidars; ++i) {
            total_points += msgs_[i]->point_num;
        }

        // (absolute time, source index, point index within source)
        std::vector<std::tuple<uint64_t, int, size_t>> sorted_indices;
        sorted_indices.reserve(total_points);
        for (int i = 0; i < kNumLidars; ++i) {
            size_t idx = 0;
            for (const auto& point : msgs_[i]->points) {
                uint64_t time = msgs_[i]->timebase + point.offset_time;
                sorted_indices.emplace_back(time, i, idx++);
            }
        }

        // Sort by absolute timestamp
        std::sort(sorted_indices.begin(), sorted_indices.end());

        // Use the header and timebase of the earliest scan as merged reference
        int earliest = 0;
        for (int i = 1; i < kNumLidars; ++i) {
            if (msgs_[i]->timebase < msgs_[earliest]->timebase) earliest = i;
        }

        livox_ros_driver::CustomMsg merged_msg;
        merged_msg.header = msgs_[earliest]->header;
        merged_msg.timebase = msgs_[earliest]->timebase;
        merged_msg.point_num = total_points;
        merged_msg.points.reserve(total_points);

        for (const auto& [time, src, point_idx] : sorted_indices) {
            livox_ros_driver::CustomPoint adjusted_point = msgs_[src]->points[point_idx];
            adjusted_point.offset_time = time - merged_msg.timebase;
            merged_msg.points.push_back(adjusted_point);
        }

        // Publish merged message
        merged_pub_.publish(merged_msg);

        // Statistics
        uint64_t min_time = std::get<0>(sorted_indices.front());
        uint64_t max_time = std::get<0>(sorted_indices.back());
        double time_span = (max_time - min_time) / 1000000000.0;  // Convert to seconds

        writeLog("Published timestamp-sorted merged message with " + std::to_string(total_points) + " points");
        writeLog("Time span: " + std::to_string(time_span) + " seconds");
        writeLog("Points per source: 159=" + std::to_string(msgs_[0]->point_num) +
                ", 160=" + std::to_string(msgs_[1]->point_num) +
                ", 161=" + std::to_string(msgs_[2]->point_num));

        // Reset flags and cache for the next round
        for (int i = 0; i < kNumLidars; ++i) {
            has_data_[i] = false;
            msgs_[i].reset();
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher merged_pub_;
    ros::Subscriber subs_[kNumLidars];

    std::string output_topic_;
    std::string input_topics_[kNumLidars];
    std::string log_dir_;
    std::string log_file_path_;
    std::ofstream log_file_;  // Log file stream

    // Buffer and arrival flags for the current round, one entry per LiDAR
    livox_ros_driver::CustomMsg::ConstPtr msgs_[kNumLidars];
    bool has_data_[kNumLidars];
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_lidar");
    ros::NodeHandle nh;

    LivoxLidarMerger merger(nh);

    ros::spin();
    return 0;
}
