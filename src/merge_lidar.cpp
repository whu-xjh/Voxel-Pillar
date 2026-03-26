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

class LivoxLidarMerger {
public:
    LivoxLidarMerger(ros::NodeHandle& nh) : nh_(nh) {
        // Get package path
        std::string package_path = ros::package::getPath("voxel_pillar");
        log_dir_ = package_path + "/Log/launch_log";

        // Create Log directory if not exists
        mkdir(log_dir_.c_str(), 0777);

        // Generate timestamped log filename
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << log_dir_ << "/" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".log";
        log_file_path_ = ss.str();

        // Open log file
        log_file_.open(log_file_path_, std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            ROS_ERROR("Failed to open log file: %s", log_file_path_.c_str());
        } else {
            ROS_INFO("Log file created: %s", log_file_path_.c_str());
        }

        // Get parameters: three input topics and one output topic
        nh_.param<std::string>("output_topic", output_topic_, "/livox/multi_lidar");
        nh_.param<std::string>("input_topic_159", input_topic_159_, "/livox/lidar_192_168_1_159");
        nh_.param<std::string>("input_topic_160", input_topic_160_, "/livox/lidar_192_168_1_160");
        nh_.param<std::string>("input_topic_161", input_topic_161_, "/livox/lidar_192_168_1_161");

        // Create ROS publisher for merged point cloud, queue size = 10
        merged_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>(output_topic_, 10);

        // Create subscribers
        sub_159_ = nh_.subscribe(input_topic_159_, 100, &LivoxLidarMerger::lidar159Callback, this);
        sub_160_ = nh_.subscribe(input_topic_160_, 100, &LivoxLidarMerger::lidar160Callback, this);
        sub_161_ = nh_.subscribe(input_topic_161_, 100, &LivoxLidarMerger::lidar161Callback, this);

        // Initialize flags and cache
        has_159_data_ = false;
        has_160_data_ = false;
        has_161_data_ = false;

        writeLog("Livox Multi Lidar initialized");
        writeLog("Input topics: " + input_topic_159_ + ", " + input_topic_160_ + ", " + input_topic_161_);
        writeLog("Output topic: " + output_topic_);
    }

    ~LivoxLidarMerger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
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

    // Merge and publish data (timestamp-sorted version)
    void mergeAndPublish() {
        if (has_159_data_ && has_160_data_ && has_161_data_) {
            writeLog("Starting timestamp-based merging");

            size_t total_points = msg_159_->point_num + msg_160_->point_num + msg_161_->point_num;

            std::vector<std::tuple<uint64_t, int, size_t>> sorted_indices;  // (time, source_id, point_index)
            sorted_indices.reserve(total_points);

            size_t idx = 0;
            for (const auto& point : msg_159_->points) {
                uint64_t time = msg_159_->timebase + point.offset_time;
                sorted_indices.emplace_back(time, 159, idx++);
            }
            idx = 0;
            for (const auto& point : msg_160_->points) {
                uint64_t time = msg_160_->timebase + point.offset_time;
                sorted_indices.emplace_back(time, 160, idx++);
            }
            idx = 0;
            for (const auto& point : msg_161_->points) {
                uint64_t time = msg_161_->timebase + point.offset_time;
                sorted_indices.emplace_back(time, 161, idx++);
            }

            // Sort
            std::sort(sorted_indices.begin(), sorted_indices.end());

            // Create merged message
            livox_ros_driver::CustomMsg merged_msg;
            if (msg_159_->timebase < msg_160_->timebase && msg_159_->timebase < msg_161_->timebase) {
                merged_msg.header = msg_159_->header;  // Use 159 header
                merged_msg.timebase = msg_159_->timebase; // 159
            } else if (msg_160_->timebase < msg_159_->timebase && msg_160_->timebase < msg_161_->timebase) {
                merged_msg.header = msg_160_->header;  // Use 160 header
                merged_msg.timebase = msg_160_->timebase; // 160
            } else {
                merged_msg.header = msg_161_->header;  // Use 161 header
                merged_msg.timebase = msg_161_->timebase; // 161
            }
            merged_msg.point_num = total_points;
            merged_msg.points.reserve(total_points);

            for (const auto& [time, source_id, point_idx] : sorted_indices) {
                livox_ros_driver::CustomPoint adjusted_point;
                if (source_id == 159) adjusted_point = msg_159_->points[point_idx];
                else if (source_id == 160) adjusted_point = msg_160_->points[point_idx];
                else if (source_id == 161) adjusted_point = msg_161_->points[point_idx];
                else continue; // Should not happen
                adjusted_point.offset_time = time - merged_msg.timebase;
                merged_msg.points.push_back(adjusted_point);
            }

            /*  Another version of merging topics based on time alignment
                // Create timestamp-point structure
                struct TimestampedPoint {
                    uint64_t absolute_time;  // Absolute timestamp
                    livox_ros_driver::CustomPoint point;
                    int source_id;  // 159, 160, or 161

                    // Comparison operator for sorting
                    bool operator<(const TimestampedPoint& other) const {
                        return absolute_time < other.absolute_time;
                    }
                };

                // Calculate total points and create container
                int total_points = msg_159_->point_num + msg_160_->point_num + msg_161_->point_num;
                std::vector<TimestampedPoint> timestamped_points;
                timestamped_points.reserve(total_points);

                // Process 159 points (as reference timebase)
                for (const auto& point : msg_159_->points) {
                    TimestampedPoint tp;
                    tp.absolute_time = msg_159_->timebase + point.offset_time;
                    tp.point = point;
                    tp.source_id = 159;
                    timestamped_points.push_back(tp);
                }

                // Process 160 points (convert to absolute time)
                for (const auto& point : msg_160_->points) {
                    TimestampedPoint tp;
                    tp.absolute_time = msg_160_->timebase + point.offset_time;
                    tp.point = point;
                    tp.source_id = 160;
                    timestamped_points.push_back(tp);
                }

                // Process 161 points (convert to absolute time)
                for (const auto& point : msg_161_->points) {
                    TimestampedPoint tp;
                    tp.absolute_time = msg_161_->timebase + point.offset_time;
                    tp.point = point;
                    tp.source_id = 161;
                    timestamped_points.push_back(tp);
                }

                // Sort by timestamp
                std::sort(timestamped_points.begin(), timestamped_points.end());

                // Create merged message
                livox_ros_driver::CustomMsg merged_msg;
                merged_msg.header = msg_159_->header;  // Use 159 header
                merged_msg.timebase = msg_159_->timebase;
                merged_msg.point_num = total_points;
                merged_msg.points.reserve(total_points);

                if (msg_159_->timebase < msg_160_->timebase && msg_159_->timebase < msg_161_->timebase) {
                    merged_msg.timebase = msg_159_->timebase; // 159
                } else if (msg_160_->timebase < msg_159_->timebase && msg_160_->timebase < msg_161_->timebase) {
                    merged_msg.timebase = msg_160_->timebase; // 160
                } else {
                    merged_msg.timebase = msg_161_->timebase; // 161
                }

                // Convert sorted points to relative time
                for (const auto& tp : timestamped_points) {
                    livox_ros_driver::CustomPoint adjusted_point = tp.point;

                    // Calculate offset time relative to timebase
                    if (tp.absolute_time >= merged_msg.timebase) {
                        adjusted_point.offset_time = tp.absolute_time - merged_msg.timebase;
                    } else {
                        // // Handle case where timestamp is before reference (should not happen)
                        // writeLog("Warning: Point timestamp " + std::to_string(tp.absolute_time) +
                        //         " is before reference timebase " + std::to_string(merged_msg.timebase));
                        continue; // Skip this point
                    }

                    merged_msg.points.push_back(adjusted_point);
                }
            */

            // Publish merged message
            merged_pub_.publish(merged_msg);

            // Statistics
            uint64_t min_time = std::get<0>(sorted_indices.front());
            uint64_t max_time = std::get<0>(sorted_indices.back());
            double time_span = (max_time - min_time) / 1000000000.0;  // Convert to seconds

            writeLog("Published timestamp-sorted merged message with " + std::to_string(total_points) + " points");
            writeLog("Time span: " + std::to_string(time_span) + " seconds");
            writeLog("Points per source: 159=" + std::to_string(msg_159_->point_num) +
                    ", 160=" + std::to_string(msg_160_->point_num) +
                    ", 161=" + std::to_string(msg_161_->point_num));

            // Reset flags and cache
            has_159_data_ = false;
            has_160_data_ = false;
            has_161_data_ = false;
            msg_159_.reset();
            msg_160_.reset();
            msg_161_.reset();
        }
    }

    void lidar159Callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        // Save 159 data
        msg_159_ = msg;
        has_159_data_ = true;
        writeLog("Received 159 LiDAR message with " + std::to_string(msg->point_num) + " points");

        // Try to merge and publish
        // mergeAndPublish();
    }

    void lidar160Callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        // Only process 160 data after receiving 159 data
        if (has_159_data_) {
            msg_160_ = msg;
            has_160_data_ = true;
            writeLog("Received 160 LiDAR message with " + std::to_string(msg->point_num) + " points");

            // Try to merge and publish
            // mergeAndPublish();
        } else {
            writeLog("Received 160 LiDAR out of sequence, waiting for 159 data first");
        }
    }

    void lidar161Callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        // Only process 161 data after receiving 159 and 160 data
        if (has_159_data_ && has_160_data_) {
            msg_161_ = msg;
            has_161_data_ = true;
            writeLog("Received 161 LiDAR message with " + std::to_string(msg->point_num) + " points");

            // Try to merge and publish
            mergeAndPublish();
        } else {
            writeLog("Received 161 LiDAR out of sequence, waiting for 159 and 160 data first");
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher merged_pub_;
    ros::Subscriber sub_159_, sub_160_, sub_161_;

    std::string output_topic_;
    std::string input_topic_159_, input_topic_160_, input_topic_161_;
    std::string log_dir_;
    std::string log_file_path_;
    std::ofstream log_file_;  // Log file stream

    // Buffer for received messages
    livox_ros_driver::CustomMsg::ConstPtr msg_159_;
    livox_ros_driver::CustomMsg::ConstPtr msg_160_;
    livox_ros_driver::CustomMsg::ConstPtr msg_161_;

    // Flags indicating whether data from each LiDAR has been received
    bool has_159_data_;
    bool has_160_data_;
    bool has_161_data_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_lidar");
    ros::NodeHandle nh;

    LivoxLidarMerger merger(nh);

    ros::spin();
    return 0;
}
