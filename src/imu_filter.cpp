/*
IMU Filter Node for FAST-LIVO2
Filter IMU topics to keep only imu159
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <deque>

class IMUFilter {
public:
    IMUFilter(ros::NodeHandle& nh) {
        // Initialize subscribers for both IMUs
        sub_imu159_ = nh.subscribe<sensor_msgs::Imu>(
            "/livox/imu_192_168_1_159", 100, &IMUFilter::imu159Callback, this);
        sub_imu160_ = nh.subscribe<sensor_msgs::Imu>(
            "/livox/imu_192_168_1_160", 100, &IMUFilter::imu160Callback, this);

        // Publisher for filtered IMU data
        pub_filtered_imu_ = nh.advertise<sensor_msgs::Imu>(
            "/livox/imu_filtered", 100);

        // Parameters
        nh.param("max_imu_rate", max_imu_rate_, 200.0); // Hz
        nh.param("enable_imu_filtering", enable_imu_filtering_, true);
        nh.param("publish_imu159_only", publish_imu159_only_, true);

        // Initialize time tracking
        last_publish_time_ = ros::Time::now();
        min_publish_interval_ = 1.0 / max_imu_rate_;

        ROS_INFO("IMU Filter initialized - publishing IMU159 only at max rate: %.1f Hz", max_imu_rate_);
    }

private:
    void imu159Callback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (publish_imu159_only_) {
            // Process and publish IMU159 data
            processAndPublishIMU(msg);
        }
    }

    void imu160Callback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!publish_imu159_only_) {
            // Could implement IMU fusion logic here if needed
            ROS_INFO("Received IMU160 data - ignoring as per configuration");
        }
    }

    void processAndPublishIMU(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!enable_imu_filtering_) {
            // Publish directly without filtering
            pub_filtered_imu_.publish(*msg);
            return;
        }

        // Rate limiting
        ros::Time current_time = msg->header.stamp;
        double time_since_last_publish = (current_time - last_publish_time_).toSec();

        if (time_since_last_publish >= min_publish_interval_) {
            // Create filtered message
            sensor_msgs::Imu filtered_msg = *msg;
            
            // Optional: Apply basic filtering
            applyBasicFiltering(filtered_msg);
            
            // Publish filtered message
            pub_filtered_imu_.publish(filtered_msg);
            
            last_publish_time_ = current_time;
            
            ROS_INFO("Published filtered IMU at time: %.6f", current_time.toSec());
        } else {
            ROS_INFO("IMU message skipped due to rate limiting");
        }
    }

    void applyBasicFiltering(sensor_msgs::Imu& imu_msg) {
        // Basic outlier detection and filtering
        const double max_acceleration = 20.0; // m/s^2
        const double max_angular_velocity = 10.0; // rad/s
        
        // Check acceleration values
        if (std::abs(imu_msg.linear_acceleration.x) > max_acceleration) {
            ROS_WARN("Acceleration X outlier detected: %.2f", imu_msg.linear_acceleration.x);
            imu_msg.linear_acceleration.x = 0.0;
        }
        if (std::abs(imu_msg.linear_acceleration.y) > max_acceleration) {
            ROS_WARN("Acceleration Y outlier detected: %.2f", imu_msg.linear_acceleration.y);
            imu_msg.linear_acceleration.y = 0.0;
        }
        if (std::abs(imu_msg.linear_acceleration.z) > max_acceleration) {
            ROS_WARN("Acceleration Z outlier detected: %.2f", imu_msg.linear_acceleration.z);
            imu_msg.linear_acceleration.z = 0.0;
        }
        
        // Check angular velocity values
        if (std::abs(imu_msg.angular_velocity.x) > max_angular_velocity) {
            ROS_WARN("Angular velocity X outlier detected: %.2f", imu_msg.angular_velocity.x);
            imu_msg.angular_velocity.x = 0.0;
        }
        if (std::abs(imu_msg.angular_velocity.y) > max_angular_velocity) {
            ROS_WARN("Angular velocity Y outlier detected: %.2f", imu_msg.angular_velocity.y);
            imu_msg.angular_velocity.y = 0.0;
        }
        if (std::abs(imu_msg.angular_velocity.z) > max_angular_velocity) {
            ROS_WARN("Angular velocity Z outlier detected: %.2f", imu_msg.angular_velocity.z);
            imu_msg.angular_velocity.z = 0.0;
        }
        
        // Ensure orientation covariance is set properly
        if (imu_msg.orientation_covariance[0] == 0.0) {
            // Set high covariance for orientation (as we typically don't trust it)
            for (int i = 0; i < 9; ++i) {
                imu_msg.orientation_covariance[i] = (i % 4 == 0) ? 99999.0 : 0.0;
            }
        }
    }

    ros::Subscriber sub_imu159_, sub_imu160_;
    ros::Publisher pub_filtered_imu_;

    ros::Time last_publish_time_;
    double min_publish_interval_;
    double max_imu_rate_;
    bool enable_imu_filtering_;
    bool publish_imu159_only_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_filter");
    ros::NodeHandle nh;
    
    IMUFilter filter(nh);
    
    ros::spin();
    return 0;
}