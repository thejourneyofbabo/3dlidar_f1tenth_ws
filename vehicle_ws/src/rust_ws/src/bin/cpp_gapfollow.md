#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

/// CHECK: include needed ROS msg type headers and libraries
class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!
public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, 
            std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "ReactiveFollowGap node initialized");
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    
    // Additional member variables for algorithm state
    std::optional<size_t> previous_best_point_;
    float angle_increment_;
    size_t ranges_size_;
    
    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        
        const float max_range = 5.0f;
        const float min_range = 0.3f;
        const float vehicle_width = 0.4f;
        
        float min_dist = max_range;
        size_t min_idx = 0;
        
        // Find minimum distance and clamp invalid values
        for (size_t i = 0; i < ranges_size_; ++i)
        {
            if (ranges[i] > max_range || std::isnan(ranges[i]) || std::isinf(ranges[i]))
            {
                ranges[i] = max_range;
            }
            else if (ranges[i] < min_dist && ranges[i] > 0.0f)
            {
                min_dist = ranges[i];
                min_idx = i;
            }
        }
        
        // Create safety bubble around closest obstacle
        if (min_dist < min_range)
        {
            float bubble_radius = ((vehicle_width / 2.0f) / min_dist) / angle_increment_;
            size_t bubble_size = static_cast<size_t>(bubble_radius);
            
            size_t start_idx = (min_idx >= bubble_size) ? min_idx - bubble_size : 0;
            size_t end_idx = std::min(min_idx + bubble_size, ranges_size_ - 1);
            
            for (size_t i = start_idx; i <= end_idx; ++i)
            {
                ranges[i] = 0.0f;
            }
        }
        
        return;
    }
    
    void find_max_gap(float* ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        
        // Set Region of Interest (ROI)
        const float roi_angle_deg = 67.0f;
        const float roi_angle_rad = roi_angle_deg * M_PI / 180.0f;
        size_t roi_angle_steps = static_cast<size_t>(roi_angle_rad / angle_increment_);
        
        size_t mid_lidar_idx = ranges_size_ / 2;
        size_t roi_idx_start = (mid_lidar_idx >= roi_angle_steps) ? mid_lidar_idx - roi_angle_steps : 0;
        size_t roi_idx_end = std::min(mid_lidar_idx + roi_angle_steps, ranges_size_ - 1);
        
        // Find the largest gap in free space
        const float min_range = 1.5f;
        size_t max_gap_size = 0;
        size_t max_gap_start = roi_idx_start;
        size_t max_gap_end = roi_idx_start;
        size_t gap_start = roi_idx_start;
        bool in_gap = false;
        
        for (size_t i = roi_idx_start; i <= roi_idx_end; ++i)
        {
            bool is_free = ranges[i] > min_range;
            
            if (is_free && !in_gap)
            {
                // Start of new gap
                gap_start = i;
                in_gap = true;
            }
            else if (!is_free && in_gap)
            {
                // End of current gap
                size_t gap_size = i - gap_start;
                if (gap_size > max_gap_size)
                {
                    max_gap_size = gap_size;
                    max_gap_start = gap_start;
                    max_gap_end = i - 1;
                }
                in_gap = false;
            }
        }
        
        // Handle case where gap extends to end of ROI
        if (in_gap)
        {
            size_t gap_size = roi_idx_end - gap_start + 1;
            if (gap_size > max_gap_size)
            {
                max_gap_size = gap_size;
                max_gap_start = gap_start;
                max_gap_end = roi_idx_end;
            }
        }
        
        // Fallback if no gap found
        if (max_gap_size == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Warning: No gap found!");
            max_gap_start = (roi_idx_start + roi_idx_end) / 2;
            max_gap_end = max_gap_start;
        }
        
        // Return indices through pointer parameters
        indice[0] = static_cast<int>(max_gap_start);
        indice[1] = static_cast<int>(max_gap_end);
        
        return;
    }
    
    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
        // Naive: Choose the furthest point within ranges and go there
        
        size_t gap_start = static_cast<size_t>(indice[0]);
        size_t gap_end = static_cast<size_t>(indice[1]);
        
        // Find best point using weighted average
        const float alpha = 0.6f;
        float weighted_sum = 0.0f;
        float weight_total = 0.0f;
        
        for (size_t i = gap_start; i <= gap_end; ++i)
        {
            float weight = ranges[i];
            weighted_sum += static_cast<float>(i) * weight;
            weight_total += weight;
        }
        
        size_t best_point = (weight_total > 0.0f) ? 
                           static_cast<size_t>(weighted_sum / weight_total) : 
                           (gap_start + gap_end) / 2;
        
        // Apply EMA filter
        size_t ema_best;
        if (previous_best_point_.has_value())
        {
            float ema_result = alpha * static_cast<float>(best_point) + 
                              (1.0f - alpha) * static_cast<float>(previous_best_point_.value());
            ema_best = static_cast<size_t>(std::round(ema_result));
        }
        else
        {
            ema_best = best_point;
        }
        
        previous_best_point_ = ema_best;
        
        // Return best point index through pointer parameter
        indice[2] = static_cast<int>(ema_best);
        
        return;
    }
    
    float pure_pursuit(float steer_ang_rad, float lookahead_dist)
    {
        const float lidar_to_rear = 0.27f;
        const float wheel_base = 0.32f;
        
        float bestpoint_x = lookahead_dist * std::cos(steer_ang_rad);
        float bestpoint_y = lookahead_dist * std::sin(steer_ang_rad);
        
        float lookahead_angle = std::atan2(bestpoint_y, bestpoint_x + lidar_to_rear);
        float lookahead_rear = std::sqrt(std::pow(bestpoint_x + lidar_to_rear, 2) + 
                                        std::pow(bestpoint_y, 2));
        
        // Final Pure Pursuit Angle
        return std::atan2(2.0f * wheel_base * std::sin(lookahead_angle), lookahead_rear);
    }
    
    std::pair<float, float> vehicle_control(float* ranges, size_t best_point)
    {
        // Calculate steering angle and speed
        size_t vehicle_center_idx = ranges_size_ / 2;
        float steer_ang_rad = (static_cast<float>(best_point) - 
                              static_cast<float>(vehicle_center_idx)) * angle_increment_;
        
        float best_lookahead = std::min(ranges[best_point], 3.0f);
        float steer_ang_deg = std::abs(steer_ang_rad) * 180.0f / M_PI;
        
        // Adaptive lookahead distance
        float adaptive_lookahead;
        if (steer_ang_deg < 5.0f)
            adaptive_lookahead = best_lookahead * 1.0f;
        else if (steer_ang_deg < 15.0f)
            adaptive_lookahead = best_lookahead * 0.7f;
        else if (steer_ang_deg < 30.0f)
            adaptive_lookahead = best_lookahead * 0.5f;
        else
            adaptive_lookahead = best_lookahead * 0.3f;
        
        float pure_pursuit_steer = pure_pursuit(steer_ang_rad, adaptive_lookahead);
        float final_steer_ang_deg = std::abs(pure_pursuit_steer) * 180.0f / M_PI;
        
        // Speed control based on steering angle (Fast Speed profile)
        float drive_speed;
        if (final_steer_ang_deg < 5.0f)
            drive_speed = 4.0f;
        else if (final_steer_ang_deg < 10.0f)
            drive_speed = 2.5f;
        else if (final_steer_ang_deg < 15.0f)
            drive_speed = 1.2f;
        else
            drive_speed = 0.8f;
        
        RCLCPP_INFO(this->get_logger(), "Final Steer: %.2f°, Driving Speed: %.2f", 
                   final_steer_ang_deg, drive_speed);
        
        return std::make_pair(pure_pursuit_steer, drive_speed);
    }
    
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        /// TODO:
        // Find closest point to LiDAR
        // Eliminate all points inside 'bubble' (set them to zero) 
        // Find max length gap 
        // Find the best point in the gap 
        // Publish Drive message
        
        try
        {
            // Store scan parameters for use in other functions
            angle_increment_ = scan_msg->angle_increment;
            ranges_size_ = scan_msg->ranges.size();
            
            // Create mutable copy of ranges for processing
            std::vector<float> ranges_vec = scan_msg->ranges;
            float* ranges = ranges_vec.data();
            
            // Preprocess LiDAR data (includes bubble elimination around closest point)
            preprocess_lidar(ranges);
            
            // Find the largest gap
            int gap_indices[3]; // [start_idx, end_idx, best_point_idx]
            find_max_gap(ranges, gap_indices);
            
            // Find the best point in the gap
            find_best_point(ranges, gap_indices);
            
            size_t best_point_idx = static_cast<size_t>(gap_indices[2]);
            
            // Calculate control commands
            auto [steering_angle, drive_speed] = vehicle_control(ranges, best_point_idx);
            
            // Create and publish drive message
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.steering_angle = steering_angle;
            drive_msg.drive.speed = drive_speed;
            
            drive_publisher_->publish(drive_msg);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error during scan process: %s", e.what());
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
