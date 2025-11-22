/**
    - Subscribes to the caregiver's odometry
    - Computes a target pose located at a fixed offset distance behind the caregiver
    - The computed pose is published to /caregiver_pos only when FOLLOW mode is enabled
    - Nav2 Following Behavior Tree uses /caregiver_pos as a continuously
    - Updated goal, allowing the wheelchair to follow the caregiver at a constant distance
*/

#include <memory>
#include <cmath>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

//Ros2 Service
#include "std_srvs/srv/set_bool.hpp"

class CaregiverPoseNode : public rclcpp::Node
{
    public:
        CaregiverPoseNode():Node("caregiver_pose_node"), last_yaw(0.0), following_enabled(false), system_clock(RCL_SYSTEM_TIME), tf_buffer(this->get_clock()), tf_listener(tf_buffer)
        {
            offset_distance = this->declare_parameter<double>("offset_distance", 1.3);
            min_offset = this->declare_parameter<double>("min_offset", 0.5);
            max_offset = this->declare_parameter<double>("max_offset", 2.0);
            dynamic_gain = this->declare_parameter<double>("dynamic_gain", 0.6);

            update_period = this->declare_parameter<double>("update_period", 0.05);

            odom_topic = this->declare_parameter<std::string>("odom_topic", "/human_actor/odometry");
            wheelchair_odom_topic = this->declare_parameter<std::string>("wheelchair_odom_topic", "/odometry/filtered");

            output_topic = this->declare_parameter<std::string>("output_topic", "/caregiver_pos");
            global_frame = this->declare_parameter<std::string>("global_frame", "odom");

            // Declare QoS setting
            auto qos = rclcpp::SystemDefaultsQoS(); 

            // Subscriptions
            sub = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic,
                qos,
                std::bind(&CaregiverPoseNode::odomCallback, this, std::placeholders::_1)
            );

            wheelchair_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                wheelchair_odom_topic,
                qos,
                std::bind(&CaregiverPoseNode::wheelchairOdomCallback, this, std::placeholders::_1)
            );

            // Publisher
            pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                output_topic,
                qos
            );

            // Service Call
            follow_service = this->create_service<std_srvs::srv::SetBool>(
                "set_following_enabled",
                [this](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr resp)
                {
                    following_enabled = req->data;
                    resp->success = true;
                    resp->message = following_enabled ? "FOLLOW enabled" : "FOLLOW disabled";
                    RCLCPP_INFO(this->get_logger(), "Following mode: %s", following_enabled ? "ON" : "OFF");
                }
            );

            last_pub_time = system_clock.now();
            has_wheelchair_pose = false;
            RCLCPP_INFO(this->get_logger(), "CaregiverPoseNode started");
        }


    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheelchair_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr follow_service;
        rclcpp::Time last_pub_time;

        double last_yaw;

        bool following_enabled;

        double offset_distance;
        double min_offset;
        double max_offset;
        double dynamic_gain;

        double update_period;
        std::string odom_topic;
        std::string wheelchair_odom_topic;
        std::string output_topic;
        std::string global_frame;

        rclcpp::Clock system_clock;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        geometry_msgs::msg::Pose caregiver_last_pose;   
        geometry_msgs::msg::Pose wheelchair_last_pose; 
        bool has_wheelchair_pose;                           

        void wheelchairOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            geometry_msgs::msg::PoseStamped wheelchair_in, wheelchair_global;
            wheelchair_in.header = msg->header;
            wheelchair_in.pose = msg->pose.pose;
            
            try
            {
                tf_buffer.transform(wheelchair_in, wheelchair_global, global_frame, tf2::durationFromSec(0.1));
                wheelchair_last_pose = wheelchair_global.pose;
                has_wheelchair_pose = true;
            }
            catch(const tf2::TransformException& ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to transform wheelchair pose to %s: %s", global_frame.c_str(), ex.what());
            }
        }

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            if (!following_enabled) return;

            auto now = system_clock.now();
            if((now - last_pub_time).seconds() < update_period) return;

            if(!has_wheelchair_pose) return;

            geometry_msgs::msg::PoseStamped caregiver_pose_in, caregiver_pose_global;
            caregiver_pose_in.header = msg->header;
            caregiver_pose_in.pose = msg->pose.pose;

            try
            {
                tf_buffer.transform(caregiver_pose_in, caregiver_pose_global, global_frame, tf2::durationFromSec(0.1));
            }
            catch(const tf2::TransformException& ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to transform caregiver pose to %s: %s", global_frame.c_str(), ex.what());
                return;
            }

            caregiver_last_pose = caregiver_pose_global.pose;

            double roll, pitch, yaw;
            tf2::Quaternion q(caregiver_last_pose.orientation.x, caregiver_last_pose.orientation.y, caregiver_last_pose.orientation.z, caregiver_last_pose.orientation.w);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            last_yaw = yaw;

            double dx0 = caregiver_last_pose.position.x - wheelchair_last_pose.position.x;
            double dy0 = caregiver_last_pose.position.y - wheelchair_last_pose.position.y;
            double actual_dist = std::sqrt((dx0*dx0) + (dy0*dy0));

            double desired_dist = offset_distance;
            double error = actual_dist - desired_dist;
            double target_offset = desired_dist - dynamic_gain * error;

            if(target_offset < min_offset)
            {
                target_offset = min_offset;
            }
            if(target_offset > max_offset)
            {
                target_offset = max_offset;
            }

            double gx = caregiver_last_pose.position.x - target_offset * std::cos(last_yaw);
            double gy = caregiver_last_pose.position.y - target_offset * std::sin(last_yaw);

            geometry_msgs::msg::PoseStamped result;
            result.header.stamp = now;
            result.header.frame_id = global_frame;

            result.pose.position.x = gx;
            result.pose.position.y = gy;
            result.pose.position.z = caregiver_last_pose.position.z;

            tf2::Quaternion q_target;
            q_target.setRPY(0.0, 0.0, last_yaw);
            result.pose.orientation = tf2::toMsg(q_target);

            last_pub_time = now;
            pub->publish(result);
        }

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CaregiverPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}