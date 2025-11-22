#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include<iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class ModeManager : public rclcpp::Node
{
    public:
        ModeManager():Node("mode_manager_node"), is_following(false), running(true)
        {
            // Parameters
            navigation_mode_topic = this->declare_parameter<std::string>("navigation_mode_topic", "/navigation_mode");
            global_frame = this->declare_parameter<std::string>("global_frame", "odom");

            // Publisher
            mode_pub = this->create_publisher<std_msgs::msg::String>(navigation_mode_topic, 10);

            // Service Client
            follow_client = this->create_client<std_srvs::srv::SetBool>("set_following_enabled");

            // Nav2 Action Client
            nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

            key_thread = std::thread(&ModeManager::keyboardLoop, this);

            RCLCPP_INFO(this->get_logger(), "ModeManager initialized.");
            publishMode();
        }

        ~ModeManager()
        {
            running = false;
            if(key_thread.joinable())
            {
                key_thread.join();
            }
        }

        private:
            void publishMode()
            {
                auto msg = std_msgs::msg::String();
                msg.data = is_following ? "FOLLOW" : "NAV";
                mode_pub -> publish(msg);
            }

            void setFollowEnabled(bool enabled)
            {
                if(!follow_client -> wait_for_service(1s))
                {
                    RCLCPP_WARN(this->get_logger(), "FOLLOW service not available");
                    return;
                }

                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                req -> data = enabled;
                follow_client -> async_send_request(req);
            }

            void cancelNavGoal()
            {
                if(!nav_client->wait_for_action_server(2s))
                {
                    RCLCPP_WARN(this->get_logger(), "navigate_to_pose action server not ready (cancel skipped)");
                    return;
                }
                nav_client -> async_cancel_all_goals();
            }

            void sendFollowBTGoal()
            {
                if (!nav_client->wait_for_action_server(2s))
                {
                    RCLCPP_ERROR(this->get_logger(), "navigate_to_pose action server not available");
                    return;
                }

                nav2_msgs::action::NavigateToPose::Goal goal;

                goal.pose.header.frame_id = global_frame;
                goal.pose.header.stamp = this->now();

                // Dummy value. It will be overwritten by GoalUpdater using /caregiver_pos topic
                goal.pose.pose.position.x = 0.0;
                goal.pose.pose.position.y = 0.0;
                goal.pose.pose.orientation.w = 1.0;

                rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
                options.goal_response_callback = [this](std::shared_ptr<GoalHandleNavigateToPose> handle)
                {
                    if (!handle)
                    {
                        RCLCPP_ERROR(this->get_logger(), "[FOLLOW BT] Goal rejected");
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "[FOLLOW BT] Goal accepted");
                    }
                };

                options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result)
                {
                    RCLCPP_INFO(this->get_logger(), "[FOLLOW BT] Finished with result code: %d", static_cast<int>(result.code));
                };

                nav_client -> async_send_goal(goal, options);
            }

            void keyCallback(const std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data.empty()) return;
                char key = msg->data[0];

                // "f" -> following mode
                if (key == 'f')
                {
                    if (is_following)
                    {
                        RCLCPP_INFO(this->get_logger(), "Already in FOLLOW mode.");
                        return;
                    }

                    RCLCPP_INFO(this->get_logger(), "[Key f] NAV → FOLLOW");
                    setFollowEnabled(true);
                    std::this_thread::sleep_for(150ms);
                    sendFollowBTGoal();

                    is_following = true;
                    publishMode();
                }

                // "s" -> Return to Nav
                else if (key == 's')
                {
                    if(!is_following)
                    {
                        RCLCPP_INFO(this->get_logger(), "Already in NAV mode.");
                        return;
                    }

                    RCLCPP_INFO(this->get_logger(), "[Key s] FOLLOW → NAV");
                    
                    setFollowEnabled(false);
                    cancelNavGoal();

                    is_following = false;
                    publishMode();
                }
            }

            void keyboardLoop()
            {
                RCLCPP_INFO(this->get_logger(), "Keyboard input enabled. Press 'f' or 's' + Enter.");

                while(rclcpp::ok() && running)
                {
                   std::cout << "Select the mode " << std::flush;

                   std::string line;
                    if (!std::getline(std::cin, line)) 
                    {
                        RCLCPP_WARN(this->get_logger(), "stdin closed, stopping keyboard loop.");
                        break;
                    }

                    if (line.empty()) 
                    {
                        continue;
                    }

                    char c = line[0];

                    if (c != 'f' && c != 's') 
                    {
                        RCLCPP_INFO(this->get_logger(), "Unknown key '%c'. Use 'f' (FOLLOW) or 's' (NAV).", c);
                        continue;
                    }

                    auto msg = std_msgs::msg::String();
                    msg.data = std::string(1, c);

                    keyCallback(std::make_shared<std_msgs::msg::String>(msg));
                }   
            }

            bool is_following;

            std::string navigation_mode_topic;
            std::string global_frame;

            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub;

            rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr follow_client;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client;     
            
            std::thread key_thread;
            bool running;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModeManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}