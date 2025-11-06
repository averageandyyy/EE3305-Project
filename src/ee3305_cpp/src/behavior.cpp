#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    /** Publishes the raw occupancy grid data, and the global costmap that has inflation cost values. */
    class Behavior : public rclcpp::Node
    {
    private:
        // Node Instance Properties =============================================================

        // Handles
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_request_; // should use ROS services.
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_plan_;

        // Data from Subscriptions
        double goal_x_; // read only; written only by subscriber(s)
        double goal_y_; // read only; written only by subscriber(s)
        double rbt_x_;  // read only; written only by subscriber(s)
        double rbt_y_;  // read only; written only by subscriber(s)

        // Parameters
        double frequency_;      // read only; written only in constructor
        double plan_frequency_; // read only; written only in constructor

        // Other Instance Variables
        bool received_goal_coords_;
        bool received_rbt_coords_;
        bool goal_reached_;

    public:
        explicit Behavior(std::string node_name = "behavior")
            : Node(node_name)
        {
            // Node Constructor =============================================================

            // Parameters: Declare (remember to get after declare)
            this->declare_parameter<double>("frequency", 10);
            this->declare_parameter<double>("plan_frequency", 2);

            // Parameters: Get values (remember to declare first)
            this->frequency_ = this->get_parameter("frequency").get_value<double>();
            this->plan_frequency_ = this->get_parameter("plan_frequency").get_value<double>();

            // Handles: Topic Subscribers
            // !TODO: Goal pose subscriber

            // !TODO: Odometry subscriber

            // Handles: Topic Publishers
            // !TODO: Path request publisher

            // Handles: Timers
            this->timer_ = this->create_timer(
                1.0s / this->frequency_, std::bind(&Behavior::callbackTimer_, this));

            this->timer_plan_ = this->create_timer(
                1.0s / this->plan_frequency_, std::bind(&Behavior::callbackTimerPlan_, this));

            // Other Instance Variables
            this->received_goal_coords_ = false;
            this->received_rbt_coords_ = false;
            this->goal_reached_ = true;
        }

    private:
        // Callbacks =============================================================

        // Goal pose subscriber callback.
        void callbackSubGoalPose_(geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->received_goal_coords_ = true;
            this->goal_reached_ = true; // "cancel" the goal and trigger the if condition in timer.

            // !TODO: Copy to goal_x_, goal_y_.
            this->goal_x_ = msg->pose.position.x;

            RCLCPP_INFO(this->get_logger(),
                        "Received New Goal @ (%7.3f, %7.3f).",
                        this->goal_x_, this->goal_y_);
        }

        // Odometry subscriber callback.
        void callbackSubOdom_(nav_msgs::msg::Odometry::SharedPtr msg)
        {
            this->received_rbt_coords_ = true;

            // !TODO: Copy to rbt_x_, rbt_y_.
            this->rbt_x_ = msg->pose.pose.orientation.y;
        }

        // Callback for timer.
        // Normally the decisions of the robot system are made here, and this callback is dramatically simplified.
        // The callback contains some example code for waypoint detection.
        void callbackTimer_()
        {
            if (!this->received_goal_coords_ || !this->received_rbt_coords_)
                return; // silently return if none of the coords are received from the subscribers.

            double dx = this->goal_x_ - this->rbt_x_;
            double dy = this->goal_y_ - this->rbt_y_;

            bool goal_is_close = std::hypot(dx, dy) < 0.1;

            if (goal_is_close && !this->goal_reached_)
            { // reached the goal pose
                RCLCPP_INFO(this->get_logger(),
                            "Reached Goal @ (%7.3f, %7.3f).",
                            this->goal_x_, this->goal_y_);
                this->goal_reached_ = true;
            }
            else if (!goal_is_close && this->goal_reached_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Going to Goal @ (%7.3f, %7.3f).",
                            this->goal_x_, this->goal_y_);
                this->goal_reached_ = false;
            }
        }

        // Callback for publishing path requests between clicked_point (goal) and robot position.
        // Normally path requests are implented with ROS2 service, and the service is called in the main timer.
        // To keep things simple for this course, we use only ROS2 tpics.
        void callbackTimerPlan_()
        {
            if (!this->received_goal_coords_ || !this->received_rbt_coords_)
                return; // silently return if none of the coords are received from the subscribers

            // Create a new message for publishing
            nav_msgs::msg::Path msg_path_request;
            msg_path_request.header.stamp = this->get_clock()->now();
            msg_path_request.header.frame_id = "map";

            // !TODO: write the robot coordinates
            geometry_msgs::msg::PoseStamped rbt_pose;
            rbt_pose.pose.position.x = 0.0;

            // !TODO: write the goal coordinates

            // !TODO: fill up the array containing the robot coordinates at [0] and goal coordinates at [1]
            msg_path_request.poses.push_back(rbt_pose);

            // Publish the message
            RCLCPP_INFO(this->get_logger(),
                        "Sending Path Planning Request from Rbt @ (%7.3f, %7.3f) to Goal @ (%7.3f, %7.3f)",
                        rbt_x_, rbt_y_, goal_x_, goal_y_);
            this->pub_path_request_->publish(msg_path_request);
        }
    };
}

// Main Boiler Plate =============================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee3305::Behavior>());
    rclcpp::shutdown();
    return 0;
}