#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    class Controller : public rclcpp::Node
    {

    private:
        // Node Instance Properties =============================================================

        // Handles
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_lookahead_;
        rclcpp::TimerBase::SharedPtr timer_; // contains the timer that runs the main looping function at regular intervals.

        // Data from Subscribers
        std::vector<geometry_msgs::msg::PoseStamped> path_poses_; // read only; written only by subscriber(s)
        double rbt_x_;                                            // read only; written only by subscriber(s)
        double rbt_y_;                                            // read only; written only by subscriber(s)
        double rbt_yaw_;                                          // read only; written only by subscriber(s)

        // Parameters
        double frequency_;          // read only; written only in constructor
        double lookahead_distance_; // read only; written only in constructor
        double lookahead_lin_vel_;  // read only; written only in constructor
        double stop_thres_;         // read only; written only in constructor
        double max_lin_vel_;      // read only; written only in constructor
        double max_ang_vel_;      // read only; written only in constructor

        // Other Instance Variables
        bool received_path_;
        bool received_odom_;

    public:
        /** Constructor. Run only once when this node is first created in `main()`. */
        explicit Controller(std::string node_name = "controller")
            : Node(node_name)
        {
            // Node Constructor =============================================================

            // Declare Parameters
            this->declare_parameter<double>("frequency", 20);
            this->declare_parameter<double>("lookahead_distance", 0.3);
            this->declare_parameter<double>("lookahead_lin_vel", 0.1);
            this->declare_parameter<double>("stop_thres", 0.1);
            this->declare_parameter<double>("max_lin_vel", 0.2);
            this->declare_parameter<double>("max_ang_vel", 2.0);

            // Get Parameters
            this->frequency_ = this->get_parameter("frequency").get_value<double>();
            this->lookahead_distance_ = this->get_parameter("lookahead_distance").get_value<double>();
            this->lookahead_lin_vel_ = this->get_parameter("lookahead_lin_vel").get_value<double>();
            this->stop_thres_ = this->get_parameter("stop_thres").get_value<double>();
            this->max_lin_vel_ = this->get_parameter("max_lin_vel").get_value<double>();
            this->max_ang_vel_ = this->get_parameter("max_ang_vel").get_value<double>();

            // Handles: Topic Subscribers
            // !TODO: path subscriber

            // !TODO: odometry subscriber

            // Handles: Topic Publishers
            // !TODO: command velocities publisher

            // !TODO: lookahead point publisher

            // Handles: Timers
            this->timer_ = this->create_wall_timer(
                1.0s / this->frequency_, std::bind(&Controller::callbackTimer_, this));

            // Other Instance Variables
            this->received_odom_ = false;
            this->received_path_ = false;
        }

    private:
        // Callbacks =============================================================

        // Path subscriber callback
        void callbackSubPath_(nav_msgs::msg::Path::SharedPtr msg)
        {
            if (msg->poses.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received path message is empty!");
                return; // do not update the path if no path is returned. This will ensure the copied path contains at least one point when the first non-empty path is received.
            }

            // !TODO: copy the array from the path
            this->path_poses_ = {};

            this->received_path_ = true;
        }

        // Odometry subscriber callback
        void callbackSubOdom_(nav_msgs::msg::Odometry::SharedPtr msg)
        {
            // !TODO: write robot pose to rbt_x_, rbt_y_, rbt_yaw_
            this->rbt_x_ = msg->pose.pose.position.x;
            
            const auto &q = msg->pose.pose.orientation;
            this->rbt_yaw_ = q.w;

            this->received_odom_ = true;
        }

        // Gets the lookahead point's coordinates based on the current robot's position and planner's path
        // Make sure path and robot positions are already received, and the path contains at least one point.
        std::pair<double, double> getLookaheadPoint_()
        {
            // Find the point along the path that is closest to the robot

            // From the closest point, iterate towards the goal and find the first point that is at least a lookahead distance away.
            // Return the goal point if no such lookahead point can be found
            size_t lookahead_idx = this->path_poses_.size() - 1;

            // Get the lookahead coordinates
            geometry_msgs::msg::PoseStamped lookahead_pose = this->path_poses_[lookahead_idx];
            double lookahead_x = lookahead_pose.pose.position.x;
            double lookahead_y = lookahead_pose.pose.position.y;

            // Publish the lookahead coordinates
            geometry_msgs::msg::PoseStamped msg_lookahead;
            msg_lookahead.header.stamp = this->get_clock()->now();
            msg_lookahead.header.frame_id = "map";
            msg_lookahead.pose.position.x = lookahead_x;
            msg_lookahead.pose.position.y = lookahead_y;
            this->pub_lookahead_->publish(msg_lookahead);

            // Return the coordinates
            return {lookahead_x, lookahead_y};
        }

        
        // Implement the pure pursuit controller here
        void callbackTimer_()
        {
            if (!this->received_odom_ || !this->received_path_)
                return; // return silently if path or odom is not received.

            // get lookahead point
            auto [lookahead_x, lookahead_y] = this->getLookaheadPoint_();

            // get distance to lookahead point (not to be confused with lookahead_distance)

            // stop the robot if close to the point.

            // get curvature

            // calculate velocities
            double lin_vel = 0.0;
            double ang_vel = 0.0 * lookahead_x * lookahead_y;

            // saturate velocities. The following can result in the wrong curvature,
            // but only when the robot is travelling too fast (which should not occur if well tuned).
            lin_vel = 0.0; 
            ang_vel = 0.0; 

            // publish velocities
            geometry_msgs::msg::TwistStamped msg_cmd_vel;
            msg_cmd_vel.header.stamp = this->get_clock()->now();
            msg_cmd_vel.twist.linear.x = lin_vel;
            msg_cmd_vel.twist.angular.z = ang_vel;
            this->pub_cmd_vel_->publish(msg_cmd_vel);
        }
    };
}

// Main Boiler Plate =============================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee3305::Controller>());
    rclcpp::shutdown();
    return 0;
}