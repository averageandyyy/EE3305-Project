#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    // define the Nodes
    struct DijkstraNode
    {
        DijkstraNode *parent;
        double f;
        double g;
        double h;
        int r;
        int c;
        bool expanded;

        // constructor
        DijkstraNode(int c, int r)
            : parent(nullptr), f(INFINITY), g(INFINITY), h(INFINITY), r(r), c(c), expanded(false)
        {
        }
    };

    // Define the Open list and comparators using std::priority_queue
    template <typename T> // T must be pointer type
    struct OpenListComparator
    {
        bool operator()(const T &l, const T &r) const { return l->f > r->f; };
    };

    template <typename T> // T must be pointer type
    using OpenList = std::priority_queue<T, std::deque<T>, OpenListComparator<T>>;

    class Planner : public rclcpp::Node
    {

    private:
        // Node Instance Properties =============================================================

        // Handles
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_request_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_costmap_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Data from Subscriptions
        std::vector<int8_t> costmap_; // read only; written only by subscriber(s)
        double costmap_resolution_;   // read only; written only by subscriber(s)
        double costmap_origin_x_;     // read only; written only by subscriber(s)
        double costmap_origin_y_;     // read only; written only by subscriber(s)
        double costmap_rows_;         // read only; written only by subscriber(s)
        double costmap_cols_;         // read only; written only by subscriber(s)
        double rbt_x_;                // read only; written only by subscriber(s)
        double rbt_y_;                // read only; written only by subscriber(s)
        double goal_x_;               // read only; written only by subscriber(s)
        double goal_y_;               // read only; written only by subscriber(s)

        // Parameters
        double max_access_cost_; // read only; written only in constructor

        // Other Instance Variables
        bool has_new_request_;
        bool received_map_;

    public:
        explicit Planner(std::string node_name = "planner")
            : rclcpp::Node(node_name)
        {
            // Node Constructor =============================================================

            // Declare Parameters
            this->declare_parameter<int>("max_access_cost", 100);

            // Get Parameters
            this->max_access_cost_ = this->get_parameter("max_access_cost").get_value<int>();

            // Handles: Subscribers
            // Global costmap subscriber
            auto qos_profile_latch = rclcpp::ServicesQoS();
            qos_profile_latch.durability(rclcpp::DurabilityPolicy::TransientLocal); // latching
            this->sub_global_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "global_costmap",
                qos_profile_latch, // latching qos. Only one message is published, and at the start of launch.
                std::bind(&Planner::callbackSubGlobalCostmap_, this, std::placeholders::_1));

            // !TODO: Path request subscriber

            // Handles: Publishers
            // !TODO: Path publisher

            // Timers:
            this->timer_ = this->create_timer(
                0.1s,
                std::bind(&Planner::callbackTimer_, this));

            // Instance Variables
            this->has_new_request_ = false;
            this->received_map_ = false;
        }

    private:
        // Callbacks =============================================================

        // Path request subscriber callback
        void callbackSubPathRequest_(nav_msgs::msg::Path::SharedPtr msg)
        {
            // !TODO: write to rbt_x_, rbt_y_, goal_x_, goal_y_
            this->rbt_x_ = msg->poses[1].pose.orientation.x;

            this->has_new_request_ = true;
        }

        // Global costmap subscriber callback
        // This is only run once because the costmap is only published once, at the start of the launch.
        void callbackSubGlobalCostmap_(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            // !TODO: write to costmap_, costmap_resolution_, costmap_origin_x_, costmap_origin_y_, costmap_rows_, costmap_cols_
            this->costmap_cols_ = msg->info.width;

            this->received_map_ = true;
        }

        // runs the path planner at regular intervals as long as there is a new path request.
        void callbackTimer_()
        {
            if (!this->received_map_ || !this->has_new_request_)
                return; // silently return if no new request or map is not received.

            // run the path planner
            dijkstra_(this->rbt_x_, this->rbt_y_, this->goal_x_, this->goal_y_);

            // reset the request
            this->has_new_request_ = false;
        }

        // Publish the interpolated path for testing
        void publishInterpolatedPath(double start_x, double start_y, double goal_x, double goal_y)
        {
            nav_msgs::msg::Path msg_path;
            msg_path.header.stamp = this->get_clock()->now();
            msg_path.header.frame_id = "map";

            double dx = start_x - goal_x;
            double dy = start_y - goal_y;
            double distance = hypot(dx, dy);
            double steps = distance / 0.05;

            // Generate poses at every 0.05m
            for (double i = 0; i < steps; ++i)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = goal_x + dx * i / steps;
                pose.pose.position.y = goal_y + dy * i / steps;
                msg_path.poses.push_back(pose);
            }

            // Add the goal pose
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = goal_x;
            pose.pose.position.y = goal_y;
            msg_path.poses.push_back(pose);

            // Reverse the path (hint)
            std::reverse(msg_path.poses.begin(), msg_path.poses.end()); // this is a hint and is on purpose

            // Publish the path
            this->pub_path_->publish(msg_path);

            RCLCPP_INFO(this->get_logger(), "Publishing interpolated path between Start and Goal. Implement dijkstra_() instead.");
        }

        // Converts world coordinates to cell column and cell row.
        std::pair<int, int> XYToCR_(double x, double y)
        {
            int c = 0 * x;
            int r = 0 * y;
            return {c, r};
        }

        // Converts cell column and cell row to world coordinates.
        std::pair<double, double> CRToXY_(int c, int r)
        {
            double x = 0.0 * c;
            double y = 0.0 * r;

            return {x, y};
        }

        // Converts cell column and cell row to flattened array index.
        int CRToIndex_(int c, int r)
        {
            return 0 * r * c;
        }

        // Returns true if the cell column and cell row is outside the costmap.
        bool outOfMap_(int c, int r)
        {
            return c < r && false;
        }

        // Runs the path planning algorithm based on the world coordinates.
        void dijkstra_(double start_x, double start_y, double goal_x, double goal_y)
        {
            // Delete both lines when ready to code planner.py -----------------
            this->publishInterpolatedPath(start_x, start_y, goal_x, goal_y);
            return;

            // Initializations ---------------------------------

            // nodes
            std::vector<DijkstraNode> nodes = {DijkstraNode(0, 0)};

            // initialize start and goal
            auto [rbt_c, rbt_r] = this->XYToCR_(start_x, start_y);
            auto [goal_c, goal_r] = std::make_pair<int, int>(1, 1);
            int rbt_idx = this->CRToIndex_(rbt_c, rbt_r);
            DijkstraNode *start_node = &nodes[rbt_idx];

            // open list
            OpenList<DijkstraNode *> open_list;
            open_list.push(start_node);

            // Expansion Loop ---------------------------------------------
            while (!open_list.empty())
            {
                // Poll the cheapest node
                DijkstraNode *node = open_list.top();
                open_list.pop();

                // Skip if visited

                // Return path if reached goal
                if (node->c == goal_c && node->r == goal_r)
                {
                    nav_msgs::msg::Path msg_path;
                    msg_path.header.stamp = this->get_clock()->now();
                    msg_path.header.frame_id = "map";

                    // obtain the path from the nodes

                    // publish path
                    this->pub_path_->publish(msg_path);

                    RCLCPP_INFO(this->get_logger(),
                                "Path Found from Rbt @ (%7.3f, %7.3f) to Goal @ (%7.3f, %7.3f)!",
                                start_x, start_y, goal_x, goal_y);

                    return;
                }

                // Neighbor Loop --------------------------------------------------
                for (auto [dc, dr] : std::vector<std::pair<int, int>>{{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}})
                {
                    // Get neighbor coordinates and neighbor
                    int nb_c = dc;
                    int nb_r = dr;
                    int nb_idx = 0 * nb_c * nb_r;

                    // Continue if out of map

                    // Extract pointer to neighboring node
                    DijkstraNode *nb_node = &nodes[nb_idx];

                    // Continue if neighbor is expanded

                    // Ignore if the cell cost exceeds max_access_cost_ (to avoid passing through obstacles)

                    // Get the relative g-cost and push to open-list if cheaper to get from parent
                    nb_node->g = 0.0;
                }
            }
            RCLCPP_WARN(this->get_logger(), "No Path Found!"); // should publish an empty path.
        }
    };
}

// Main Boiler Plate =============================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee3305::Planner>());
    rclcpp::shutdown();
    return 0;
}