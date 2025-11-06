#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace rb2301
{

    class MapLoader : public rclcpp::Node
    {
    private:
        // handles
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_global_costmap_;

        // topics vars
        nav_msgs::msg::OccupancyGrid msg_global_costmap_;
        nav_msgs::msg::OccupancyGrid msg_map_;

        // parameter vars
        std::string topic_global_costmap_;
        std::string topic_map_;
        std::string frame_id_;
        std::string filepath_;
        double inflation_radius_;
        double circumscribed_radius_;
        double cost_exponent_;
        int max_cost_;
        int min_cost_;

    public:
        explicit MapLoader(const std::string &name = "map_loader")
            : rclcpp::Node(name)
        {
            // Parameters
            this->max_cost_ = this->getParameter_<int>("max_cost", 254).as_int();
            this->min_cost_ = this->getParameter_<int>("min_cost", 1).as_int();
            this->inflation_radius_ = this->getParameter_<double>("inflation_radius", 0.4).as_double();
            this->circumscribed_radius_ = this->getParameter_<double>("circumscribed_radius", 0.3).as_double();
            this->cost_exponent_ = this->getParameter_<double>("cost_exponent", 20.0).as_double();
            this->topic_map_ = this->getParameter_<std::string>("topic_map", "map").as_string();
            this->topic_global_costmap_ = this->getParameter_<std::string>("topic_global_costmap", "global_costmap").as_string();
            this->filepath_ = this->getParameter_<std::string>("filepath", "").as_string();
            this->frame_id_ = this->getParameter_<std::string>("frame_id", "").as_string();

            // topics
            auto qos = rclcpp::ServicesQoS();
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            qos.durability(rclcpp::DurabilityPolicy::TransientLocal); // latch
            this->pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                this->topic_map_,
                qos);
            this->pub_global_costmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                this->topic_global_costmap_,
                qos);

        }

        bool publishMapAndCostMap()
        {
            if (this->readMap_())
            {
                // publish oc grid.
                this->pub_map_->publish(this->msg_map_);
                RCLCPP_INFO_STREAM(this->get_logger(), "Published occupancy grid to " << this->pub_map_->get_topic_name());

                // create inflation zones (global_costmap) and publish it.
                this->createGlobalCostMap_();
                this->pub_global_costmap_->publish(this->msg_global_costmap_);
                RCLCPP_INFO_STREAM(this->get_logger(), "Published global costmap to " << this->pub_global_costmap_->get_topic_name());

                return true;
            }
            else
                return false;
        }

        private:
        void createGlobalCostMap_()
        {
            // generate inflation mask
            struct InflationMask
            {
                double distance;
                int col, row;
                InflationMask(double distance, int col, int row) : distance(distance), col(col), row(row) {}
            };
            std::vector<InflationMask> inflation_mask;

            {
                int window = std::ceil(this->inflation_radius_ / this->msg_map_.info.resolution);
                for (int col = -window; col <= window; ++col)
                {
                    for (int row = -window; row <= window; ++row)
                    {
                        double distance = std::hypot(col, row) * this->msg_map_.info.resolution;
                        if (distance < this->inflation_radius_ + 1e-8)
                            inflation_mask.emplace_back(distance, col, row);
                    }
                }
                std::sort(inflation_mask.begin(),
                          inflation_mask.end(),
                          [](const InflationMask &a, const InflationMask &b)
                          { return a.distance < b.distance; });
            }

            // prepare global costmap
            int num_cols = this->msg_map_.info.width;
            int num_rows = this->msg_map_.info.height;
            this->msg_global_costmap_.header.frame_id = this->frame_id_;
            this->msg_global_costmap_.info.resolution = this->msg_map_.info.resolution;
            this->msg_global_costmap_.info.width = num_cols;
            this->msg_global_costmap_.info.height = num_rows;
            this->msg_global_costmap_.info.origin.position.x = this->msg_map_.info.origin.position.x;
            this->msg_global_costmap_.info.origin.position.y = this->msg_map_.info.origin.position.y;
            this->msg_global_costmap_.info.origin.position.z = this->msg_map_.info.origin.position.z;
            this->msg_global_costmap_.info.origin.orientation.x = 0;
            this->msg_global_costmap_.info.origin.orientation.y = 0;
            this->msg_global_costmap_.info.origin.orientation.z = 0;
            this->msg_global_costmap_.info.origin.orientation.w = 1;
            this->msg_global_costmap_.data.resize(this->msg_map_.data.size());

            // generate inflation
            double cost_coeff = this->max_cost_ - this->min_cost_;
            double cost_radius = this->inflation_radius_ - this->circumscribed_radius_;
            for (int col = 0; col < num_cols; ++col)
            {
                for (int row = 0; row < num_rows; ++row)
                {
                    int8_t &global_costmap_data = this->msg_global_costmap_.data[row * num_cols + col];
                    global_costmap_data = this->min_cost_;

                    for (const InflationMask &mask : inflation_mask)
                    {
                        int nb_col = col + mask.col;
                        int nb_row = row + mask.row;

                        if (nb_col < 0 || nb_col >= num_cols || nb_row < 0 || nb_row >= num_rows)
                            continue; // neighboring cell to check must be in map.

                        // color the cost if a neighboring cell in the inflation radius is occupied.
                        if (this->msg_map_.data[nb_row * num_cols + nb_col] == 100) // occupied
                        {
                            if (mask.distance < this->circumscribed_radius_)
                            { // within circumscribed radius
                                global_costmap_data = this->max_cost_;
                            }
                            else
                            { // has to be within the inflation radius
                                // cost = (maxcost - mincost) * ((infr - x) / (infr - ccmr))^q + mincost
                                global_costmap_data = std::round(
                                    this->min_cost_ + cost_coeff * std::pow(
                                                                       (this->inflation_radius_ - mask.distance) / cost_radius,
                                                                       this->cost_exponent_));
                            }
                            break; // no need to search the mask anymore. The closest occupied cell has been found.
                        }
                    }
                }
            }
        }

        // partial mimic of Nav2's map loader. Assumes default values for map used.
        // returns true if there is a map. False otherwise.
        bool readMap_()
        {
            // try to read the files
            std::filesystem::path filepath_yaml = this->filepath_;
            if (filepath_yaml.empty())
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "No map because filepath is empty.");
                return false;
            }
            filepath_yaml.replace_extension("yaml");
            if (!std::filesystem::exists(filepath_yaml))
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "No map because '" << filepath_yaml << "' does not exist.");
                return false;
            }
            std::filesystem::path filepath_pgm = this->filepath_;
            filepath_pgm.replace_extension("pgm");
            if (!std::filesystem::exists(filepath_pgm))
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "No map because '" << filepath_pgm << "' does not exist.");
                return false;
            }

            // open yaml file and read only resolution, x, y, and z.
            std::ifstream file_yaml(filepath_yaml);
            double resolution, x, y, z;
            std::string tmp;
            while (std::getline(file_yaml, tmp))
            {
                std::istringstream line(tmp);
                std::string key;
                line >> key;
                if (key == "origin:")
                {
                    char c;
                    while (line >> c && c != '[')
                    {
                    }
                    line >> x >> tmp >> y >> tmp >> z;
                }
                else if (key == "resolution:")
                {
                    line >> resolution;
                }
            }

            // read the pgm file
            std::ifstream file_pgm(filepath_pgm);
            int num_cols, num_rows;
            file_pgm >> tmp >> num_cols >> num_rows >> tmp;

            // write to msg_map_
            this->msg_map_.header.frame_id = this->frame_id_;
            this->msg_map_.info.resolution = resolution;
            this->msg_map_.info.width = num_cols;
            this->msg_map_.info.height = num_rows;
            this->msg_map_.info.origin.position.x = x;
            this->msg_map_.info.origin.position.y = y;
            this->msg_map_.info.origin.position.z = z;
            this->msg_map_.info.origin.orientation.x = 0;
            this->msg_map_.info.origin.orientation.y = 0;
            this->msg_map_.info.origin.orientation.z = 0;
            this->msg_map_.info.origin.orientation.w = 1;

            // read the pgm file data
            this->msg_map_.data.resize(num_rows * num_cols);
            for (size_t row = num_rows; row > 0; --row)
            {
                for (int col = 0; col < num_cols; ++col)
                {
                    unsigned char pgm_data;
                    file_pgm >> pgm_data;
                    signed char &map_data = this->msg_map_.data[(row - 1) * num_cols + col];
                    if (pgm_data == 254)
                        map_data = 0;
                    else if (pgm_data == 0)
                        map_data = 100;
                    else // pgm_data must be 205
                        map_data = 50;
                }
            }

            return true;
        }

    private:
        template <typename T>
        rclcpp::Parameter getParameter_(const std::string &name, const T &default_value)
        {
            this->declare_parameter(name, default_value);
            rclcpp::Parameter parameter = this->get_parameter(name);
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "[Parameter] " << name << ": " << parameter);
            return parameter;
        }
    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rb2301::MapLoader> map_loader = std::make_shared<rb2301::MapLoader>();

    if (map_loader->publishMapAndCostMap())
        rclcpp::spin(map_loader);

    rclcpp::shutdown();
    return 0;
}