#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class GoalPointSubNode : public rclcpp::Node
{
public:
  GoalPointSubNode()
  : Node("goal_point_sub")
  {
    this->declare_parameter("topic_name", "/way_point");
    this->declare_parameter("save_file", "src/kilo-map/legkilo/result/goal_point/goal_point.txt");

    topic_name_ = this->get_parameter("topic_name").as_string();
    save_file_ = this->get_parameter("save_file").as_string();

    reset_file();

    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      topic_name_, 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        save_waypoint(*msg);
      });

    RCLCPP_INFO(this->get_logger(), "Listening to topic '%s' and saving to '%s'", topic_name_.c_str(), save_file_.c_str());
  }

private:
  void reset_file()
  {
    const std::filesystem::path file_path(save_file_);
    if (file_path.has_parent_path()) {
      std::filesystem::create_directories(file_path.parent_path());
    }

    std::ofstream out(file_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open save file for overwrite: %s", save_file_.c_str());
      return;
    }

    out << "# waypoint saved by goal_point_sub\n";
    out.close();
    RCLCPP_INFO(this->get_logger(), "Reset waypoint file: %s", save_file_.c_str());
  }

  void save_waypoint(const geometry_msgs::msg::PointStamped& msg)
  {
    static int count = 0;
    const double eps = 1e-9;
    const bool same_point = has_last_waypoint_ &&
      std::fabs(last_x_ - msg.point.x) < eps &&
      std::fabs(last_y_ - msg.point.y) < eps &&
      std::fabs(last_z_ - msg.point.z) < eps;

    if (same_point) {
      return;
    }

    const std::filesystem::path file_path(save_file_);
    if (file_path.has_parent_path()) {
      std::filesystem::create_directories(file_path.parent_path());
    }

    std::ofstream out(file_path, std::ios::out | std::ios::app);
    if (!out.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open save file for writing: %s", save_file_.c_str());
      return;
    }

    out << std::fixed << std::setprecision(6);
    out << "id: " << count << "\n";
    out << "x: " << msg.point.x << "\n";
    out << "y: " << msg.point.y << "\n";
    out << "z: " << msg.point.z << "\n";
    out.close();
    count++;

    last_x_ = msg.point.x;
    last_y_ = msg.point.y;
    last_z_ = msg.point.z;
    has_last_waypoint_ = true;

    RCLCPP_INFO(this->get_logger(), "Saved waypoint: x=%f, y=%f, z=%f", msg.point.x, msg.point.y, msg.point.z);
  }

  std::string topic_name_;
  std::string save_file_;
  bool has_last_waypoint_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_z_{0.0};
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPointSubNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
