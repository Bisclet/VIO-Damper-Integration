#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class OdomStaticAligner : public rclcpp::Node
{
public:
  OdomStaticAligner()
  : Node("odom_static_aligner")
  {
    basalt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/basalt/odom", 10,
      std::bind(&OdomStaticAligner::basaltCallback, this, std::placeholders::_1));

    mavros_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", 10,
      std::bind(&OdomStaticAligner::mavrosCallback, this, std::placeholders::_1));

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    RCLCPP_INFO(get_logger(), "Waiting for initial odometry messages...");
  }

private:
  void basaltCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!basalt_pose_) {
      basalt_pose_ = msg->pose.pose;
      tryPublish();
    }
  }

  void mavrosCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!mavros_pose_) {
      mavros_pose_ = msg->pose.pose;
      tryPublish();
    }
  }

  void tryPublish()
  {
    if (published_) return;
    if (!basalt_pose_ || !mavros_pose_) return;

    tf2::Transform T_basalt, T_mavros;

    tf2::fromMsg(*basalt_pose_, T_basalt);
    tf2::fromMsg(*mavros_pose_, T_mavros);

    tf2::Transform T_offset = T_mavros * T_basalt.inverse();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = "mavros_odom";
    tf_msg.child_frame_id = "basalt_aligned";

    tf_msg.transform = tf2::toMsg(T_offset);

    broadcaster_->sendTransform(tf_msg);

    published_ = true;

    RCLCPP_INFO(get_logger(), "Static transform published!");
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr basalt_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavros_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

  std::optional<geometry_msgs::msg::Pose> basalt_pose_;
  std::optional<geometry_msgs::msg::Pose> mavros_pose_;

  bool published_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomStaticAligner>());
  rclcpp::shutdown();
  return 0;
}
