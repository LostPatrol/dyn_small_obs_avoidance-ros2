// ROS 2 TF and topic interfaces for the original clicked-point transformer.
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// Retain the original cached transform and initialization state.
tf2::Transform transform;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr transformed_pub;
bool has_transform = false;

// Apply the original cached point transform when available.
void waypointCb(const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg)
{
  if(has_transform)
  {
    tf2::Vector3 point_odom(msg->point.x, msg->point.y, msg->point.z);
    geometry_msgs::msg::PointStamped point_base_link;
    tf2::Vector3 point_stamped = transform * point_odom;
    point_base_link.point.x = point_stamped.x();
    point_base_link.point.y = point_stamped.y();
    point_base_link.point.z = point_stamped.z();
    transformed_pub->publish(point_base_link);
  }
  else
    RCLCPP_ERROR(rclcpp::get_logger("bebop_tf_listner"), "Transform not yet initialized");
}

// Initialize ROS 2 and cache the same transform as the original node.
int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("bebop_tf_listner");
  auto planner_sub = node->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 1, waypointCb);
  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer);
  transformed_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("/transformed/clicked_point", 1);
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()){

    try{
      auto transform_msg = buffer.lookupTransform("odom", "base_link", tf2::TimePointZero);
      tf2::fromMsg(transform_msg.transform, transform);
      RCLCPP_INFO(node->get_logger(), "Transform initialized");
      has_transform = true;
      break;
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(node->get_logger(), "%s",ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;
    }

    rate.sleep();
  }
  rclcpp::spin(node);
  // Release the global publisher before middleware static teardown.
  transformed_pub.reset();
  rclcpp::shutdown();
  return 0;
};
