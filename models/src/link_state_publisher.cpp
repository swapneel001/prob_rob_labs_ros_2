#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gazebo_msgs/msg/link_states.hpp>

namespace gazebo
{
class LinkStatePublisherPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr) override
  {
    world_ = _world;
    node_ = rclcpp::Node::make_shared("link_state_publisher");
    publisher_ = node_->create_publisher<gazebo_msgs::msg::LinkStates>("/gazebo/link_states", 10);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LinkStatePublisherPlugin::OnUpdate, this));
    
    rclcpp_thread_ = std::thread([this]() {
      rclcpp::spin(node_);
    });

    RCLCPP_INFO(node_->get_logger(), "LinkStatePublisherPlugin loaded.");
  }

  void OnUpdate()
  {
    gazebo_msgs::msg::LinkStates msg;

    for (const auto& model : world_->Models())
    {
      for (const auto& link : model->GetLinks())
      {
        msg.name.push_back(link->GetScopedName());

        ignition::math::Pose3d pose = link->WorldPose();
        geometry_msgs::msg::Pose ros_pose;
        ros_pose.position.x = pose.Pos().X();
        ros_pose.position.y = pose.Pos().Y();
        ros_pose.position.z = pose.Pos().Z();
        ros_pose.orientation.x = pose.Rot().X();
        ros_pose.orientation.y = pose.Rot().Y();
        ros_pose.orientation.z = pose.Rot().Z();
        ros_pose.orientation.w = pose.Rot().W();
        msg.pose.push_back(ros_pose);

        ignition::math::Vector3d lin_vel = link->WorldLinearVel();
        ignition::math::Vector3d ang_vel = link->WorldAngularVel();
        geometry_msgs::msg::Twist ros_twist;
        ros_twist.linear.x = lin_vel.X();
        ros_twist.linear.y = lin_vel.Y();
        ros_twist.linear.z = lin_vel.Z();
        ros_twist.angular.x = ang_vel.X();
        ros_twist.angular.y = ang_vel.Y();
        ros_twist.angular.z = ang_vel.Z();
        msg.twist.push_back(ros_twist);
      }
    }

    publisher_->publish(msg);
  }

  ~LinkStatePublisherPlugin() override
  {
    rclcpp::shutdown();
    if (rclcpp_thread_.joinable())
      rclcpp_thread_.join();
  }

private:
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<gazebo_msgs::msg::LinkStates>::SharedPtr publisher_;
  std::thread rclcpp_thread_;
};

GZ_REGISTER_WORLD_PLUGIN(LinkStatePublisherPlugin)
}
