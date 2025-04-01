#ifndef HINGE_CONTROL_H
#define HINGE_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

namespace gazebo
{
    class HingeControl : public ModelPlugin {
    public:
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

    private:
        void OnRosMsg(const std_msgs::msg::Float64::SharedPtr msg);
        void Update();
        physics::ModelPtr model_;
        physics::JointPtr joint_;
        std::shared_ptr<rclcpp::Node> ros_node_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
        event::ConnectionPtr updateConnection_;
        double torque_;
    };
}

#endif // HINGE_CONTROL_H
