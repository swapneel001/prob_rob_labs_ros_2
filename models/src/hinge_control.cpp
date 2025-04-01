#include "hinge_control.h"

namespace gazebo
{

    void HingeControl::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        if (!model) {
            RCLCPP_ERROR(rclcpp::get_logger("HingeControl"),
                         "Model pointer is NULL");
            return;
        }

        model_ = model;
        joint_ = model_->GetJoint("hinge");

        if (!joint_) {
            RCLCPP_ERROR(rclcpp::get_logger("HingeControl"),
                         "Failed to find hinge_joint");
            return;
        }

        // Initialize ROS 2 node
        ros_node_ = gazebo_ros::Node::Get(sdf);

        // Create subscriber
        sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
            "/" + model_->GetName() + "/torque", 10,
            std::bind(&HingeControl::OnRosMsg, this, std::placeholders::_1));
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&HingeControl::Update, this));
        RCLCPP_INFO(ros_node_->get_logger(),
                    "HingeControl plugin loaded successfully!");
    }

    void HingeControl::OnRosMsg(const std_msgs::msg::Float64::SharedPtr msg)
    {
        torque_ = msg->data;
    }

    void HingeControl::Update()
    {
        joint_->SetForce(0, torque_);
    }

    GZ_REGISTER_MODEL_PLUGIN(HingeControl)

}  // namespace gazebo
