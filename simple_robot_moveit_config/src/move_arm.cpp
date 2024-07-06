#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "geometric_shapes/shape_operations.h"
#include "shape_msgs/msg/mesh.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_arm");

// Function to plan to a target POSE, i.e. target specified in spacial coordinates x, y ,z and orientation w x y z
void planToTargetPose(
    moveit::planning_interface::MoveGroupInterface &move_group,
    geometry_msgs::msg::Pose &target_pose)
{
    move_group.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
    }
}

// Function to plan to a joint space target, to be used for openning and clossing the gripper
void planToJointSpaceTarget(
    moveit::planning_interface::MoveGroupInterface &move_group,
    std::vector<double> joint_target)
{
    move_group.setJointValueTarget(joint_target);

    // Create a plan to that joint value target
    auto const [success, plan] = [&move_group]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_arm_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    static const std::string PLANNING_GROUP_ARM = "arm_group";

    moveit::planning_interface::MoveGroupInterface::Options arm_options(PLANNING_GROUP_ARM);
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_arm = MoveGroupInterface(move_group_node, arm_options);


    auto pose1 = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3;
        msg.position.y = 0.2;
        msg.position.z = 2.5;
        return msg;
    }();

    auto pose2 = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3;
        msg.position.y = -0.2;
        msg.position.z = 2.5;
        return msg;
    }();

// Pose with orientation sppecified
    auto pose3 = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 1.035;
        msg.position.y = 0.5;
        msg.position.z = 1.054;
        msg.orientation.x = -0.0459;
        msg.orientation.y = 0.9975;
        msg.orientation.z = 0.0024;
        msg.orientation.w = 0.0538;
        return msg;
    }();

    RCLCPP_INFO(LOGGER, "Moving to POSE1");
    planToTargetPose(move_group_arm, pose1);
    RCLCPP_INFO(LOGGER, "Moving to POSE2");
    planToTargetPose(move_group_arm, pose2);
    RCLCPP_INFO(LOGGER, "Moving to POSE3");
    planToTargetPose(move_group_arm, pose3);

    RCLCPP_INFO(LOGGER, "Done!");

    rclcpp::shutdown();
    return 0;
}