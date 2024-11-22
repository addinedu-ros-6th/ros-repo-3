#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread> // sleep_for를 위해 필요

void move_to_pose(const std::shared_ptr<rclcpp::Node>& node, 
                moveit::planning_interface::MoveGroupInterface& move_group, 
                const std::string& pose_name)
{
    move_group.setNamedTarget(pose_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.execute(plan);
        RCLCPP_INFO(node->get_logger(), "Moved to '%s' pose successfully.", pose_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan to '%s' pose.", pose_name.c_str());
    }
}

int main(int argc, char** argv)
{
    // ROS 2 노드 초기화
    rclcpp::init(argc, argv);
    
    // 노드 생성
    auto node = rclcpp::Node::make_shared("start_push");

    // MoveGroupInterface 객체 생성 (arm)
    moveit::planning_interface::MoveGroupInterface move_group(node, "pollibot_arm");

    // MoveGroupInterface 객체 생성 (pusher)
    moveit::planning_interface::MoveGroupInterface pusher_move_group(node, "pusher");

    // 'ready' 자세로 이동
    // move_to_pose(node, move_group, "ready");

    // 'pollination' 자세로 이동
    // move_to_pose(node, move_group, "pollination");

    // 'push' 자세로 이동
    // std::this_thread::sleep_for(std::chrono::seconds(1)); // 1초 대기
    move_to_pose(node, pusher_move_group, "push");

    // 'pull' 자세로 이동
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 1초 대기
    move_to_pose(node, pusher_move_group, "pull");
    
    // ROS 2 노드를 계속 실행하여 종료되지 않도록 유지
    // rclcpp::spin(node);

    // ROS 2 노드 종료
    rclcpp::shutdown();
    return 0;
}