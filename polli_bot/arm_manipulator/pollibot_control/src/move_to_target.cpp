// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <thread>
// #include <chrono>

// int main(int argc, char** argv)
// {
//     // Initialize ROS 2
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("move_to_target");

//     // 비동기 스피너 초기화 및 시작
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     std::thread([&executor]() { executor.spin(); }).detach();

//    // MoveGroupInterface 생성
//     moveit::planning_interface::MoveGroupInterface move_group(node, "pollibot_arm");

//     // Reference frame 설정
//     move_group.setPoseReferenceFrame("base_link");

//     // Allow some time for connections to establish and messages to be received
//     rclcpp::sleep_for(std::chrono::seconds(1));


//     // Retrieve the current state of the robot with a timeout
//     // moveit::core::RobotStatePtr current_state;
//     // bool state_received = false;
//     // auto start = std::chrono::steady_clock::now();
//     // while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
//     // {
//     //     current_state = move_group.getCurrentState();
//     //     if (current_state)
//     //     {
//     //         state_received = true;
//     //         break;
//     //     }
//     //     rclcpp::spin_some(node);
//     //     rclcpp::sleep_for(std::chrono::milliseconds(100));
//     // }
//     // // Check if the current state was successfully retrieved
//     // if (!state_received)
//     // {
//     //     RCLCPP_ERROR(node->get_logger(), "로봇의 현재 상태를 가져오지 못했습니다.");
//     //     rclcpp::shutdown();
//     //     return 1;
//     // }

//     // // 로봇의 현재 상태 가져오기
//     auto current_state = move_group.getCurrentState(10.0);
//     if (!current_state)
//     {
//         RCLCPP_ERROR(node->get_logger(), "로봇의 현재 상태를 가져오지 못했습니다.");
//         rclcpp::shutdown();
//         return 1;
//     }

//     // 초기 관절 위치 설정
//     std::vector<double> initial_joint_positions = {2.35619, 2.00713, 2.35619, 2.35619};

//     // Get the joint model group
//     const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup("pollibot_arm");
//     if (!joint_model_group)
//     {
//         RCLCPP_ERROR(node->get_logger(), "'pollibot_arm' 조인트 모델 그룹을 찾을 수 없습니다.");
//         rclcpp::shutdown();
//         return 1;
//     }

//     // 조인트 모델 그룹의 변수 수 확인
//     size_t joint_count = joint_model_group->getVariableCount();
//     RCLCPP_INFO(node->get_logger(), "'pollibot_arm' 그룹의 조인트 수: %zu", joint_count);

//     // 초기 관절 위치 설정 (조인트 수에 맞게 조정)
//     if (initial_joint_positions.size() != joint_count)
//     {
//         RCLCPP_ERROR(node->get_logger(), "초기 관절 위치의 크기와 조인트 수가 일치하지 않습니다.");
//         rclcpp::shutdown();
//         return 1;
//     }

//     current_state->setJointGroupPositions(joint_model_group, initial_joint_positions);
    
//     // Get the joint model group
//     // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("pollibot_arm");

//     // Set the start state to the initial joint positions
//     move_group.setStartState(*current_state);

//     // 목표 위치를 설정합니다.
//     geometry_msgs::msg::Pose target_pose;
//     target_pose.position.x = 0.2;  // 원하는 x 좌표로 변경
//     target_pose.position.y = 0.0;  // 원하는 y 좌표로 변경
//     target_pose.position.z = 0.4;  // 원하는 z 좌표로 변경

//     // 원하는 방향으로의 쿼터니언 생성 (예: 엔드 이펙터가 아래를 향하도록)
//     tf2::Quaternion q;
//     q.setRPY(1.57, 0.22, -0.05);  // Roll, Pitch, Yaw 값을 설정합니다.
//     target_pose.orientation = tf2::toMsg(q);

//     // 현재 로봇 상태 복사
//     moveit::core::RobotStatePtr test_state(new moveit::core::RobotState(*current_state));

//     // IK 솔버를 사용하여 조인트 값 계산
//     bool found_ik = test_state->setFromIK(joint_model_group, target_pose);
//     if (found_ik)
//     {
//         RCLCPP_INFO(node->get_logger(), "IK 해를 성공적으로 찾았습니다.");
//     }
//     else
//     {
//         RCLCPP_ERROR(node->get_logger(), "IK 해를 찾을 수 없습니다.");
//         rclcpp::shutdown();
//         return 1;
//     }

//     // // 충돌 검사 비활성화
//     // move_group.setPlanningSceneMonitorActive(false);
//     // move_group.setPlanningScenePublishingFrequency(0.0);
//     // move_group.setPlanningTime(10.0);
//     // move_group.allowReplanning(true);
//     // move_group.setNumPlanningAttempts(10);
//     // move_group.setGoalTolerance(0.01);

//     // // 충돌 검사 비활성화 옵션 설정
//     // move_group.setAvoidCollisions(false);

//     // // 목표 자세 설정
//     // move_group.setPoseTarget(target_pose);

//     // // 경로 계획 및 실행
//     // moveit::planning_interface::MoveGroupInterface::Plan plan;
//     // bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     // if (success)
//     // {
//     //     RCLCPP_INFO(node->get_logger(), "계획이 성공적으로 수립되었습니다. 실행 중...");
//     //     move_group.execute(plan);
//     //     RCLCPP_INFO(node->get_logger(), "이동이 성공적으로 완료되었습니다.");
//     // }
//     // else
//     // {
//     //     RCLCPP_ERROR(node->get_logger(), "목표 자세로의 계획에 실패했습니다.");
//     // }

//     // 프로그램 종료 시
//     executor.cancel();
//     rclcpp::shutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_target");

    // 비동기 스피너 초기화 및 시작
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // MoveGroupInterface 생성
    moveit::planning_interface::MoveGroupInterface move_group(node, "pollibot_arm");

    // MoveGroupInterface가 제대로 생성되었는지 확인
    if (!move_group.getRobotModel())
    {
        RCLCPP_ERROR(node->get_logger(), "MoveGroupInterface를 초기화하는 데 실패했습니다.");
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }

    // 기준 프레임 설정
    move_group.setPoseReferenceFrame("base_link");

    // 연결이 설정되고 메시지가 수신될 시간을 허용
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 로봇의 현재 상태 가져오기
    auto current_state = move_group.getCurrentState(10.0);
    if (!current_state)
    {
        RCLCPP_ERROR(node->get_logger(), "로봇의 현재 상태를 가져오지 못했습니다.");
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }

    // 초기 관절 위치 설정
    std::vector<double> initial_joint_positions = {2.35619, 2.00713, 2.35619, 2.35619};

    // 조인트 모델 그룹 가져오기
    const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup("pollibot_arm");
    if (!joint_model_group)
    {
        RCLCPP_ERROR(node->get_logger(), "'pollibot_arm' 조인트 모델 그룹을 찾을 수 없습니다.");
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }

    // Retrieve the joint names for the 'pollibot_arm' group
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // 현재 로봇 상태의 관절 위치 출력
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);

    RCLCPP_INFO(node->get_logger(), "현재 관절 위치:");
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(node->get_logger(), "  %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // 조인트 모델 그룹의 변수 수 확인
    size_t joint_count = joint_model_group->getVariableCount();
    RCLCPP_INFO(node->get_logger(), "'pollibot_arm' 그룹의 조인트 수: %zu", joint_count);

    // 초기 관절 위치 설정 (조인트 수에 맞게 조정)
    if (initial_joint_positions.size() != joint_count)
    {
        RCLCPP_ERROR(node->get_logger(), "초기 관절 위치의 크기와 조인트 수가 일치하지 않습니다.");
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }

    // 현재 상태에 초기 관절 위치 설정
    current_state->setJointGroupPositions(joint_model_group, initial_joint_positions);
    // 시작 상태를 초기 관절 위치로 설정
    move_group.setStartState(*current_state);
    // 목표 위치에 대한 허용 오차 설정 (미터 단위)
    move_group.setGoalPositionTolerance(1); // 0.01 = 1cm
    // 목표 자세에 대한 허용 오차 설정 (라디안 단위)
    move_group.setGoalOrientationTolerance(1); //
    // 목표 조인트 값에 대한 허용 오차 설정 (라디안 단위)
    move_group.setGoalJointTolerance(1); // 0.01 = 0.57도
    // 경로 계획 시도 횟수 설정
    move_group.setNumPlanningAttempts(5); // 예: 최대 10번 시도

    // 목표 자세 설정
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.2;  // 원하는 x 좌표로 변경
    target_pose.position.y = 0.0;  // 원하는 y 좌표로 변경
    target_pose.position.z = 0.4;  // 원하는 z 좌표로 변경

    // 원하는 방향으로의 쿼터니언 생성 (예: 엔드 이펙터가 아래를 향하도록)
    tf2::Quaternion q;
    q.setRPY(1.57, 0.22, -0.05);  // Roll, Pitch, Yaw 값을 설정합니다.
    target_pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(node->get_logger(), "목표 자세:");
    RCLCPP_INFO(node->get_logger(), "  위치 - x: %f, y: %f, z: %f",
            target_pose.position.x, target_pose.position.y, target_pose.position.z);
    RCLCPP_INFO(node->get_logger(), "  방향 - x: %f, y: %f, z: %f, w: %f",
            target_pose.orientation.x, target_pose.orientation.y,
            target_pose.orientation.z, target_pose.orientation.w);

    // IK 솔버를 사용하여 조인트 값 계산 (테스트용)
    moveit::core::RobotStatePtr test_state(new moveit::core::RobotState(*current_state));
    bool found_ik = test_state->setFromIK(joint_model_group, target_pose);

    if (!found_ik)
    {
        RCLCPP_ERROR(node->get_logger(), "IK 해를 찾을 수 없습니다. 목표 자세가 로봇의 작업 공간 내에 있는지 확인하십시오.");
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }

    // 목표 자세 설정
    move_group.setPoseTarget(target_pose);

    // 경로 계획 및 실행
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode success = move_group.plan(plan);
    // bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "경로 계획에 실패했습니다. 에러 코드: %d", success.val);
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "경로 계획에 성공했습니다. 실행 중...");
        moveit::core::MoveItErrorCode exec_success = move_group.execute(plan);
        if (exec_success != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "경로 실행에 실패했습니다. 에러 코드: %d", exec_success.val);
            executor.cancel();
            rclcpp::shutdown();
            return 1;
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "경로 실행에 성공했습니다.");
        }
    }

    // if (success)
    // {
    //     RCLCPP_INFO(node->get_logger(), "계획이 성공적으로 수립되었습니다. 실행 중...");
    //     move_group.execute(plan);
    //     RCLCPP_INFO(node->get_logger(), "이동이 성공적으로 완료되었습니다.");
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "목표 자세로의 계획에 실패했습니다.");
    // }

    // 프로그램 종료 시
    executor.cancel();
    rclcpp::shutdown();
    return 0;
}
