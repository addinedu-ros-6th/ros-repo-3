// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <thread>
// #include <chrono>
// #include <vector>
// #include <utility>
// #include <fstream>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/kinematics_base/kinematics_base.h> 

// // bool setFromIK(
// //     const JointModelGroup* group,
// //     const geometry_msgs::msg::Pose& pose,
// //     const std::string& end_effector_link = "",
// //     double timeout = 0.0,
// //     const moveit::core::GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
// //     const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()
// // );

// int main(int argc, char** argv)
// {
//     // ROS 2 초기화
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("move_to_target");

//     // 비동기 스피너 초기화 및 시작
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     std::thread([&executor]() { executor.spin(); }).detach();

//     // MoveGroupInterface 생성
//     moveit::planning_interface::MoveGroupInterface move_group(node, "pollibot_arm");

//     if (!move_group.getRobotModel())
//     {
//         RCLCPP_ERROR(node->get_logger(), "MoveGroupInterface를 초기화하는 데 실패했습니다.");
//         executor.cancel();
//         rclcpp::shutdown();
//         return 1;
//     }

//     // 기준 프레임 설정
//     move_group.setPoseReferenceFrame("base_link");

//     // 잠시 대기하여 시스템 안정화
//     rclcpp::sleep_for(std::chrono::seconds(1));

//     // 로봇의 현재 상태 가져오기
//     auto current_state = move_group.getCurrentState(10.0);
//     if (!current_state)
//     {
//         RCLCPP_ERROR(node->get_logger(), "로봇의 현재 상태를 가져오지 못했습니다.");
//         executor.cancel();
//         rclcpp::shutdown();
//         return 1;
//     }

//     const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup("pollibot_arm");
//     if (!joint_model_group)
//     {
//         RCLCPP_ERROR(node->get_logger(), "'pollibot_arm' 조인트 모델 그룹을 찾을 수 없습니다.");
//         executor.cancel();
//         rclcpp::shutdown();
//         return 1;
//     }

//     // 탐색 범위 설정
//     double x_min = 0.0, x_max = 0.2, x_step = 0.05;
//     double y_min = -0.1, y_max = 0.1, y_step = 0.05;
//     double z_min = 0.3, z_max = 0.5, z_step = 0.05;

//     // IK 해 저장을 위한 벡터
//     std::vector<std::pair<geometry_msgs::msg::Pose, std::vector<double>>> ik_solutions;

//     // 결과 파일 경로 설정
//     std::string file_path = "/home/ask/dev_ws/ik_results/ik_solutions.txt";

//     // 결과를 파일에 저장하기 위한 스트림 열기
//     std::ofstream outfile(file_path);
//     if (!outfile.is_open())
//     {
//         RCLCPP_ERROR(node->get_logger(), "결과 파일을 열 수 없습니다: %s", file_path.c_str());
//         executor.cancel();
//         rclcpp::shutdown();
//         return 1;
//     }
//     else
//     {
//         RCLCPP_INFO(node->get_logger(), "ik_solutions.txt 파일을 성공적으로 열었습니다: %s", file_path.c_str());
//     }


//     // IK 해 탐색
//     RCLCPP_INFO(node->get_logger(), "IK 해 탐색 시작...");
//     size_t total_iterations = 0;
//     size_t found_solutions = 0;

//     // 허용 오차 설정
//     double timeout = 1.0; // IK 솔버의 타임아웃 시간 (초)
//     // unsigned int attempts = 100; // 시도 횟수 (0이면 내부적으로 결정)

//     // 위치 및 자세 허용 오차 설정
//     double position_tolerance = 0.05;     // 위치 허용 오차 (미터)
//     double orientation_tolerance = 0.1;   // 자세 허용 오차 (라디안)

//     // IK 옵션 설정
//     moveit::core::GroupStateValidityCallbackFn constraint_fn = nullptr; // 제약 조건이 없으면 빈 함수
//     // moveit::core::KinematicsQueryOptions options;
//     kinematics::KinematicsQueryOptions options;
//     options.return_approximate_solution = true; // 근사 솔루션 허용

//     rclcpp::sleep_for(std::chrono::seconds(1));

//     for (double x = x_min; x <= x_max; x += x_step)
//     {
//         for (double y = y_min; y <= y_max; y += y_step)
//         {
//             for (double z = z_min; z <= z_max; z += z_step)
//             {
//                 geometry_msgs::msg::Pose target_pose;
//                 target_pose.position.x = x;
//                 target_pose.position.y = y;
//                 target_pose.position.z = z;

//                 // 원하는 방향으로의 쿼터니언 생성
//                 tf2::Quaternion q;
//                 q.setRPY(1.57, 0.22689, -0.052); // 기본 방향
//                 target_pose.orientation = tf2::toMsg(q);

//                 moveit::core::RobotStatePtr test_state(new moveit::core::RobotState(*current_state));
//                 bool found_ik = test_state->setFromIK(
//                     joint_model_group,
//                     target_pose,
//                     "endeffector", // 사용 중인 엔드 이펙터 링크의 이름으로 대체하거나 빈 문자열 사용
//                     timeout,
//                     constraint_fn,
//                     options
//                 );

//                 total_iterations++;

//                 if (found_ik)
//                 {
//                     std::vector<double> joint_values;
//                     test_state->copyJointGroupPositions(joint_model_group, joint_values);

//                     ik_solutions.emplace_back(std::make_pair(target_pose, joint_values));
//                     found_solutions++;

//                     // 파일에 저장
//                     outfile << "Solution " << found_solutions << ":\n";
//                     outfile << "  Position: (" << x << ", " << y << ", " << z << ")\n";
//                     outfile << "  Orientation: (" << target_pose.orientation.x << ", "
//                             << target_pose.orientation.y << ", " << target_pose.orientation.z << ", "
//                             << target_pose.orientation.w << ")\n";
//                     outfile << "  Joint Values:\n";
//                     for (size_t i = 0; i < joint_values.size(); ++i)
//                     {
//                         outfile << "    " << joint_model_group->getJointModelNames()[i] 
//                                 << ": " << joint_values[i] << "\n";
//                     }
//                     outfile << "\n";
//                 }
//                 else
//                 {
//                     RCLCPP_WARN(node->get_logger(), "IK 해를 찾지 못했습니다 (x: %.2f, y: %.2f, z: %.2f)", x, y, z);
//                 }

//                 // 각 시도에 대한 디버그 로그 (옵션)
//                 RCLCPP_DEBUG(node->get_logger(), "시도 #%zu: x=%.2f, y=%.2f, z=%.2f", total_iterations, x, y, z);

//                 // 진행 상황 로그 (예: 매 100번 시도마다 출력)
//                 if (total_iterations % 100 == 0)
//                 {
//                     RCLCPP_INFO(node->get_logger(), "진행 중: %zu / %zu 시도", total_iterations, 
//                                 static_cast<size_t>((x_max - x_min) / x_step + 1) * 
//                                 static_cast<size_t>((y_max - y_min) / y_step + 1) * 
//                                 static_cast<size_t>((z_max - z_min) / z_step + 1));
//                 }


//             }
//         }
//     }

//     RCLCPP_INFO(node->get_logger(), "IK 해 탐색 완료.");
//     RCLCPP_INFO(node->get_logger(), "총 %zu 번의 시도 중 %zu 개의 IK 해를 찾았습니다.", total_iterations, found_solutions);

//     // 요약 출력 (콘솔)
//     std::cout << "총 시도 횟수: " << total_iterations << "\n";
//     std::cout << "찾은 IK 해의 개수: " << found_solutions << "\n";

//     // 파일 닫기
//     outfile.close();
//     RCLCPP_INFO(node->get_logger(), "ik_solutions.txt 파일을 닫았습니다.");

//     // 프로그램 종료 시
//     executor.cancel();
//     rclcpp::shutdown();
//     return 0;
// }

// // bool setFromIK(
// //     const JointModelGroup* group,
// //     const geometry_msgs::msg::Pose& pose,
// //     const std::string& end_effector_link = "",
// //     double timeout = 0.0,
// //     const moveit::core::GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
// //     const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()
// // );


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <chrono>
#include <vector>
#include <utility>
#include <fstream>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <filesystem>  // C++17 이상 필요

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

    if (!move_group.getRobotModel())
    {
        RCLCPP_ERROR(node->get_logger(), "MoveGroupInterface를 초기화하는 데 실패했습니다.");
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }

    // 기준 프레임 설정
    move_group.setPoseReferenceFrame("base_link");

    // 잠시 대기하여 시스템 안정화
    rclcpp::sleep_for(std::chrono::seconds(5));

    // 로봇의 현재 상태 가져오기
    auto current_state = move_group.getCurrentState(10.0);
    if (!current_state)
    {
        RCLCPP_ERROR(node->get_logger(), "로봇의 현재 상태를 가져오지 못했습니다.");
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }

    const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup("pollibot_arm");
    if (!joint_model_group)
    {
        RCLCPP_ERROR(node->get_logger(), "'pollibot_arm' 조인트 모델 그룹을 찾을 수 없습니다.");
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }

    // 탐색 범위 설정
    double x_min = 0.0, x_max = 0.1, x_step = 0.01;
    double y_min = -0.1, y_max = 0.0, y_step = 0.01;
    double z_min = 0.3, z_max = 0.5, z_step = 0.05;

    // IK 해 저장을 위한 벡터
    std::vector<std::pair<geometry_msgs::msg::Pose, std::vector<double>>> ik_solutions;

    // 파일 저장 디렉토리 및 경로 설정
    std::string directory = "/home/ask/dev_ws/ik_results";
    std::string file_path = directory + "/ik_solutions.txt";

    // 디렉토리 존재 여부 확인 및 생성
    if (!std::filesystem::exists(directory))
    {
        try
        {
            std::filesystem::create_directories(directory);
            RCLCPP_INFO(node->get_logger(), "디렉토리를 생성했습니다: %s", directory.c_str());
        }
        catch (const std::filesystem::filesystem_error& e)
        {
            RCLCPP_ERROR(node->get_logger(), "디렉토리를 생성하지 못했습니다: %s", e.what());
            executor.cancel();
            rclcpp::shutdown();
            return 1;
        }
    }

    // 결과를 파일에 저장하기 위한 스트림 열기
    std::ofstream outfile(file_path);
    if (!outfile.is_open())
    {
        RCLCPP_ERROR(node->get_logger(), "결과 파일을 열 수 없습니다: %s", file_path.c_str());
        executor.cancel();
        rclcpp::shutdown();
        return 1;
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "ik_solutions.txt 파일을 성공적으로 열었습니다: %s", file_path.c_str());
    }

    // IK 해 탐색
    RCLCPP_INFO(node->get_logger(), "IK 해 탐색 시작...");
    size_t total_iterations = 0;
    size_t found_solutions = 0;

    // 허용 오차 설정
    double timeout = 0.5; // IK 솔버의 타임아웃 시간 (초)

    // 위치 및 자세 허용 오차 설정
    double position_tolerance = 0.1;     // 위치 허용 오차 (미터)
    // double orientation_tolerance = 0.5;   // 자세 허용 오차 (라디안)

    for (double x = x_min; x <= x_max; x += x_step)
    {
        for (double y = y_min; y <= y_max; y += y_step)
        {
            for (double z = z_min; z <= z_max; z += z_step)
            {
                geometry_msgs::msg::Pose target_pose;
                target_pose.position.x = x;
                target_pose.position.y = y;
                target_pose.position.z = z;

                // 원하는 방향으로의 쿼터니언 생성
                tf2::Quaternion q;
                // q.setRPY(1.57, 0.22689, -0.052); // 기본 방향
                q.setRPY(-1.57, -1.56, 2.26); // 기본 방향
                target_pose.orientation = tf2::toMsg(q);

                // 제약 조건 콜백 함수 정의
                moveit::core::GroupStateValidityCallbackFn constraint_fn = [&](const moveit::core::RobotState *state, const moveit::core::JointModelGroup *group, const double *ik_solution) -> bool {
                    (void)group;
                    (void)ik_solution;

                    const std::string end_effector_link = "endeffector";

                    // 현재 상태의 엔드 이펙터 자세 가져오기
                    // Eigen::Isometry3d transform = state->getGlobalLinkTransform(end_effector_link);
                    // Eigne::Vector3d current_position = transform.translation();
                    Eigen::Vector3d current_position = state->getGlobalLinkTransform(end_effector_link).translation();

                    // geometry_msgs::msg::Pose current_pose;
                    // current_pose.position.x = transform.translation().x();
                    // current_pose.position.y = transform.translation().y();
                    // current_pose.position.z = transform.translation().z();

                    // Eigen::Quaterniond q_current(transform.rotation());
                    // current_pose.orientation.x = q_current.x();
                    // current_pose.orientation.y = q_current.y();
                    // current_pose.orientation.z = q_current.z();
                    // current_pose.orientation.w = q_current.w();

                    // 위치 차이 계산
                    double dx = target_pose.position.x - current_position.x();
                    double dy = target_pose.position.y - current_position.y();
                    double dz = target_pose.position.z - current_position.z();
                    double position_diff = std::sqrt(dx*dx + dy*dy + dz*dz);

                    // 자세 차이 계산
                    // tf2::Quaternion q1, q2;
                    // tf2::fromMsg(target_pose.orientation, q1);
                    // tf2::fromMsg(current_pose.orientation, q2);
                    // double orientation_diff = q1.angleShortestPath(q2);

                    // 위치 및 자세 차이에 대한 로그 메시지 추가
                    // RCLCPP_WARN(node->get_logger(), "Position Diff: %.4f m, Orientation Diff: %.4f rad", position_diff, orientation_diff);
                    // RCLCPP_INFO(node->get_logger(), "Position Diff: %.4f m, Orientation Diff: %.4f rad", position_diff, orientation_diff);
                    // std::cout << std::flush;

                    // 허용 오차 내에 있는지 확인
                    // return (position_diff <= position_tolerance) && (orientation_diff <= orientation_tolerance);
                    return (position_diff <= position_tolerance);
                };

                kinematics::KinematicsQueryOptions options;
                options.return_approximate_solution = true; // 근사 솔루션 허용
                // options.orientation_tolerance = M_PI; // 자세 허용 오차 (라디안)
                // options.avoid_collisions = false; // 충돌 회피 사용 안함

                moveit::core::RobotStatePtr test_state(new moveit::core::RobotState(*current_state));
                bool found_ik = test_state->setFromIK(
                    joint_model_group,
                    target_pose,
                    // target_position,
                    "endeffector",
                    timeout,
                    constraint_fn,
                    // kinematics::KinematicsQueryOptions().return_approximate_solution(true),
                    // kinematics::KinematicsQueryOptions().avoid_collisions(false),
                    options
                );

                total_iterations++;

                if (found_ik)
                {
                    std::vector<double> joint_values;
                    test_state->copyJointGroupPositions(joint_model_group, joint_values);

                    ik_solutions.emplace_back(std::make_pair(target_pose, joint_values));
                    found_solutions++;

                    // 파일에 저장
                    outfile << "Solution " << found_solutions << ":\n";
                    outfile << "  Position: (" << x << ", " << y << ", " << z << ")\n";
                    outfile << "  Orientation: (" << target_pose.orientation.x << ", "
                            << target_pose.orientation.y << ", " << target_pose.orientation.z << ", "
                            << target_pose.orientation.w << ")\n";
                    outfile << "  Joint Values:\n";
                    for (size_t i = 0; i < joint_values.size(); ++i)
                    {
                        outfile << "    " << joint_model_group->getJointModelNames()[i] 
                                << ": " << joint_values[i] << "\n";
                    }
                    outfile << "\n";
                }
                else
                {
                    RCLCPP_WARN(node->get_logger(), "IK 해를 찾지 못했습니다 (x: %.2f, y: %.2f, z: %.2f)", x, y, z);
                }

                // 각 시도에 대한 디버그 로그 (옵션)
                RCLCPP_DEBUG(node->get_logger(), "시도 #%zu: x=%.2f, y=%.2f, z=%.2f", total_iterations, x, y, z);

                // 진행 상황 로그 (예: 매 100번 시도마다 출력)
                size_t total_possible_iterations = 
                    static_cast<size_t>((x_max - x_min) / x_step + 1) * 
                    static_cast<size_t>((y_max - y_min) / y_step + 1) * 
                    static_cast<size_t>((z_max - z_min) / z_step + 1);
                if (total_iterations % 100 == 0)
                {
                    RCLCPP_INFO(node->get_logger(), "진행 중: %zu / %zu 시도", total_iterations, total_possible_iterations);
                }
            }
        }

        RCLCPP_INFO(node->get_logger(), "IK 해 탐색 완료.");
        RCLCPP_INFO(node->get_logger(), "총 %zu 번의 시도 중 %zu 개의 IK 해를 찾았습니다.", total_iterations, found_solutions);

        // 요약 출력 (콘솔)
        std::cout << "총 시도 횟수: " << total_iterations << "\n";
        std::cout << "찾은 IK 해의 개수: " << found_solutions << "\n";

        // 파일 닫기
        outfile.close();
        RCLCPP_INFO(node->get_logger(), "ik_solutions.txt 파일을 닫았습니다.");

        // 프로그램 종료 시
        executor.cancel();
        rclcpp::shutdown();
        return 0;
    }
}