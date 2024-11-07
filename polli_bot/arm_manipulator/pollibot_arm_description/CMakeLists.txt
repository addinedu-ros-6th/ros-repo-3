cmake_minimum_required(VERSION 3.8)
project(pollibot_arm_description)

# 컴파일 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# 설치할 디렉토리 설정
install(
  DIRECTORY launch urdf rviz config meshes
  DESTINATION share/${PROJECT_NAME}
)

# 주석 처리된 부분에 추가 의존성을 필요로 하면 해당 내용을 추가할 수 있습니다
# 예시: find_package(<dependency> REQUIRED)

# 테스트 빌드 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # 아래 두 줄은 linter를 건너뛰도록 설정합니다
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

