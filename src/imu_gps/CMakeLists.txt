cmake_minimum_required(VERSION 3.8)
project(imu_gps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)

# ==== Source / header globs ==== #
file(GLOB_RECURSE SRC_FILES   CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS include/*.hpp include/imu_gps/*.hpp)
list(FILTER SRC_FILES EXCLUDE REGEX "src/app/")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/app/run.cpp ${SRC_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    include
)

target_link_libraries(${PROJECT_NAME}_node
  Eigen3::Eigen
  yaml-cpp
  TBB::tbb
)


ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  Eigen3
  yaml-cpp
  TBB
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
