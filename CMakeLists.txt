cmake_minimum_required(VERSION 3.8)
project(gz_odom_pub_hw_component)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(backward_ros REQUIRED)

set(DEPENDENCIES_FOR_THIS_PACKAGE
  gz_math_vendor
  gz_msgs_vendor
  gz_plugin_vendor
  gz_ros2_control
  gz_sim_vendor
  gz_transport_vendor
  gz-math
  gz-msgs
  gz-plugin
  gz-sim
  gz-transport
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

foreach(dependency IN ITEMS ${DEPENDENCIES_FOR_THIS_PACKAGE})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(gz_odom_pub_hw_component
  src/gz_odom_pub_hw_component.cpp
)

target_compile_features(gz_odom_pub_hw_component PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

target_include_directories(gz_odom_pub_hw_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(gz_odom_pub_hw_component
  gz-sim::gz-sim
  gz-msgs::gz-msgs
  gz-math::gz-math
  gz-transport::gz-transport
)

ament_target_dependencies(gz_odom_pub_hw_component
  ${DEPENDENCIES_FOR_THIS_PACKAGE}
)

pluginlib_export_plugin_description_file(gz_ros2_control
  gz_odom_pub_hw_component.xml
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS gz_odom_pub_hw_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

  ament_add_gtest(test_gz_odom_pub_hw_component
    test/gz_odom_pub_hw_component_loader.cpp
  )

  target_link_libraries(test_gz_odom_pub_hw_component
    gz_odom_pub_hw_component
  )

  ament_target_dependencies(test_gz_odom_pub_hw_component
    pluginlib
    gz_ros2_control
  )
endif()

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_package()
