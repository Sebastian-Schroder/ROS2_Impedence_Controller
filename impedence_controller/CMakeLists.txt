cmake_minimum_required(VERSION 3.8)
project(impedence_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(get_parameter_library REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set( ament_dependencies 
  controller_interface
  hardware_interface
  rclcpp
  pluginlib
  rclcpp_lifecycle
  Eigen3
  tf2_eigen # for some reason pinocchio cant find eigen without this package also being included
  pinocchio
  realtime_tools
  generate_parameter_library
)


add_library(impedence_controller SHARED
src/impedence_controller.cpp)
target_include_directories(impedence_controller PUBLIC include)
ament_target_dependencies(impedence_controller PUBLIC ${ament_dependencies})


target_compile_definitions(impedence_controller PRIVATE "IMPEDENCE_CONTROLLERS_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface cartesianimpedencecontroller.xml)


if(BUILD_TESTING)
  #? find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_impedence_controller
    test/test_load_impedence_controller.cpp
  )
  target_link_libraries(test_load_impedence_controller
  impedence_controller
  )
  ament_target_dependencies(test_load_impedence_controller
    controller_manager
    ros2_control_test_assets
  )
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #? set(ament_cmake_cpplint_FOUND TRUE)
  #? ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include
  DESTINATION include/impedence_controller
)
install(
  TARGETS impedence_controller
  EXPORT export_impedence_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
