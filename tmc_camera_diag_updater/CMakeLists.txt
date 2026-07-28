cmake_minimum_required(VERSION 3.8)
project(tmc_camera_diag_updater)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tmc_utils REQUIRED)

add_library(${PROJECT_NAME}_component SHARED src/camera_diag_component.cpp)
target_link_libraries(${PROJECT_NAME}_component ${rclcpp_LIBRARIES})
target_include_directories(${PROJECT_NAME}_component PUBLIC include ${rclcpp_INCLUDE_DIRS})

ament_target_dependencies(
  ${PROJECT_NAME}_component
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  sensor_msgs
  tmc_utils)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "camera_diag_updater::CameraDiagComponent"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  TARGETS
    ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(Boost REQUIRED)
  find_package(rcl_interfaces REQUIRED)

  ament_add_gtest_executable(camera_image_diag_boot_test
    test/camera_image_diag_boot-test.cpp)
  ament_target_dependencies(camera_image_diag_boot_test
  Boost
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  rcl_interfaces
  sensor_msgs)

  target_link_libraries(camera_image_diag_boot_test ${rclcpp_LIBRARIES})
  install(
    TARGETS camera_image_diag_boot_test
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    )
  ament_add_pytest_test(camera_image_diag_boot_test_py
    test/camera_image_diag_boot-test.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  ament_add_gtest_executable(camera_point_diag_boot_test
    test/camera_point_diag_boot-test.cpp)
  ament_target_dependencies(camera_point_diag_boot_test
  Boost
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  rcl_interfaces
  sensor_msgs)

  target_link_libraries(camera_point_diag_boot_test ${rclcpp_LIBRARIES})
  install(
    TARGETS camera_point_diag_boot_test
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    )
  ament_add_pytest_test(camera_point_diag_boot_test_py
    test/camera_point_diag_boot-test.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  ament_add_gtest_executable(camera_diag_update_test
    test/camera_diag_update-test.cpp)
  ament_target_dependencies(camera_diag_update_test
  Boost
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  rcl_interfaces
  sensor_msgs)

  target_link_libraries(camera_diag_update_test ${PROJECT_NAME}_component ${rclcpp_LIBRARIES})
  install(
    TARGETS camera_diag_update_test
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    )
  ament_add_pytest_test(camera_diag_update_test_py
    test/camera_diag_update-test.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    TIMEOUT 120)
endif()

ament_export_dependencies(rclcpp rclcpp_components sensor_msgs diagnostic_msgs diagnostic_updater tmc_utils)

ament_package()
