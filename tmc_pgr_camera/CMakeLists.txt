cmake_minimum_required(VERSION 3.5)
project(tmc_pgr_camera)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tmc_vision_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

find_library(POINTGREY_LIB flycapture)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/point_grey_camera.cpp)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${POINTGREY_LIB})
target_include_directories(${PROJECT_NAME}
  PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  image_transport
  compressed_image_transport
  pluginlib
  sensor_msgs
  tmc_vision_msgs
)

add_library(${PROJECT_NAME}_system src/${PROJECT_NAME}/point_grey_camera_system.cpp)
target_link_libraries(${PROJECT_NAME}_system
  ${Boost_LIBRARIES}
  ${PROJECT_NAME})

add_library(${PROJECT_NAME}_system_plugin SHARED
  src/${PROJECT_NAME}/camera_system_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME}_system_plugin
  rclcpp
  cv_bridge
  image_transport
  compressed_image_transport
  pluginlib
  sensor_msgs
  tmc_vision_msgs)
target_link_libraries(${PROJECT_NAME}_system_plugin
  ${PROJECT_NAME}_system yaml-cpp)
pluginlib_export_plugin_description_file(${PROJECT_NAME} camera_system_plugins.xml)
set(PLUGIN_LIBS ${PROJECT_NAME}_system_plugin)
set(EXPORT_PLUGINS export_${PROJECT_NAME}_system_plugin)

add_library(${PROJECT_NAME}_images_publisher src/${PROJECT_NAME}/camera_images_publisher.cpp)
ament_target_dependencies(${PROJECT_NAME}_images_publisher
  rclcpp
  cv_bridge
  image_transport
  compressed_image_transport
  pluginlib
  sensor_msgs
  tmc_vision_msgs)
target_link_libraries(${PROJECT_NAME}_images_publisher
  ${POINTGREY_LIB}
  ${OpenCV_LIBS})

add_library(${PROJECT_NAME}_system_setting
  src/${PROJECT_NAME}/yaml_point_grey_camera_system_setting.cpp
  src/${PROJECT_NAME}/ros_parameter_point_grey_camera_system_setting.cpp)
ament_target_dependencies(${PROJECT_NAME}_system_setting
  rclcpp
  image_transport
  compressed_image_transport
  pluginlib
  sensor_msgs
  tmc_vision_msgs)
target_link_libraries(${PROJECT_NAME}_system_setting
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${POINTGREY_LIB}
  yaml-cpp)

add_library(${PROJECT_NAME}_nodelet SHARED
  src/${PROJECT_NAME}/camera_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_images_publisher
  ${PROJECT_NAME}_system_setting)
ament_target_dependencies(${PROJECT_NAME}_nodelet
  rclcpp
  rclcpp_components
  image_transport
  compressed_image_transport
  pluginlib
  sensor_msgs
  tmc_vision_msgs)
rclcpp_components_register_node(
  ${PROJECT_NAME}_nodelet
  PLUGIN "tmc_pgr_camera::CameraNodelet"
  EXECUTABLE ${PROJECT_NAME}_node)

add_executable(update_config
  src/${PROJECT_NAME}/update_config.cpp)
ament_target_dependencies(update_config
  rclcpp
  image_transport
  compressed_image_transport
  pluginlib
  sensor_msgs
  tmc_vision_msgs)
target_link_libraries(update_config
  ${Boost_LIBRARIES}
  yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_yaml_point_grey_camera_system_setting
    test/yaml_point_grey_camera_system_setting-test.cpp
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  target_link_libraries(test_yaml_point_grey_camera_system_setting
    ${PROJECT_NAME}_system_setting
    ${Boost_LIBRARIES}
    gtest)

  ament_add_gtest(test_point_grey_camera_system
    test/point_grey_camera_stub.cpp
    test/point_grey_camera_system-test.cpp
    src/${PROJECT_NAME}/point_grey_camera_system.cpp
    src/${PROJECT_NAME}/yaml_point_grey_camera_system_setting.cpp
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_target_dependencies(test_point_grey_camera_system
    rclcpp
    image_transport
    compressed_image_transport
    pluginlib
    sensor_msgs
    tmc_vision_msgs)
  target_link_libraries(test_point_grey_camera_system
    ${Boost_LIBRARIES}
    ${POINTGREY_LIB}
    ${OpenCV_LIBS}
    yaml-cpp)

  add_library(test_camera_system_plugin SHARED
    test/dummy_camera_system_plugin.cpp
    src/${PROJECT_NAME}/yaml_point_grey_camera_system_setting.cpp)
  ament_target_dependencies(test_camera_system_plugin
    rclcpp
    image_transport
    compressed_image_transport
    pluginlib
    sensor_msgs
    tmc_vision_msgs)
  target_link_libraries(test_camera_system_plugin
    ${OpenCV_LIBS} yaml-cpp)
  pluginlib_export_plugin_description_file(${PROJECT_NAME}
    test/camera_system_plugin.xml)
  set(PLUGIN_LIBS ${PLUGIN_LIBS} test_camera_system_plugin)
  set(EXPORT_PLUGINS ${EXPORT_PLUGINS} export_test_camera_system_plugin)

  ament_add_gtest(ros_parameter_point_grey_camera_system_setting-test
    test/ros_parameter_point_grey_camera_system_setting-test.cpp
    src/${PROJECT_NAME}/yaml_point_grey_camera_system_setting.cpp
    src/${PROJECT_NAME}/ros_parameter_point_grey_camera_system_setting.cpp
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_target_dependencies(ros_parameter_point_grey_camera_system_setting-test
    rclcpp
    image_transport
    compressed_image_transport
    pluginlib
    sensor_msgs
    tmc_vision_msgs)
  target_link_libraries(ros_parameter_point_grey_camera_system_setting-test
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    yaml-cpp)

  ament_add_gtest_executable(camera_node-test
    test/camera_node-test.cpp)
  ament_target_dependencies(camera_node-test
    rclcpp
    cv_bridge
    image_transport
    compressed_image_transport
    pluginlib
    sensor_msgs
    tmc_vision_msgs)
  target_link_libraries(camera_node-test
    ${OpenCV_LIBS})
  install(
    TARGETS camera_node-test
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    )
  ament_add_pytest_test(camera_node-test_py
    test/camera_node_test.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

endif()

install(TARGETS
  ${PROJECT_NAME}_system_setting
  ${PROJECT_NAME}_images_publisher
  ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_system
  ${PLUGIN_LIBS}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  update_config
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  USE_SOURCE_PERMISSIONS
)

install(FILES
  camera_system_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  test/camera_system_plugin.xml
  DESTINATION share/${PROJECT_NAME}/test
)

ament_export_include_directories(include)
ament_export_libraries(
  ${PLUGIN_LIBS}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
