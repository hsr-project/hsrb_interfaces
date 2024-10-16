cmake_minimum_required(VERSION 3.5)
project(hsrb_interface_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)

set(_package_namespace hsrb_interface)
add_library(${PROJECT_NAME}_extension SHARED
            ${_package_namespace}/ext/head_kinematics.cpp
            ${_package_namespace}/ext/kinematics.cpp)
target_include_directories(${PROJECT_NAME}_extension
  PUBLIC include ${rclcpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_extension ${rclcpp_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(
  ${PROJECT_NAME}_extension
  PROPERTIES OUTPUT_NAME _extension
             LIBRARY_OUTPUT_DIRECTORY lib/${_package_namespace}
             PREFIX "")

ament_target_dependencies(${PROJECT_NAME}_extension rclcpp tmc_robot_kinematics_model Eigen3)

ament_python_install_package(hsrb_interface)

# Install python scripts
install(PROGRAMS scripts/ihsrb.py
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_extension
        DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_robot_auto_init test/test_robot_auto_init.py)
  ament_add_pytest_test(test_robot_duplicate_init test/test_robot_duplicate_init.py)
  ament_add_pytest_test(test_sensors test/test_sensors.py)
  ament_add_pytest_test(test_geometry test/test_geometry.py)
  ament_add_pytest_test(test_robot test/test_robot.py)
  ament_add_pytest_test(test_text_to_speech test/test_text_to_speech.py)
  ament_add_pytest_test(test_utils test/test_utils.py)
  ament_add_pytest_test(test_whole_body test/test_whole_body.py)
  ament_add_pytest_test(test_collision_world test/test_collision_world.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_extension)
ament_package()
