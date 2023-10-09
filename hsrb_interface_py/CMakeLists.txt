cmake_minimum_required(VERSION 2.8.3)
project(hsrb_interface_py)

find_package(catkin REQUIRED COMPONENTS hsr_kinematics)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(Eigen REQUIRED)
find_package(PythonLibs REQUIRED)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS hsr_kinematics)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

set(_package_namespace hsrb_interface)
add_library(${PROJECT_NAME}_extension SHARED src/${_package_namespace}/ext/kinematics.cpp)
target_link_libraries(${PROJECT_NAME}_extension ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(
    ${PROJECT_NAME}_extension PROPERTIES
    OUTPUT_NAME _extension
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${_package_namespace}
    PREFIX ""
)

catkin_install_python(PROGRAMS
  scripts/ihsrb
  scripts/hsrb_operator.py
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_extension LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${_package_namespace})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_nosetests(test)
    add_rostest(test/robot_auto_init.test)
    add_rostest(test/robot_duplicate_init.test)
    #    add_rostest(test/simtest_gripper.test)
    #    add_rostest(test/simtest_omni_base.test)
    #    add_rostest(test/simtest_whole_body.test)
    #    add_rostest(test/simtest_collision.test)
    #    add_rostest(test/simtest_marker.test)
    #    add_rostest(test/simtest_linemod.test)
    #    add_rostest(test/simtest_fetch.test)
endif()

