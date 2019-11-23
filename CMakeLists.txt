cmake_minimum_required(VERSION 2.8.3)
project(mas_cartesian_control)
set(CMAKE_CXX_STANDARD 11)

add_compile_options(
    # Code optimization level 3 (-O3). Diable this flag for compiling in debug mode or in case of unexpected crashing. 
	    )

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp sensor_msgs
    tf
    urdf
    kdl_parser
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS  
    sensor_msgs
    kdl_parser
)

include_directories(
 ${PROJECT_SOURCE_DIR}/ros/include 
 ${PROJECT_SOURCE_DIR}/common/include
 ${EIGEN3_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
            common/src/arm_cartesian_control.cpp
            ros/src/ros_arm_cartesian_control.cpp
            ros/src/ros_urdf_loader.cpp
  )


add_executable(arm_cartesian_control_node
               ros/src/arm_cartesian_control_node.cpp
  )
    
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(arm_cartesian_control_node
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}
  ${orocos_kdl_LIBRARIES} 
)


add_dependencies(${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)

set(MCR_ARM_CART_CONTROL_TARGETS ${PROJECT_NAME} arm_cartesian_control_node)

install(TARGETS ${MCR_ARM_CART_CONTROL_TARGETS} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME} 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


install(DIRECTORY ros/include/${PROJECT_NAME} 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()

