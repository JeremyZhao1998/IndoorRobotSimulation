cmake_minimum_required(VERSION 2.8.3)
project(dstar_global_planner)

ADD_DEFINITIONS(-std=c++0x) # C++ 11 is required

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  costmap_2d
  roscpp
  rospy
  std_msgs
  tf
)

find_package(cmake_modules REQUIRED)


#FIND_PACKAGE(Eigen REQUIRED)
#FIND_PACKAGE(Boost REQUIRED)




catkin_package(

  CATKIN_DEPENDS
    geometry_msgs
    nav_core
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
    costmap_2d
    pluginlib
  LIBRARIES
    dstar_global_planner


)

###########
## Build ##
###########
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)



set(SOURCES
    src/dstar_global_planner.cpp  src/Dstar.cpp include/dstar_global_planner/pathSplineSmoother/pathSplineSmoother.cpp

)



# Plugin
add_library(dstar_global_planner  ${SOURCES_RRT} ${SOURCES})

target_link_libraries(dstar_global_planner ${catkin_LIBRARIES} ${BOOST_LIBRARIES} )

add_dependencies(dstar_global_planner dstar_global_planner_gencfg ${catkin_EXPORTED_TARGETS})

add_dependencies(dstar_global_planner nav_msgs_gencpp)


install(TARGETS dstar_global_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}

       )

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
