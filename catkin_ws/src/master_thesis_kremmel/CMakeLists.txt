cmake_minimum_required(VERSION 3.0.2)
project(master_thesis_kremmel)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
  std_msgs
  pcl_ros
  pcl_conversions
)

pkg_check_modules(EIGEN3 eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp
                   sensor_msgs 
                   std_msgs
                   pcl_ros
                   pcl_conversions
  )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(LandMarkCornerDetector src/cornerDetection.cpp)
target_link_libraries(LandMarkCornerDetector ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#add_executable(PubToCmdVel src/pubToCmdVel.cpp)
#target_link_libraries(PubToCmdVel ${catkin_LIBRARIES})