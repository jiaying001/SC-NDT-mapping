cmake_minimum_required(VERSION 2.8.3)
project(lidar_localizer)

#add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)
find_package(GTSAM REQUIRED)
#find_package(OpenCV REQUIRED QUIET)

if (NOT EIGEN3_FOUND)
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        tf
        pcl_ros
        sensor_msgs
        pcl_conversions
        message_generation
        )

###################################
## catkin specific configuration ##
###################################

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
        CATKIN_DEPENDS std_msgs  
        DEPENDS PCL
        message_runtime
        message_generation
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")


add_executable(ndt_mapping src/ndt_mapping_node.cpp src/ndt_mapping.cpp src/Scancontext.cpp)
target_link_libraries(ndt_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARIES} gtsam)
add_dependencies(ndt_mapping ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)



