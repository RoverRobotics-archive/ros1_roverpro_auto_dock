cmake_minimum_required(VERSION 2.8.3)
project(rr_auto_dock)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
  message_generation
  nodelet
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

add_message_files(
   FILES
   Fiducial.msg
   FiducialArray.msg
   FiducialTransform.msg
   FiducialTransformArray.msg
   FiducialMapEntry.msg
   FiducialMapEntryArray.msg
)

add_service_files(
  FILES
  InitializeMap.srv
)

catkin_package(
  LIBRARIES image_sub_nodelet
  INCLUDE_DIRS DEPENDS OpenCV
  CATKIN_DEPENDS nodelet message_runtime roscpp std_msgs
)


###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(image_sub_node src/image_sub_node.cpp)
add_executable(aruco_detect src/aruco_detect.cpp)
add_executable(create_marker src/create_marker.cpp)

add_dependencies(aruco_detect ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(aruco_detect ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(create_marker ${OpenCV_LIBS})
target_link_libraries(image_sub_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# image_sub_nodelet  Stuff
add_library(image_sub_nodelet src/image_sub_nodelet.cpp)
target_link_libraries(image_sub_nodelet ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(image_sub_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS aruco_detect create_marker image_sub_node image_sub_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS image_sub_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# image_nodelet_stuff Copy xml files for
install(FILES nodelet_plugins.xml nodelet.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/create_markers.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
) 

###########
## Tests ##
###########

if(CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED)

        # Tests need c++11
        add_definitions(-std=c++11)
        
        add_rostest_gtest(aruco_images_test 
          test/aruco_images.test 
          test/aruco_images_test.cpp)
        target_link_libraries(aruco_images_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()