cmake_minimum_required(VERSION 2.8.3)
project(hercules_file_player)

add_definitions(-std=c++14)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE release)
endif ()

find_package(catkin REQUIRED cmake_modules
  COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  rosbag
  image_transport
  cv_bridge
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
  pcl_msgs
  camera_info_manager
  tf
)
set(CMAKE_AUTOMOC ON)

find_package(Eigen REQUIRED)

set (CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)

#find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5OpenGL)


add_message_files(
  DIRECTORY msg
)

generate_messages(
   DEPENDENCIES
    std_msgs
    geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/hercules_file_player.cfg
  #...
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    hercules_file_player
  CATKIN_DEPENDS 
    roscpp rospy 
    std_msgs 
    geometry_msgs 
    message_runtime 
    image_transport 
    cv_bridge 
    dynamic_reconfigure 
    pcl_ros
    pcl_conversions  
    pcl_msgs
    camera_info_manager
    tf

  DEPENDS 
    Eigen
)


set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set (hercules_file_player_QTLib_src ${SRC_DIR}/mainwindow.cpp ${SRC_DIR}/ROSThread.cpp)
set (hercules_file_player_QTLib_hdr ${SRC_DIR}/mainwindow.h ${SRC_DIR}/ROSThread.h)
set (hercules_file_player_QTLib_ui  ${SRC_DIR}/mainwindow.ui)
set (hercules_file_player_QTBin_src ${SRC_DIR}/main.cpp)



#find_package(GLEW REQUIRED)
#find_package(GLUT REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${hercules_file_player_INCLUDE_DIRS}
  ${SRC_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}
  include
  ${Eigen_INCLUDE_DIRS}
)


#qt5_wrap_cpp(hercules_file_player_QTLib_hdr_moc ${hercules_file_player_QTLib_hdr})
qt5_wrap_ui (hercules_file_player_QTLib_ui_moc  ${hercules_file_player_QTLib_ui})
qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)



###########
## Build ##
###########


add_executable(hercules_file_player ${hercules_file_player_QTLib_src} ${hercules_file_player_QTLib_hdr} ${hercules_file_player_QTBin_src} ${SHADER_RSC_ADDED} ${hercules_file_player_QTLib_ui_moc})     

add_dependencies(hercules_file_player hercules_file_player_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(hercules_file_player ${catkin_EXPORTED_TARGETS})

target_link_libraries(hercules_file_player

  ${catkin_LIBRARIES}
  ${QT_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  Qt5::Widgets 
  Qt5::Gui 
  Qt5::OpenGL
  GL
  ${BOOST_CUSTOM_LIBS}
  ${Eigen_LIBRARIES}
)



