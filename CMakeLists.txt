cmake_minimum_required(VERSION 2.8.3)
project(ricoh_theta_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  DualFisheye2Equirectangular.srv
)

generate_messages(
   DEPENDENCIES
   sensor_msgs
)

find_package(OpenCV REQUIRED)
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(perspective_converter src/perspective_converter.cpp)
target_link_libraries(perspective_converter ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stitching src/stitching.cpp)
target_link_libraries(stitching ${catkin_LIBRARIES} ${OpenCV_LIBS})
