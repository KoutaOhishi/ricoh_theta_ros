cmake_minimum_required(VERSION 2.8.3)
project(ricoh_theta_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(img_show src/img_show.cpp)
target_link_libraries(img_show ${catkin_LIBRARIES} ${OpenCV_LIBS})
