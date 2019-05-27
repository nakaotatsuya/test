cmake_minimum_required(VERSION 2.8.3)
project(fft)

find_package(catkin REQUIRED
  message_generation)

add_message_files(
  FILES
  SpectrumData.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
CATKIN_DEPENDS message_runtime
)
