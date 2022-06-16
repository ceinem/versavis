cmake_minimum_required(VERSION 3.5)
project(serial_msgs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_message_files(
    FILES
    Sentence.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)
