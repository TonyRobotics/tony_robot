cmake_minimum_required(VERSION 2.8.3)
project(trd_diff_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(trd_diff_controller
#   src/${PROJECT_NAME}/trd_diff_controller.cpp
# )
# add_dependencies(trd_diff_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(trd_diff_controller_node
    src/trd_diff_controller_node.cpp
    src/trd_diff_controller.cpp
    src/message_manager.cpp
    src/serial_port.cpp
)
add_dependencies(trd_diff_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trd_diff_controller_node
  ${catkin_LIBRARIES}
)

