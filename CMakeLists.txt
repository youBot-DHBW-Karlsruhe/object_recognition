cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 tf
 std_msgs
 find_object_2d
 geometry_msgs
 message_generation
)

add_message_files(
 FILES
 ObjectPosition.msg
)
generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS std_msgs
 message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_recognition
    src/ObjectListener.cpp
)
add_dependencies(object_recognition
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(object_recognition
	${catkin_LIBRARIES}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

