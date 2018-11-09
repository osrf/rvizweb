cmake_minimum_required(VERSION 2.8.3)
project(rvizweb)

find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
  roswww
  tf2_web_republisher
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

execute_process(
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME}
  COMMAND mkdir -p www
)

execute_process(
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/www

  # Copy needed files
  COMMAND cp ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/index.html .
  COMMAND cp ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/bower.json .

  # Install bower components
  COMMAND bower install
)

execute_process(
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/www

  # Build components using polymer
  COMMAND polymer build --name www
)

install(
  DIRECTORY
    launch
    ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/www/build/www
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

