cmake_minimum_required(VERSION 3.0.2)
project(robot_client)

add_compile_options(-std=c++2a)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  std_srvs
)

find_package(Boost REQUIRED COMPONENTS graph)
find_package(PkgConfig REQUIRED)

catkin_package(
   INCLUDE_DIRS include/robot_client
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp  geometry_msgs std_srvs
   )

include_directories(${catkin_INCLUDE_DIRS})


add_library(drivers
    src/utils/tinyxml2.cpp
    src/communication/client.cpp
    src/communication/common.cpp
    src/communication/pipe_ret_t.cpp
    src/communication/tcp_client.cpp
    src/communication/tcp_server.cpp
    src/drivers/kincoMotor/KincoMotor.cpp
    src/drivers/MoonsMotor/MoonsMotor.cpp
    src/kinematics/DifferentialDrive/DifferentialDrive.cpp
    )

target_include_directories(drivers PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_client
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_client/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_client/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_client/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_client/drivers/kincoMotor
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_client/drivers/MoonsMotor
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_client/kinematics/DifferentialDrive
    )

add_executable(${PROJECT_NAME} src/mainWrapper.cpp src/DeviceManager.cpp)

target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} drivers modbus -lpthread)

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
