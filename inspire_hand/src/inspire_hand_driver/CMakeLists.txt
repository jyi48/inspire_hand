cmake_minimum_required(VERSION 3.14)
project(inspire_hand_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(inspire_hand_msgs REQUIRED)

# libmodbus — PkgConfig 있으면 우선 사용, 없으면 find_library fallback
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(MODBUS libmodbus)
endif()

if(NOT MODBUS_FOUND)
  find_library(MODBUS_LIBRARIES NAMES modbus REQUIRED)
  find_path(MODBUS_INCLUDE_DIRS NAMES modbus.h PATH_SUFFIXES modbus REQUIRED)
  message(STATUS "libmodbus found via find_library: ${MODBUS_LIBRARIES}")
else()
  message(STATUS "libmodbus found via pkg-config: ${MODBUS_LIBRARIES}")
endif()

add_executable(inspire_hand_driver
  src/modbus_client.cpp
  src/inspire_hand_driver.cpp
  src/main.cpp
)

target_include_directories(inspire_hand_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS}
)

target_link_libraries(inspire_hand_driver
  ${MODBUS_LIBRARIES}
)

ament_target_dependencies(inspire_hand_driver
  rclcpp
  inspire_hand_msgs
)

install(TARGETS inspire_hand_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
