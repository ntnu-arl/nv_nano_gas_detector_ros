cmake_minimum_required(VERSION 2.8.3)
project(gas_detect_driver_node)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
) 
add_message_files(
  FILES
  gas_detect.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories( 
  include 
	${catkin_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}
    src/uart.c
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_main 
  src/uart.c
	src/gas_detect_driver_main.cpp
)

target_link_libraries(${PROJECT_NAME}_main
	${PROJECT_NAME}
)

