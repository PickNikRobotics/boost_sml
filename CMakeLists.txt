cmake_minimum_required(VERSION 2.8.3)
project(boost_sml)

# C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_example
  CATKIN_DEPENDS
    roscpp
  INCLUDE_DIRS
    include
  DEPENDS
    Boost
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
    ${Boost_INCLUDE_DIR}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Example Library that uses boost_sml
add_library(
  ${PROJECT_NAME}_example
  src/example_library.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_example
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Example graph executable
add_executable(sml_graph_demo
  src/sml_graph_demo.cpp
)
target_link_libraries(
  sml_graph_demo
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# Install library
install(
  TARGETS
    ${PROJECT_NAME}_example
    sml_graph_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()

## Test for correct C++ source code
roslint_cpp()
