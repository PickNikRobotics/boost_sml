cmake_minimum_required(VERSION 2.8.3)
project(boost_sml)

# C++ 14
add_compile_options(-std=c++14)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
)
catkin_python_setup()

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

#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
    include/boost/
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
