cmake_minimum_required(VERSION 2.8.3)
project(ik_module)
# set(CMAKE_CXX_COMPILER "clang++-8")
# set(CMAKE_C_COMPILER "clang-8")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cmake_modules
  angles
  dynamixel_sdk
  dynamixel_workbench_toolbox
  bodyhub
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
 
################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  PoseArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SrvPoses.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test1
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###############serial###########################
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)
if(APPLE)
  # If OSX
  list(APPEND serial_SRCS src/impl/unix.cc)
  list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

## Add serial library
add_library(${PROJECT_NAME} ${serial_SRCS})
if(APPLE)
  target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
  target_link_libraries(${PROJECT_NAME} rt pthread)
else()
  target_link_libraries(${PROJECT_NAME} setupapi)
endif()

###############serial###########################

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/walkPlaning/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/RBDyn/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/SpaceVecAlg/

# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

#link_directories(/opt/ros/indigo/lib)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/test1.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_definitions(-std=c++14 )
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node  
              src/ikModule_node.cpp                         src/Util.cpp                        src/SimControll.cpp   src/Kalman.cpp 
              src/walkPlaning/LIPMWalk.cpp
              # src/walkPlaning/LIPMWalk.cpp         src/walkPlaning/CPWalk.cpp
              # src/walkPlaning/Walking.cpp          src/walkPlaning/CPWalking.cpp      src/walkPlaning/CPWalking1.cpp  src/walkPlaning/CPWalking2.cpp
              src/walkPlaning/Kinematics.cpp          src/walkPlaning/TalosRobot.cpp  
              src/inertialDataFilter.cpp           src/LIPStateEstimator.cpp

              src/serial.cc src/impl/unix.cc 
              src/impl/list_ports/list_ports_linux.cc

              # ####RBDyn
              src/RBDyn/MultiBody.cpp  src/RBDyn/MultiBodyConfig.cpp   src/RBDyn/MultiBodyGraph.cpp
              src/RBDyn/FK.cpp         src/RBDyn/IK.cpp                src/RBDyn/Jacobian.cpp
              src/RBDyn/FA.cpp         src/RBDyn/FD.cpp                src/RBDyn/IS.cpp 
              src/RBDyn/FV.cpp         src/RBDyn/ID.cpp                src/RBDyn/IDIM.cpp
              
              src/RBDyn/Momentum.cpp   src/RBDyn/CoM.cpp               src/RBDyn/ZMP.cpp
              src/RBDyn/VisServo.cpp   src/RBDyn/Coriolis.cpp          src/RBDyn/EulerIntegration.cpp
                            )

add_executable(ik_module_puthandsdown  
              examples/ik_module_puthandsdown.cpp            src/Util.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node -ldl  -lrt #-lcurses
   ${catkin_LIBRARIES}
	 ${EIGEN_LIBRARIES}
   # ${CMAKE_CURRENT_SOURCE_DIR}/include/RBDyn/libSpaceVecAlg.a
   # ${CMAKE_CURRENT_SOURCE_DIR}/include/RBDyn/libRBDyn.so
 )
 target_link_libraries(ik_module_puthandsdown -ldl  -lrt #-lcurses
 ${catkin_LIBRARIES}
 ${EIGEN_LIBRARIES}
 # ${CMAKE_CURRENT_SOURCE_DIR}/include/RBDyn/libSpaceVecAlg.a
 # ${CMAKE_CURRENT_SOURCE_DIR}/include/RBDyn/libRBDyn.so
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
