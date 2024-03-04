# Install script for directory: /home/vitaly/theta_star_ws/src/ThetaStarROS

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/vitaly/theta_star_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/vitaly/theta_star_ws/build/ThetaStarROS/catkin_generated/installspace/theta_star_planner.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/theta_star_planner/cmake" TYPE FILE FILES
    "/home/vitaly/theta_star_ws/build/ThetaStarROS/catkin_generated/installspace/theta_star_plannerConfig.cmake"
    "/home/vitaly/theta_star_ws/build/ThetaStarROS/catkin_generated/installspace/theta_star_plannerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/theta_star_planner" TYPE FILE FILES "/home/vitaly/theta_star_ws/src/ThetaStarROS/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner" TYPE EXECUTABLE FILES "/home/vitaly/theta_star_ws/devel/lib/theta_star_planner/theta_star_planner_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_node"
         OLD_RPATH "/home/vitaly/theta_star_ws/devel/lib:/home/vitaly/ros_catkin_ws/devel_isolated/tf/lib:/home/vitaly/ros_catkin_ws/devel_isolated/tf2_ros/lib:/home/vitaly/ros_catkin_ws/devel_isolated/actionlib/lib:/home/vitaly/ros_catkin_ws/devel_isolated/message_filters/lib:/home/vitaly/ros_catkin_ws/devel_isolated/tf2/lib:/home/vitaly/ros_catkin_ws/devel_isolated/roscpp/lib:/home/vitaly/ros_catkin_ws/devel_isolated/rosconsole/lib:/home/vitaly/ros_catkin_ws/devel_isolated/xmlrpcpp/lib:/home/vitaly/ros_catkin_ws/devel_isolated/roscpp_serialization/lib:/home/vitaly/ros_catkin_ws/devel_isolated/rostime/lib:/home/vitaly/ros_catkin_ws/devel_isolated/cpp_common/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner" TYPE EXECUTABLE FILES "/home/vitaly/theta_star_ws/devel/lib/theta_star_planner/theta_star_planner_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_test"
         OLD_RPATH "/home/vitaly/ros_catkin_ws/devel_isolated/tf/lib:/home/vitaly/ros_catkin_ws/devel_isolated/tf2_ros/lib:/home/vitaly/ros_catkin_ws/devel_isolated/actionlib/lib:/home/vitaly/ros_catkin_ws/devel_isolated/message_filters/lib:/home/vitaly/ros_catkin_ws/devel_isolated/tf2/lib:/home/vitaly/ros_catkin_ws/devel_isolated/roscpp/lib:/home/vitaly/ros_catkin_ws/devel_isolated/rosconsole/lib:/home/vitaly/ros_catkin_ws/devel_isolated/xmlrpcpp/lib:/home/vitaly/ros_catkin_ws/devel_isolated/roscpp_serialization/lib:/home/vitaly/ros_catkin_ws/devel_isolated/rostime/lib:/home/vitaly/ros_catkin_ws/devel_isolated/cpp_common/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/theta_star_planner/theta_star_planner_test")
    endif()
  endif()
endif()

