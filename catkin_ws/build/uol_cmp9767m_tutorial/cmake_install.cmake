# Install script for directory: /home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_tutorial

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_tutorial/srv" TYPE FILE FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_tutorial/srv/AddTwoInts.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_tutorial/cmake" TYPE FILE FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_tutorial/catkin_generated/installspace/uol_cmp9767m_tutorial-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/devel/include/uol_cmp9767m_tutorial")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/devel/share/roseus/ros/uol_cmp9767m_tutorial")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/devel/share/common-lisp/ros/uol_cmp9767m_tutorial")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/devel/share/gennodejs/ros/uol_cmp9767m_tutorial")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/devel/lib/python2.7/dist-packages/uol_cmp9767m_tutorial")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/devel/lib/python2.7/dist-packages/uol_cmp9767m_tutorial")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_tutorial/catkin_generated/installspace/uol_cmp9767m_tutorial.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_tutorial/cmake" TYPE FILE FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_tutorial/catkin_generated/installspace/uol_cmp9767m_tutorial-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_tutorial/cmake" TYPE FILE FILES
    "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_tutorial/catkin_generated/installspace/uol_cmp9767m_tutorialConfig.cmake"
    "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_tutorial/catkin_generated/installspace/uol_cmp9767m_tutorialConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_tutorial" TYPE FILE FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_tutorial/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uol_cmp9767m_tutorial" TYPE PROGRAM FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_tutorial/scripts/set_topo_nav_goal.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_tutorial/launch" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_tutorial/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_tutorial/models" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_tutorial/models/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_tutorial/config" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_tutorial/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_tutorial/maps" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_tutorial/maps/")
endif()

