# Install script for directory: /home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/catkin/profile.d" TYPE FILE FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_base/catkin_generated/installspace/90-uol_cmp9767m_base.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/catkin_env_hook" TYPE FILE FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_base/catkin_generated/installspace/90-uol_cmp9767m_base.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_base/catkin_generated/installspace/uol_cmp9767m_base.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/cmake" TYPE FILE FILES
    "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_base/catkin_generated/installspace/uol_cmp9767m_baseConfig.cmake"
    "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/build/uol_cmp9767m_base/catkin_generated/installspace/uol_cmp9767m_baseConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base" TYPE FILE FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uol_cmp9767m_base" TYPE PROGRAM FILES
    "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/tests/simple_test.py"
    "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/scripts/sprayer.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libActorCollisionsPlugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libActorCollisionsPlugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libActorCollisionsPlugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/devel/lib/libActorCollisionsPlugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libActorCollisionsPlugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libActorCollisionsPlugin.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libActorCollisionsPlugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/tests" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/tests/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/config" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/launch" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/models" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/models/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/urdf" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/urdf/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/worlds" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/worlds/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/maps" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/maps/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uol_cmp9767m_base/rviz" TYPE DIRECTORY FILES "/home/jack/Documents/agriforwards/msc/robotprogramming/Autonomous-Weed-Detection/Autonomous-Weed-Detection/catkin_ws/src/uol_cmp9767m_base/rviz/")
endif()

