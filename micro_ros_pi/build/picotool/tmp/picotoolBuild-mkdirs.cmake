# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/_deps/picotool-src"
  "/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/_deps/picotool-build"
  "/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/_deps"
  "/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/picotool/tmp"
  "/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/picotool/src/picotoolBuild-stamp"
  "/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/picotool/src"
  "/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/picotool/src/picotoolBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/picotool/src/picotoolBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/picotool/src/picotoolBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
