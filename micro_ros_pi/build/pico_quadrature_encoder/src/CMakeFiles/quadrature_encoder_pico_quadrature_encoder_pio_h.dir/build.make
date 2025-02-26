# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Documents/Omni_Bot_PI/micro_ros_pi

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build

# Utility rule file for quadrature_encoder_pico_quadrature_encoder_pio_h.

# Include any custom commands dependencies for this target.
include pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/compiler_depend.make

# Include the progress variables for this target.
include pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/progress.make

pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h: pico_quadrature_encoder/src/quadrature_encoder.pio.h

pico_quadrature_encoder/src/quadrature_encoder.pio.h: /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/pico_quadrature_encoder/src/quadrature_encoder.pio
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating quadrature_encoder.pio.h"
	cd /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/pico_quadrature_encoder/src && ../../pioasm-install/pioasm/pioasm -o c-sdk -v 0 /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/pico_quadrature_encoder/src/quadrature_encoder.pio /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/pico_quadrature_encoder/src/quadrature_encoder.pio.h

quadrature_encoder_pico_quadrature_encoder_pio_h: pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h
quadrature_encoder_pico_quadrature_encoder_pio_h: pico_quadrature_encoder/src/quadrature_encoder.pio.h
quadrature_encoder_pico_quadrature_encoder_pio_h: pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/build.make
.PHONY : quadrature_encoder_pico_quadrature_encoder_pio_h

# Rule to build all files generated by this target.
pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/build: quadrature_encoder_pico_quadrature_encoder_pio_h
.PHONY : pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/build

pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/clean:
	cd /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/pico_quadrature_encoder/src && $(CMAKE_COMMAND) -P CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/cmake_clean.cmake
.PHONY : pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/clean

pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/depend:
	cd /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Documents/Omni_Bot_PI/micro_ros_pi /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/pico_quadrature_encoder/src /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/pico_quadrature_encoder/src /home/pi/Documents/Omni_Bot_PI/micro_ros_pi/build/pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pico_quadrature_encoder/src/CMakeFiles/quadrature_encoder_pico_quadrature_encoder_pio_h.dir/depend

