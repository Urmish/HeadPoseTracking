# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Produce verbose output by default.
VERBOSE = 1

# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kush/766/HeadPoseTracking/template_generator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kush/766/HeadPoseTracking/template_generator

# Include any dependencies generated for this target.
include CMakeFiles/cvni.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cvni.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cvni.dir/flags.make

CMakeFiles/cvni.dir/openni_capture.cpp.o: CMakeFiles/cvni.dir/flags.make
CMakeFiles/cvni.dir/openni_capture.cpp.o: openni_capture.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/kush/766/HeadPoseTracking/template_generator/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/cvni.dir/openni_capture.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cvni.dir/openni_capture.cpp.o -c /home/kush/766/HeadPoseTracking/template_generator/openni_capture.cpp

CMakeFiles/cvni.dir/openni_capture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cvni.dir/openni_capture.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/kush/766/HeadPoseTracking/template_generator/openni_capture.cpp > CMakeFiles/cvni.dir/openni_capture.cpp.i

CMakeFiles/cvni.dir/openni_capture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cvni.dir/openni_capture.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/kush/766/HeadPoseTracking/template_generator/openni_capture.cpp -o CMakeFiles/cvni.dir/openni_capture.cpp.s

CMakeFiles/cvni.dir/openni_capture.cpp.o.requires:
.PHONY : CMakeFiles/cvni.dir/openni_capture.cpp.o.requires

CMakeFiles/cvni.dir/openni_capture.cpp.o.provides: CMakeFiles/cvni.dir/openni_capture.cpp.o.requires
	$(MAKE) -f CMakeFiles/cvni.dir/build.make CMakeFiles/cvni.dir/openni_capture.cpp.o.provides.build
.PHONY : CMakeFiles/cvni.dir/openni_capture.cpp.o.provides

CMakeFiles/cvni.dir/openni_capture.cpp.o.provides.build: CMakeFiles/cvni.dir/openni_capture.cpp.o

# Object files for target cvni
cvni_OBJECTS = \
"CMakeFiles/cvni.dir/openni_capture.cpp.o"

# External object files for target cvni
cvni_EXTERNAL_OBJECTS =

cvni: CMakeFiles/cvni.dir/openni_capture.cpp.o
cvni: CMakeFiles/cvni.dir/build.make
cvni: /usr/local/lib/libopencv_core.so.2.4.9
cvni: /usr/local/lib/libopencv_imgproc.so.2.4.9
cvni: /usr/local/lib/libopencv_highgui.so.2.4.9
cvni: /usr/local/lib/libopencv_objdetect.so.2.4.9
cvni: /usr/local/lib/libopencv_imgproc.so.2.4.9
cvni: /usr/local/lib/libopencv_calib3d.so.2.4.9
cvni: /usr/local/lib/libopencv_features2d.so.2.4.9
cvni: /home/kush/Libraries/libpointmatcher/build/libpointmatcher.so
cvni: /usr/lib/x86_64-linux-gnu/libboost_thread.so
cvni: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
cvni: /usr/lib/x86_64-linux-gnu/libboost_system.so
cvni: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
cvni: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cvni: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
cvni: /usr/lib/x86_64-linux-gnu/libpthread.so
cvni: /usr/local/lib/libnabo.a
cvni: /home/kush/Libraries/libpointmatcher/build/contrib/yaml-cpp-pm/libyaml-cpp-pm.a
cvni: /usr/local/lib/libopencv_highgui.so.2.4.9
cvni: /usr/local/lib/libopencv_imgproc.so.2.4.9
cvni: /usr/local/lib/libopencv_flann.so.2.4.9
cvni: /usr/local/lib/libopencv_core.so.2.4.9
cvni: CMakeFiles/cvni.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable cvni"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cvni.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cvni.dir/build: cvni
.PHONY : CMakeFiles/cvni.dir/build

CMakeFiles/cvni.dir/requires: CMakeFiles/cvni.dir/openni_capture.cpp.o.requires
.PHONY : CMakeFiles/cvni.dir/requires

CMakeFiles/cvni.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cvni.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cvni.dir/clean

CMakeFiles/cvni.dir/depend:
	cd /home/kush/766/HeadPoseTracking/template_generator && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kush/766/HeadPoseTracking/template_generator /home/kush/766/HeadPoseTracking/template_generator /home/kush/766/HeadPoseTracking/template_generator /home/kush/766/HeadPoseTracking/template_generator /home/kush/766/HeadPoseTracking/template_generator/CMakeFiles/cvni.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cvni.dir/depend

