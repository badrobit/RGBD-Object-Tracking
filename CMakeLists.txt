cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set( ROS_BUILD_TYPE Debug )

rosbuild_init()

#...: compiler options :............................................................................

#...: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
endif(CMAKE_COMPILER_IS_GNUCXX)

#..: Default Paths :................................................................................
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(
   ${PROJECT_SOURCE_DIR}/common/include
   ${PROJECT_SOURCE_DIR}/ros/include
)

#..: Libraries :....................................................................................

#..: platform_extraction
rosbuild_add_library( platform_extraction 
	common/src/platform_extraction.cpp )

#..: object_candidate_extraction
rosbuild_add_library( object_candidate_extraction
	common/src/object_candidate_extraction.cpp )

#..: object_candidate_tracking
rosbuild_add_library( object_candidate_tracking 
  common/src/object_candidate_tracking.cpp )
  
#..: bounding_box_tracking
rosbuild_add_library( bounding_box_tracking
 common/src/bounding_box_tracking.cpp )

#..: Executables :..................................................................................

#..: safe_object_transporation
rosbuild_add_executable( object_tracking ros/nodes/object_tracking.cpp )

target_link_libraries( object_tracking 	platform_extraction 
					object_candidate_extraction 
                                        object_candidate_tracking
                                        bounding_box_tracking )
