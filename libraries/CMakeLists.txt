# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME ${EVENTDRIVEN_LIBRARIES})
project(${PROJECTNAME})

set(folder_source
  src/vBottle.cpp
  src/vCodec.cpp
  src/vWindow.cpp
  src/vTempWindow.cpp
  src/vSurface.cpp
  src/vQueue.cpp
)

set(folder_header
  include/iCub/eventdriven/all.h
  include/iCub/eventdriven/vBottle.h
  include/iCub/eventdriven/vCodec.h
  include/iCub/eventdriven/vtsHelper.h
  include/iCub/eventdriven/vWindow.h
  include/iCub/eventdriven/vSurface.h
  include/iCub/eventdriven/vQueue.h
)

# Declare groups of source and header files -- makes things pretty in MSVC.
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
)

# Create everything needed to build our library
add_library(${PROJECTNAME} ${folder_source} ${folder_header})

# Makes VS a bit quiter wrt warnings.
if(MSVC)
    set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
endif(MSVC)

if(V_10BITCODEC)
    #message("Using 10bit codec")
    set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_DEFINITIONS TENBITCODEC)
endif(V_10BITCODEC)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

if(ICUBcontrib_FOUND)
    icubcontrib_export_library(${PROJECTNAME}
        INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
        EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS}
        DESTINATION include/event-driven
        FILES  ${folder_header}
    )
endif(ICUBcontrib_FOUND)