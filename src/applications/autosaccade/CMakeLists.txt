cmake_minimum_required(VERSION 2.6)

set(MODULENAME autosaccade)
project(${MODULENAME})

file(GLOB source src/*.cpp
        ../../processing/vAttention/src/vFeatureMap.cpp)
file(GLOB header include/*.h
        ../../processing/vAttention/include/vFeatureMap.h)

find_package(ICUB)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS}
                        ../../processing/vAttention/include)

add_executable(${MODULENAME} ${source} ${header})

target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})

install(TARGETS ${MODULENAME} DESTINATION bin)

yarp_install(FILES ${MODULENAME}.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
if(USE_QTCREATOR)
    add_custom_target(${MODULENAME}_token SOURCES ${MODULENAME}.ini ${MODULENAME}.xml app_${MODULENAME}.xml)
endif(USE_QTCREATOR)
