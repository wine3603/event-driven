cmake_minimum_required(VERSION 2.6)

if(NOT V_BUILD_ZYNQ)
    add_subdirectory(processing)
    add_subdirectory(visualization)
    #add_subdirectory(obsolete)
    if(ICUB_FOUND)
        add_subdirectory(movement)
    endif(ICUB_FOUND)

endif(NOT V_BUILD_ZYNQ)

if(V_USE_QTCREATOR)
    #add_custom_target(doctoken SOURCES ${xmldocumentation})
    add_subdirectory(hardwareio)
endif(V_USE_QTCREATOR)

file(GLOB xmldocumentation ${CMAKE_CURRENT_SOURCE_DIR}/hardwareio/*/*.xml)
