# Copyright: 2017 iCub Facility, Istituto Italiano di Tecnologia
# Author: Kyveli Kompatsiari
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)
project(posner-manager)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()

set(doc ${PROJECT_NAME}.xml)
source_group("DOC Files" FILES ${doc})

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp ${doc})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
yarp_install(FILES ${doc} DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

add_subdirectory(app)
