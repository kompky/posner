
cmake_minimum_required(VERSION 2.6)

project(posner-paradigm)

find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

add_subdirectory(posner-manager)
add_subdirectory(screen-handler)

icubcontrib_add_uninstall_target()

