PROJECT(imu_an_spatial)
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
INCLUDE(imu_an_spatialBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

