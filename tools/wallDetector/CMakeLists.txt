cmake_minimum_required(VERSION 3.0)
project(wallDetector)
message("process walldetector project")
set(CMAKE_CXX_STANDARD 17)
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug)
ENDIF ()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -MD -bigobj")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -MD -bigobj")
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
list(APPEND VCPKG_FEATURE_FLAGS "versions")

find_package(PythonLibs REQUIRED)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Eigen3 REQUIRED)
find_package(gsl REQUIRED)

include_directories(
${OpenCV_INCLUDE_DIRS}
        C:\\Users\\nulyc\\vcpkg\\packages\\gsl_x64-windows\\include
        C:\\Users\\nulyc\\vcpkg\\packages\\gsl_x64-windows\\lib
        ${PROJECT_SOURCE_DIR}
        C:\\Users\\nulyc\\vcpkg\\packages\\eigen3_x86-windows\\include\\eigen3
        
)

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCE_DIR}/WallDetection.cpp
    ${PROJECT_SOURCE_DIR}/IncludeLibraries.h
    ${PROJECT_SOURCE_DIR}/WallDetection.h
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        Threads::Threads Boost::boost Boost::serialization Boost::system ${PYTHON_LIBRARIES}
        Eigen3::Eigen
        C:\\Users\\nulyc\\vcpkg\\packages\\gsl_x64-windows\\lib
        )