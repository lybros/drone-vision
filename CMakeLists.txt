cmake_minimum_required(VERSION 2.8.11)
project(3d_reconstruction)

if (POLICY CMP0043)
    cmake_policy(SET CMP0043 OLD)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_PREFIX_PATH $ENV{HOME}/Qt/5.7/clang_64/lib/cmake/)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

#find_package(Eigen3 REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost COMPONENTS program_options filesystem system thread chrono regex REQUIRED)
find_package(OpenMP)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Glew REQUIRED)
find_package(Threads REQUIRED)
find_package(JPEG REQUIRED)

qt5_add_resources(QT_RESOURCES res/resources.qrc)

include_directories(
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDES}
        ${Boost_INCLUDE_DIR}
        ${GLUT_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIRS}
        ${JPEG_INCLUDE_DIR}
)

IF (OPENMP_FOUND)
    ADD_DEFINITIONS("-DOPENMP_FOUND")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF (OPENMP_FOUND)

add_subdirectory(3rdparty/sqlite)
add_subdirectory(3rdparty/sift_gpu)
add_subdirectory(3rdparty/pmvs)
add_subdirectory(3rdparty/poisson_triangulation)

add_subdirectory(src)
