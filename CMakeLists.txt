cmake_minimum_required(VERSION 2.8.11)
project(3d_reconstruction)

if(POLICY CMP0043)
    cmake_policy(SET CMP0043 OLD)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_PREFIX_PATH /home/kolina93/Qt/5.6/gcc_64/lib/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(FreeImage REQUIRED)
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
qt5_add_resources(QT_RESOURCES resources.qrc)

include_directories(
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${FREEIMAGE_INCLUDE_DIRS}
        ${CERES_INCLUDES}
        ${Boost_INCLUDE_DIR}
        ${GLUT_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIRS}
        ${JPEG_INCLUDE_DIR}
)

IF(OPENMP_FOUND)
    ADD_DEFINITIONS("-DOPENMP_FOUND")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF(OPENMP_FOUND)

add_subdirectory(sqlite)
add_subdirectory(sift_gpu)
add_subdirectory(pmvs)
add_subdirectory(poisson_triangulation)

set(SOURCE_FILES main.cpp user_interface.h user_interface.cpp controllers.h controllers.cpp options.h options.cpp
        features.h features.cpp entities.h entities.cpp utils.h utils.cpp projection.h projection.cpp estimators.h
        estimators.cpp polygons.h coefficients.h optimization.h optimization.cpp model.h model.cpp storage.h
        storage.cpp geometry.h geometry.cpp processor.h processor.cpp refinement.h refinement.cpp densify.h densify.cpp
        surface_reconstruct.h surface_reconstruct.cpp)
add_executable(3d_reconstruction ${SOURCE_FILES} ${QT_RESOURCES})
target_link_libraries(3d_reconstruction pthread dl ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} ${Boost_LIBRARIES}
        ${FREEIMAGE_LIBRARIES} ${GLEW_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}
        sqlite sift_gpu pmvs_lib cmvs_lib tinycthread image_lib ${JPEG_LIBRARIES} graclus_lib nlopt numeric_lib)
qt5_use_modules(3d_reconstruction Core OpenGL)
