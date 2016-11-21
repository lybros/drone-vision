set(SOURCE_FILES
        main.cpp
        options.h options.cpp
        features.h features.cpp
        utils.h utils.cpp
        projection.h projection.cpp
        estimators.h estimators.cpp
        optimization.h optimization.cpp
        reconstruction.h reconstruction.cpp
        storage.h storage.cpp
        geometry.h geometry.cpp
        refinement.h refinement.cpp
        densify.h densify.cpp
        surface_reconstruct.h surface_reconstruct.cpp)

add_executable(3d_reconstruction ${SOURCE_FILES} ${QT_RESOURCES})

add_subdirectory(ui)
add_subdirectory(entities)
add_subdirectory(processor)

target_link_libraries(3d_reconstruction
        ui entities processor
        pthread dl sqlite sift_gpu pmvs_lib cmvs_lib tinycthread image_lib graclus_lib nlopt numeric_lib
        ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${FREEIMAGE_LIBRARIES} ${GLEW_LIBRARIES}
        ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${JPEG_LIBRARIES})

qt5_use_modules(3d_reconstruction Core OpenGL)