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
        surface_reconstruct.h surface_reconstruct.cpp

        ui/main_window.cpp ui/main_window.h
        ui/opengl_window.cpp ui/opengl_window.h
        ui/new_project_widget.cpp ui/new_project_widget.h
        ui/model_manager_widget.cpp ui/model_manager_widget.h

        ui/db_widgets/database_management_widget.cpp ui/db_widgets/database_management_widget.h
        ui/db_widgets/point_viewer_widget.cpp ui/db_widgets/point_viewer_widget.h
        ui/db_widgets/image_viewer_widget.cpp ui/db_widgets/image_viewer_widget.h
        ui/db_widgets/matches_widget.cpp ui/db_widgets/matches_widget.h

        entities/base_camera_model.h
        entities/track.cpp entities/track.h
        entities/point_3d.cpp entities/point_3d.h
        entities/camera.cpp entities/camera.h
        entities/visibility_pyramid.cpp entities/visibility_pyramid.h
        entities/point_2d.cpp entities/point_2d.h
        entities/image.cpp entities/image.h
        entities/pinhole_camera_model.cpp entities/pinhole_camera_model.h
        entities/radial_camera_model.cpp entities/radial_camera_model.h

        processor/incremental_triangulator.cpp processor/incremental_triangulator.h
        processor/incremental_mapper.cpp processor/incremental_mapper.h
        processor/incremental_mapper_controller.cpp processor/incremental_mapper_controller.h)

add_executable(3d_reconstruction ${SOURCE_FILES} ${QT_RESOURCES})

target_link_libraries(3d_reconstruction
        pthread dl sqlite sift_gpu pmvs_lib cmvs_lib tinycthread image_lib graclus_lib nlopt numeric_lib
        ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${FREEIMAGE_LIBRARIES} ${GLEW_LIBRARIES}
        ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${JPEG_LIBRARIES})

qt5_use_modules(3d_reconstruction Core OpenGL)