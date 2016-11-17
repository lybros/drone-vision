set(SOURCE_FILES main.cpp controllers.h controllers.cpp options.h options.cpp
        features.h features.cpp entities.h entities.cpp utils.h utils.cpp projection.h projection.cpp estimators.h
        estimators.cpp optimization.h optimization.cpp model.h model.cpp storage.h
        storage.cpp geometry.h geometry.cpp processor.h processor.cpp refinement.h refinement.cpp densify.h densify.cpp
        surface_reconstruct.h surface_reconstruct.cpp

        ui/main_window.cpp ui/main_window.h
        ui/opengl_window.cpp ui/opengl_window.h
        ui/new_project_widget.cpp ui/new_project_widget.h
        ui/model_manager_widget.cpp ui/model_manager_widget.h

        ui/db_widgets/database_management_widget.cpp ui/db_widgets/database_management_widget.h
        ui/db_widgets/point_viewer_widget.cpp ui/db_widgets/point_viewer_widget.h
        ui/db_widgets/image_viewer_widget.cpp ui/db_widgets/image_viewer_widget.h
        ui/db_widgets/matches_widget.cpp ui/db_widgets/matches_widget.h)

add_executable(3d_reconstruction ${SOURCE_FILES} ${QT_RESOURCES})

target_link_libraries(3d_reconstruction
        pthread dl sqlite sift_gpu pmvs_lib cmvs_lib tinycthread image_lib graclus_lib nlopt numeric_lib
        ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${FREEIMAGE_LIBRARIES} ${GLEW_LIBRARIES}
        ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${JPEG_LIBRARIES})

qt5_use_modules(3d_reconstruction Core OpenGL)