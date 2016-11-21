add_library(ui
        main_window.cpp main_window.h
        opengl_window.cpp opengl_window.h
        new_project_widget.cpp new_project_widget.h
        model_manager_widget.cpp model_manager_widget.h

        db_widgets/database_management_widget.cpp db_widgets/database_management_widget.h
        db_widgets/point_viewer_widget.cpp db_widgets/point_viewer_widget.h
        db_widgets/image_viewer_widget.cpp db_widgets/image_viewer_widget.h
        db_widgets/matches_widget.cpp db_widgets/matches_widget.h)