#ifndef INC_3D_RECONSTRUCTION_MAINWINDOW_H
#define INC_3D_RECONSTRUCTION_MAINWINDOW_H

#include "../controllers.h"
#include "../densify.h"
#include "../surface_reconstruct.h"

#include "db_widgets/database_management_widget.h"
#include "model_manager_widget.h"
#include "new_project_widget.h"
#include "opengl_window.h"

#include <QtWidgets>

#include <boost/filesystem.hpp>

class NewProjectWidget;

class MainWindow : public QMainWindow {
public:
    MainWindow(const std::string& binary_path);

    bool OverwriteReconstruction();

    std::unique_ptr<IncrementalMapperController> mapper_controller;

    void ReadProjectConfiguration(const std::string&);
    void WriteProjectConfiguration();

    void UpdateProjectInfoStatusBar();

protected:
    void showEvent(QShowEvent* event);

    void moveEvent(QMoveEvent* event);

    void closeEvent(QCloseEvent* event);

    void afterShowEvent();

private:
    void CreateWidgets();

    void CreateActions();

    void CreateToolbar();

    void CreateStatusbar();

    void CreateControllers();

    void CreateFutures();

    void CreateProgressBar();

    void CenterProgressBar();

    void NewProject();

    void ImportModel();

    void ImportModelFinished();

    void ExportModel();

    void ExportModelFinished();

    void OpenProject();

    bool IsValidProjectDirectory(const std::string&);

    void FeatureExtraction();

    void FeatureMatching();

    void DatabaseManagement();

    void ReconstructionStart();

    void ReconstructionPause();

    void ReconstructionReset();

    void ReconstructionFinish();

    void Render();

    void RenderNow();

    void RenderSelectedModel();

    void RenderClear();

    void SelectModelIdx(const size_t);

    size_t SelectedModelIdx();

    bool HasSelectedModel();

    bool IsSelectedModelValid();

    void DensifyModel();

    void SurfaceReconstructModel();

    void ShowInvalidProjectError();

    void UpdateTimer();

    void EnableBlockingActions();

    void DisableBlockingActions();

    void UpdateWindowTitle();

    OptionManager options_;

    OpenGLWindow* opengl_window_;

    Timer timer_;

    QTimer* after_show_event_timer_;

    NewProjectWidget* new_project_widget_;
    DatabaseManagementWidget* database_management_widget_;
    ModelManagerWidget* model_manager_widget_;

    QToolBar* project_toolbar_;
    QToolBar* import_export_toolbar_;
    QToolBar* preprocessing_toolbar_;
    QToolBar* reconstruction_toolbar_;
    QToolBar* render_toolbar_;

    QTimer* statusbar_timer_;
    QLabel* statusbar_timer_label_;

    QLabel* project_info_label_;

    QAction* action_new_project_;
    QAction* action_open_project_;
    QAction* action_import_model_;
    QAction* action_export_model_;
    QAction* action_quit_;

    QAction* action_feature_extraction_;
    QAction* action_feature_matching_;
    QAction* action_database_management_;

    QAction* action_reconstruction_start_;
    QAction* action_reconstruction_pause_;
    QAction* action_reconstruction_reset_;
    QAction* action_reconstruction_finish_;

    QAction* action_render_;
    QAction* action_render_now_;
    QAction* action_render_reset_view_;

    QAction* action_densify_;
    QAction* action_surface_reconstruct_;

    QProgressDialog* progress_bar_;

    QFutureWatcher<void>* import_model_watcher_;
    QFutureWatcher<void>* export_model_watcher_;

    std::vector<QAction*> blocking_actions_;

    std::string binary_path_;
    std::string working_directory_;

    size_t render_counter_;

    bool window_closed_;
};

#endif //INC_3D_RECONSTRUCTION_MAINWINDOW_H
