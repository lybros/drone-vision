#include "main_window.h"

MainWindow::MainWindow(const std::string& binary_path)
        : options_(),
          binary_path_(binary_path),
          working_directory_(),
          import_model_watcher_(nullptr),
          export_model_watcher_(nullptr),
          render_counter_(0),
          window_closed_(false) {
    resize(1024, 600);
    UpdateWindowTitle();

    CreateWidgets();
    CreateActions();
    CreateToolbar();
    CreateStatusbar();
    CreateControllers();
    CreateFutures();
    CreateProgressBar();

    options_.AddAllOptions();
}

bool MainWindow::OverwriteReconstruction() {
    if (mapper_controller->NumModels() > 0) {
        QMessageBox::StandardButton reply = QMessageBox::question(
                this, "",
                tr("Do you really want to overwrite the existing reconstruction?"),
                QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No) {
            return false;
        } else {
            ReconstructionReset();
        }
    }
    return true;
}

void MainWindow::showEvent(QShowEvent* event) {
    after_show_event_timer_ = new QTimer(this);
    connect(after_show_event_timer_, &QTimer::timeout, this,
            &MainWindow::afterShowEvent);
    after_show_event_timer_->start(100);
}

void MainWindow::moveEvent(QMoveEvent* event) { CenterProgressBar(); }

void MainWindow::closeEvent(QCloseEvent* event) {
    if (window_closed_) {
        event->accept();
        return;
    }

    if (mapper_controller->IsRunning()) {
        mapper_controller->Resume();
    }

    mapper_controller->Stop();
    mapper_controller->wait();
    mapper_controller->Stop();

    event->accept();

    window_closed_ = true;
}

void MainWindow::afterShowEvent() {
    after_show_event_timer_->stop();
    CenterProgressBar();
}

void MainWindow::CreateWidgets() {
    opengl_window_ = new OpenGLWindow(this, &options_);
    setCentralWidget(QWidget::createWindowContainer(opengl_window_));

    new_project_widget_ = new NewProjectWidget(this, &options_);
    new_project_widget_->SetImagePath(*options_.image_path);
    new_project_widget_->SetProjectPath(*options_.project_path);

    database_management_widget_ = new DatabaseManagementWidget(this, &options_);
    model_manager_widget_ = new ModelManagerWidget(this);
}

void MainWindow::CreateActions() {
    action_new_project_ = new QAction(QIcon(":/media/project-new.png"), tr("New project"), this);
    action_new_project_->setShortcuts(QKeySequence::New);
    connect(action_new_project_, &QAction::triggered, this, &MainWindow::NewProject);

    action_import_model_ = new QAction(QIcon(":/media/import.png"), tr("Import model"), this);
    connect(action_import_model_, &QAction::triggered, this, &MainWindow::ImportModel);
    blocking_actions_.push_back(action_import_model_);

    action_export_model_ = new QAction(QIcon(":/media/export.png"), tr("Export model"), this);
    connect(action_export_model_, &QAction::triggered, this, &MainWindow::ExportModel);
    blocking_actions_.push_back(action_export_model_);

    action_open_project_ = new QAction(QIcon(":media/project-open.png"), tr("Open existent project"), this);
    connect(action_open_project_, &QAction::triggered, this, &MainWindow::OpenProject);
    blocking_actions_.push_back(action_open_project_);

    QString recent_project_path = ReadAppConfig();
    QString recent_project_label = "Open the most recent project" +
            (recent_project_path == "" ? "" : ": " + recent_project_path);
    action_open_recent_project_ = new QAction(QIcon(":media/project-open.png"), tr(qPrintable(recent_project_label)), this);
    connect(action_open_recent_project_, &QAction::triggered, this, &MainWindow::OpenRecentProject);
    blocking_actions_.push_back(action_open_recent_project_);

    action_quit_ = new QAction(tr("Quit"), this);
    connect(action_quit_, &QAction::triggered, this, &MainWindow::close);

    action_feature_extraction_ = new QAction(QIcon(":/media/feature-extraction.png"), tr("Extract features"), this);
    connect(action_feature_extraction_, &QAction::triggered, this, &MainWindow::FeatureExtraction);
    blocking_actions_.push_back(action_feature_extraction_);

    action_feature_matching_ = new QAction(QIcon(":/media/feature-matching.png"), tr("Match features"), this);
    connect(action_feature_matching_, &QAction::triggered, this,
            &MainWindow::FeatureMatching);
    blocking_actions_.push_back(action_feature_matching_);

    action_database_management_ = new QAction(
            QIcon(":/media/database-management.png"), tr("Manage database"), this);
    connect(action_database_management_, &QAction::triggered, this,
            &MainWindow::DatabaseManagement);
    blocking_actions_.push_back(action_database_management_);

    action_reconstruction_start_ =
            new QAction(QIcon(":/media/reconstruction-start.png"),
                        tr("Start / resume reconstruction"), this);
    connect(action_reconstruction_start_, &QAction::triggered, this,
            &MainWindow::ReconstructionStart);
    blocking_actions_.push_back(action_reconstruction_start_);

    action_reconstruction_pause_ =
            new QAction(QIcon(":/media/reconstruction-pause.png"),
                        tr("Pause reconstruction"), this);
    connect(action_reconstruction_pause_, &QAction::triggered, this,
            &MainWindow::ReconstructionPause);
    action_reconstruction_pause_->setEnabled(false);
    blocking_actions_.push_back(action_reconstruction_pause_);

    action_reconstruction_reset_ =
            new QAction(QIcon(":/media/reconstruction-reset.png"),
                        tr("Reset reconstruction"), this);
    connect(action_reconstruction_reset_, &QAction::triggered, this,
            &MainWindow::OverwriteReconstruction);

    action_render_reset_view_ = new QAction(
            QIcon(":/media/render-reset-view.png"), tr("Reset view"), this);
    connect(action_render_reset_view_, &QAction::triggered, opengl_window_,
            &OpenGLWindow::ResetView);

    connect(model_manager_widget_, static_cast<void (QComboBox::*)(int)>(
                    &QComboBox::currentIndexChanged),
            this, &MainWindow::SelectModelIdx);

    action_surface_reconstruct_ =
            new QAction(QIcon(":/media/model-stats.png"), tr("Reconstruct surface model"), this);
    action_densify_ =
            new QAction(QIcon(":/media/undistort.png"), tr("Densify model"), this);
    connect(action_densify_, &QAction::triggered, this,
            &MainWindow::DensifyModel);
    connect(action_surface_reconstruct_, &QAction::triggered, this, &MainWindow::SurfaceReconstructModel);
    blocking_actions_.push_back(action_densify_);
    blocking_actions_.push_back(action_surface_reconstruct_);

    action_render_ = new QAction(tr("Render"), this);
    connect(action_render_, &QAction::triggered, this, &MainWindow::Render,
            Qt::BlockingQueuedConnection);

    action_render_now_ = new QAction(tr("Render now"), this);
    connect(action_render_now_, &QAction::triggered, this, &MainWindow::RenderNow, Qt::BlockingQueuedConnection);

    action_reconstruction_finish_ = new QAction(tr("Finish reconstruction"), this);
    connect(action_reconstruction_finish_, &QAction::triggered, this,
            &MainWindow::ReconstructionFinish, Qt::BlockingQueuedConnection);
}

void MainWindow::CreateToolbar() {
    project_toolbar_ = addToolBar(tr("Project"));
    project_toolbar_->addAction(action_new_project_);
    project_toolbar_->addAction(action_open_project_);
    project_toolbar_->addSeparator();
    project_toolbar_->addAction(action_open_recent_project_);
    project_toolbar_->setIconSize(QSize(16, 16));

    import_export_toolbar_ = addToolBar(tr("Model import/export"));
    import_export_toolbar_->addAction(action_import_model_);
    import_export_toolbar_->addAction(action_export_model_);
    import_export_toolbar_->setIconSize(QSize(16, 16));

    preprocessing_toolbar_ = addToolBar(tr("Processing"));
    preprocessing_toolbar_->addAction(action_feature_extraction_);
    preprocessing_toolbar_->addAction(action_feature_matching_);
    preprocessing_toolbar_->addAction(action_database_management_);
    preprocessing_toolbar_->setIconSize(QSize(16, 16));

    reconstruction_toolbar_ = addToolBar(tr("Reconstruction"));
    reconstruction_toolbar_->addAction(action_reconstruction_start_);
    reconstruction_toolbar_->addAction(action_reconstruction_pause_);
    reconstruction_toolbar_->addAction(action_densify_);
    reconstruction_toolbar_->addAction(action_surface_reconstruct_);
    reconstruction_toolbar_->setIconSize(QSize(16, 16));

    render_toolbar_ = addToolBar(tr("Render"));
    render_toolbar_->addAction(action_render_reset_view_);
    render_toolbar_->addWidget(model_manager_widget_);
    render_toolbar_->setIconSize(QSize(16, 16));
}

void MainWindow::CreateStatusbar() {
    QFont font;
    font.setPointSize(12);

    project_info_label_ = new QLabel("Project not opened", this);
    project_info_label_->setFont(font);
    project_info_label_->setAlignment(Qt::AlignCenter);
    statusBar()->addWidget(project_info_label_, 1);

    statusbar_timer_label_ = new QLabel("Time 00:00:00:00", this);
    statusbar_timer_label_->setFont(font);
    statusbar_timer_label_->setAlignment(Qt::AlignCenter);
    statusBar()->addWidget(statusbar_timer_label_, 1);
    statusbar_timer_ = new QTimer(this);
    connect(statusbar_timer_, &QTimer::timeout, this, &MainWindow::UpdateTimer);
    statusbar_timer_->start(1000);

    opengl_window_->statusbar_status_label =
            new QLabel("0 Images - 0 Points", this);
    opengl_window_->statusbar_status_label->setFont(font);
    opengl_window_->statusbar_status_label->setAlignment(Qt::AlignCenter);
    statusBar()->addWidget(opengl_window_->statusbar_status_label, 1);
}

void MainWindow::CreateControllers() {
    if (mapper_controller) {
        mapper_controller->Stop();
        mapper_controller->wait();
    }

    mapper_controller.reset(new IncrementalMapperController(options_));
    mapper_controller->action_render = action_render_;
    mapper_controller->action_render_now = action_render_now_;
    mapper_controller->action_finish = action_reconstruction_finish_;
}

void MainWindow::CreateFutures() {
    import_model_watcher_ = new QFutureWatcher<void>(this);
    connect(import_model_watcher_, &QFutureWatcher<void>::finished, this,
            &MainWindow::ImportModelFinished);

    export_model_watcher_ = new QFutureWatcher<void>(this);
    connect(export_model_watcher_, &QFutureWatcher<void>::finished, this,
            &MainWindow::ExportModelFinished);
}

void MainWindow::CreateProgressBar() {
    progress_bar_ = new QProgressDialog(this);
    progress_bar_->setWindowModality(Qt::ApplicationModal);
    progress_bar_->setWindowFlags(Qt::Popup);
    progress_bar_->setCancelButton(nullptr);
    progress_bar_->setMaximum(0);
    progress_bar_->setMinimum(0);
    progress_bar_->setValue(0);
    progress_bar_->hide();
    progress_bar_->close();
}

void MainWindow::CenterProgressBar() {
    const QPoint global = mapToGlobal(rect().center());
    progress_bar_->move(global.x() - progress_bar_->width() / 2,
                        global.y() - progress_bar_->height() / 2);
}

void MainWindow::NewProject() {
    new_project_widget_->show();
    new_project_widget_->raise();
}

void MainWindow::ImportModel() {
    if (!OverwriteReconstruction()) {
        return;
    }

    QString path = QFileDialog::getOpenFileName(this, tr("Select source..."), "");

    if (path.isEmpty()) {
        return;
    }

    if (!QDir(path).exists()) {
        QMessageBox::critical(this, "", tr("Invalid file"));
        return;
    }

    if (!HasFileExtension(path.toStdString(), ".ply")) {
        QMessageBox::critical(this, "", tr("Invalid file format (supported formats: PLY)"));
        return;
    }

    progress_bar_->setLabelText(tr("Importing model"));
    progress_bar_->raise();
    progress_bar_->show();

    import_model_watcher_->setFuture(QtConcurrent::run([this, path]() {
        const size_t model_idx = this->mapper_controller->AddModel();
        this->mapper_controller->Model(model_idx).ImportPLY(path.toStdString(), false);
        this->options_.render_options->min_track_len = 0;
        model_manager_widget_->UpdateModels(mapper_controller->Models());
        model_manager_widget_->SetModelIdx(model_idx);
    }));
}

void MainWindow::ImportModelFinished() {
    RenderSelectedModel();
    progress_bar_->hide();
}

void MainWindow::ExportModel() {
    if (!IsSelectedModelValid()) {
        return;
    }

    QString default_filter("PLY (*.ply)");
    const std::string path = QFileDialog::getSaveFileName(
            this,
            tr("Select project file"),
            "",
            "PLY (*.ply)",
            &default_filter).toUtf8().constData();

    if (path == "") { return; }

    progress_bar_->setLabelText(tr("Exporting model"));
    progress_bar_->raise();
    progress_bar_->show();

    export_model_watcher_->setFuture(QtConcurrent::run([this, path]() {
        const Reconstruction& model = mapper_controller->Model(SelectedModelIdx());
        try {
            model.ExportPLY(path);
        } catch (std::domain_error& error) {
            std::cerr << "ERROR: " << error.what() << std::endl;
        }
    }));
}

void MainWindow::ExportModelFinished() { progress_bar_->hide(); }

void MainWindow::OpenProject() {
    QString path;

    while (true) {
        path = QFileDialog::getExistingDirectory(
                this,
                tr("Select a directory with project"),
                "",
                QFileDialog::ShowDirsOnly
        );

        if (path.isEmpty()) { return; }

        if (IsValidProjectDirectory(path)) {
            break;
        } else {
            QMessageBox::critical(this, "", tr("You must choose a directory with a project!"));
        }
    }

    OpenProjectPath(path);
}

void MainWindow::OpenRecentProject() {
    OpenProjectPath(ReadAppConfig());
}

void MainWindow::OpenProjectPath(const QString& path) {
    progress_bar_->setLabelText(tr("Opening project"));
    progress_bar_->raise();
    progress_bar_->show();

    *(this->options_.project_path) = path.toStdString();
    *(this->options_.database_path) = (path + "/base.db").toStdString();

    ReadProjectConfiguration(path);

    UpdateProjectInfoStatusBar();

    WriteAppConfig();

    progress_bar_->hide();
}

bool MainWindow::IsValidProjectDirectory(const QString& path) {
    return QFileInfo(EnsureTrailingSlash(path) + "base.db").exists() &&
           QFileInfo(EnsureTrailingSlash(path) + "config").exists();
}

void MainWindow::ReadProjectConfiguration(const QString& project_path) {
    std::ifstream file((project_path + "/config").toStdString());

    std::string project_name_line;
    std::getline(file, project_name_line);

    std::string images_line;
    std::getline(file, images_line);

    *(this->options_.project_name) = project_name_line.substr(
            std::string("project_name:").length(),
            project_name_line.length() - std::string("project_name:").length());
    *(this->options_.image_path) = images_line.substr(
            std::string("image_path:").length(),
            images_line.length() - std::string("image_path:").length());

    file.close();
}

void MainWindow::WriteProjectConfiguration() {
    std::ofstream file;
    std::string filename = *options_.project_path + "config";
    file.open(filename.c_str(), std::ios::trunc);

    file << "project_name:" << *options_.project_name << std::endl;
    file << "image_path:" << *options_.image_path << std::endl;

    file.close();
}

QString MainWindow::ReadAppConfig() {
    QString config_file_name = EnsureTrailingSlash(QDir::homePath()) + MainWindow::APP_CONFIG_FILENAME;

    if (!QFileInfo(config_file_name).exists()) {
        QMessageBox::critical(this, "", tr("Reading from config file failed."));
        return QString("");
    }

    std::ifstream config_file(config_file_name.toStdString());
    std::string recent_project_path;
    std::getline(config_file, recent_project_path);
    config_file.close();

    return QString::fromStdString(recent_project_path);
}

void MainWindow::WriteAppConfig() {
    std::ofstream config_file;
    std::string filename = (EnsureTrailingSlash(QDir::homePath()) + MainWindow::APP_CONFIG_FILENAME).toStdString();
    config_file.open(filename, std::ios::trunc);
    config_file << EnsureTrailingSlash(*options_.project_path) << std::endl;
    config_file.close();

    action_open_recent_project_->setToolTip("RELOAD THIS PROJECT");
}

void MainWindow::FeatureExtraction() {
    if (options_.Check()) {
//        FeatureExtractor* feature_extractor = new SiftGPUFeatureExtractor(
//                options_.extraction_options->Options(),
//                options_.extraction_options->sift_options,
//                *options_.database_path,
//                *options_.image_path);

        FeatureExtractor* feature_extractor = new OpenCVFeatureExtractor(
                "SIFT", "SIFT",
                options_.extraction_options->Options(),
                options_.extraction_options->sift_options,
                *options_.database_path,
                *options_.image_path);

        feature_extractor->start();
        QProgressDialog* progress_bar_ = new QProgressDialog(this);
        progress_bar_->setWindowModality(Qt::ApplicationModal);
        progress_bar_->setLabel(new QLabel(tr("Extracting..."), this));
        progress_bar_->setMaximum(0);
        progress_bar_->setMinimum(0);
        progress_bar_->setValue(0);
        progress_bar_->hide();
        progress_bar_->close();
        connect(feature_extractor, &QThread::finished, progress_bar_,
                [progress_bar_, feature_extractor]() {
                    progress_bar_->hide();
                    feature_extractor->deleteLater();
                });
        connect(progress_bar_, &QProgressDialog::canceled, [feature_extractor]() {
            if (feature_extractor->isRunning()) {
                feature_extractor->Stop();
                feature_extractor->wait();
            }
        });
        progress_bar_->show();
        progress_bar_->raise();
    } else {
        ShowInvalidProjectError();
    }
}

void MainWindow::FeatureMatching() {
    if (options_.Check()) {
        ExhaustiveFeatureMatcher* feature_matcher = new ExhaustiveFeatureMatcher(
                options_.match_options->Options(),
                options_.exhaustive_match_options->Options(),
                *options_.database_path);
        feature_matcher->start();
        QProgressDialog* progress_bar_ = new QProgressDialog(this);
        progress_bar_->setWindowModality(Qt::ApplicationModal);
        progress_bar_->setLabel(new QLabel(tr("Matching..."), this));
        progress_bar_->setMaximum(0);
        progress_bar_->setMinimum(0);
        progress_bar_->setValue(0);
        progress_bar_->hide();
        progress_bar_->close();
        connect(feature_matcher, &QThread::finished, progress_bar_,
                [progress_bar_, feature_matcher]() {
                    progress_bar_->hide();
                    feature_matcher->deleteLater();
                });

        connect(progress_bar_, &QProgressDialog::canceled, [feature_matcher]() {
            feature_matcher->Stop();
            feature_matcher->wait();
        });
        progress_bar_->show();
        progress_bar_->raise();
    } else {
        ShowInvalidProjectError();
    }
}

void MainWindow::DatabaseManagement() {
    if (options_.Check()) {
        database_management_widget_->show();
        database_management_widget_->raise();
    } else {
        ShowInvalidProjectError();
    }
}

void MainWindow::ReconstructionStart() {
    if (!mapper_controller->IsStarted() && !options_.Check()) {
        ShowInvalidProjectError();
        return;
    }

    if (mapper_controller->IsFinished() && HasSelectedModel()) {
        QMessageBox::critical(this, "", tr("Reset reconstruction before starting."));
        return;
    }

    if (mapper_controller->IsStarted()) {
        timer_.Resume();
        mapper_controller->Resume();
    } else {
        timer_.Restart();
        mapper_controller->start();
    }

    DisableBlockingActions();
    action_reconstruction_pause_->setEnabled(true);
}

void MainWindow::ReconstructionPause() {
    timer_.Pause();
    mapper_controller->Pause();
    EnableBlockingActions();
    action_reconstruction_pause_->setEnabled(false);
}

void MainWindow::ReconstructionFinish() {
    timer_.Pause();
    mapper_controller->Stop();
    EnableBlockingActions();
    action_reconstruction_pause_->setEnabled(false);
}

void MainWindow::ReconstructionReset() {
    timer_.Reset();
    UpdateTimer();
    CreateControllers();
    EnableBlockingActions();
    RenderClear();
}

void MainWindow::Render() {
    if (mapper_controller->NumModels() == 0) {
        return;
    }

    const Reconstruction& model = mapper_controller->Model(SelectedModelIdx());

    int refresh_rate;
    if (options_.render_options->adapt_refresh_rate) {
        refresh_rate = static_cast<int>(model.NumRegImages() / 50 + 1);
    } else {
        refresh_rate = options_.render_options->refresh_rate;
    }

    if (render_counter_ % refresh_rate != 0) {
        render_counter_ += 1;
        return;
    }

    render_counter_ += 1;

    RenderNow();
}

void MainWindow::RenderNow() {
    model_manager_widget_->UpdateModels(mapper_controller->Models());
    RenderSelectedModel();
}

void MainWindow::RenderSelectedModel() {
    if (mapper_controller->NumModels() == 0) {
        RenderClear();
        return;
    }

    const size_t model_idx = SelectedModelIdx();
    if (mapper_controller->Model(model_idx).NumImages() > 0) {
        this->options_.render_options->min_track_len = 3;
    }
    else {
        this->options_.render_options->min_track_len = 0;
    }
    opengl_window_->reconstruction = &mapper_controller->Model(model_idx);
    opengl_window_->Update();
}

void MainWindow::RenderClear() {
    model_manager_widget_->SetModelIdx(ModelManagerWidget::kNewestModelIdx);
    opengl_window_->Clear();
}

void MainWindow::SelectModelIdx(const size_t) { RenderSelectedModel(); }

size_t MainWindow::SelectedModelIdx() {
    size_t model_idx = model_manager_widget_->ModelIdx();
    if (model_idx == ModelManagerWidget::kNewestModelIdx) {
        if (mapper_controller->NumModels() > 0) {
            model_idx = mapper_controller->NumModels() - 1;
        }
    }
    return model_idx;
}

bool MainWindow::HasSelectedModel() {
    const size_t model_idx = model_manager_widget_->ModelIdx();
    if (model_idx == ModelManagerWidget::kNewestModelIdx) {
        if (mapper_controller->NumModels() == 0) {
            return false;
        }
    }
    return true;
}

bool MainWindow::IsSelectedModelValid() {
    if (!HasSelectedModel()) {
        QMessageBox::critical(this, "", tr("No model selected."));
        return false;
    }
    return true;
}

void MainWindow::DensifyModel() {
    if (mapper_controller->NumModels() == 0) {
        QMessageBox::critical(this, "", tr("There is no model available for densify yet."));
        return;
    }

    QString output_path = QString::fromStdString(*options_.project_path);
    QDir(output_path).cdUp();
    EnsureTrailingSlash(output_path);

    ImageDensifier* densifier = new ImageDensifier(
            mapper_controller->Model(0), *options_.image_path, output_path.toStdString(), binary_path_
    );
    densifier->start();
    QProgressDialog* progress_bar_ = new QProgressDialog(this);
    progress_bar_->setWindowModality(Qt::ApplicationModal);
    progress_bar_->setLabel(new QLabel(tr("Densifying..."), this));
    progress_bar_->setMaximum(0);
    progress_bar_->setMinimum(0);
    progress_bar_->setValue(0);
    progress_bar_->hide();
    progress_bar_->close();
    connect(densifier, &QThread::finished, progress_bar_,
            [this, progress_bar_, densifier, output_path]() {
                if (densifier->IsSuccessfull()) {
                    const size_t model_idx = this->mapper_controller->AddModel();
                    for (auto option: densifier->ResultFiles()) {
                        const std::string path = output_path.toStdString() + "pmvs/models/" + option + ".ply";
                        this->mapper_controller->Model(model_idx).ImportPLY(path, false);
                    }
                    model_manager_widget_->UpdateModels(mapper_controller->Models());
                    model_manager_widget_->SetModelIdx(model_idx);
                }
                else {
                    QMessageBox::critical(this, "", tr("Densifying failed."));
                }
                progress_bar_->hide();
                densifier->deleteLater();
            });
    connect(progress_bar_, &QProgressDialog::canceled, [densifier]() {
        if (densifier->isRunning()) {
            densifier->Stop();
            densifier->wait();
        }
    });
    progress_bar_->show();
    progress_bar_->raise();
}

void MainWindow::UpdateTimer() {
    const int elapsed_time = static_cast<int>(timer_.ElapsedSeconds());
    const int seconds = elapsed_time % 60;
    const int minutes = (elapsed_time / 60) % 60;
    const int hours = (elapsed_time / 3600) % 24;
    const int days = elapsed_time / 86400;
    statusbar_timer_label_->setText(QString().sprintf(
            "Time %02d:%02d:%02d:%02d", days, hours, minutes, seconds));
}

void MainWindow::ShowInvalidProjectError() {
    QMessageBox::critical(this, "",
                          tr("You must create or open a valid project."));
}

void MainWindow::EnableBlockingActions() {
    for (auto& action : blocking_actions_) {
        action->setEnabled(true);
    }
}

void MainWindow::DisableBlockingActions() {
    for (auto& action : blocking_actions_) {
        action->setDisabled(true);
    }
}

void MainWindow::UpdateWindowTitle() {
    if (*options_.project_path == "") {
        setWindowTitle(QString::fromStdString("3D reconstruction"));
    } else {
        std::string project_title = *options_.project_path;
        if (project_title.size() > 80) {
            project_title =
                    "..." + project_title.substr(project_title.size() - 77, 77);
        }
        setWindowTitle(QString::fromStdString("3D reconstruction - " + project_title));
    }
}

void MainWindow::UpdateProjectInfoStatusBar() {
    if (*options_.project_name == "") {
        return;
    }

    project_info_label_->setText(
            QString::fromStdString(*options_.project_name) +
            " | " +
            QString::fromStdString(*options_.project_path));
}

void MainWindow::SurfaceReconstructModel() {
    QString output_path = QString::fromStdString(*options_.database_path);
    QDir(output_path).cdUp();
    EnsureTrailingSlash(output_path);
    output_path += "pmvs/models/";

    SurfaceReconstructer* reconstructer = new SurfaceReconstructer(output_path.toStdString(), binary_path_);
    reconstructer->start();
    QProgressDialog* progress_bar_ = new QProgressDialog(this);
    progress_bar_->setWindowModality(Qt::ApplicationModal);
    progress_bar_->setLabel(new QLabel(tr("Reconstructing surface..."), this));
    progress_bar_->setMaximum(0);
    progress_bar_->setMinimum(0);
    progress_bar_->setValue(0);
    progress_bar_->hide();
    progress_bar_->close();
    connect(reconstructer, &QThread::finished, progress_bar_,
            [this, progress_bar_, reconstructer, output_path]() {
                if (reconstructer->IsSuccessfull()) {
                    const size_t model_idx = this->mapper_controller->AddModel();
                    const std::string path = output_path.toStdString() + "output.ply";
                    this->mapper_controller->Model(model_idx).ImportPLY(path, false);
                    model_manager_widget_->UpdateModels(mapper_controller->Models());
                    model_manager_widget_->SetModelIdx(model_idx);
                }
                else {
                    QMessageBox::critical(this, "", tr("Surface reconstruct failed."));
                }
                progress_bar_->hide();
                reconstructer->deleteLater();
            });
    connect(progress_bar_, &QProgressDialog::canceled, [reconstructer]() {
        if (reconstructer->isRunning()) {
            reconstructer->Stop();
            reconstructer->wait();
        }
    });
    progress_bar_->show();
    progress_bar_->raise();
}