#include "new_project_widget.h"

NewProjectWidget::NewProjectWidget(MainWindow* parent, OptionManager* options) : main_window_(parent),
                                                                                 options_(options),
                                                                                 prev_selected_(false) {
    setWindowFlags(Qt::Dialog);
    setWindowModality(Qt::ApplicationModal);
    setWindowTitle("New project");

    project_name_text_ = new QLineEdit(this);
    project_name_text_->setText(QString::fromStdString(*options_->project_name));

    QPushButton* project_path_select = new QPushButton(tr("Select"), this);
    connect(project_path_select, &QPushButton::released, this, &NewProjectWidget::SelectProjectPath);
    project_path_text_ = new QLineEdit(this);
    project_path_text_->setText(QString::fromStdString(*options_->project_path));

    QPushButton* image_path_select = new QPushButton(tr("Select"), this);
    connect(image_path_select, &QPushButton::released, this, &NewProjectWidget::SelectImagePath);
    image_path_text_ = new QLineEdit(this);
    image_path_text_->setText(QString::fromStdString(*options_->image_path));

    QPushButton* create_button = new QPushButton(tr("Create"), this);
    connect(create_button, &QPushButton::released, this, &NewProjectWidget::Create);

    QGridLayout* grid = new QGridLayout(this);

    grid->addWidget(new QLabel(tr("Project name"), this), 0, 0);
    grid->addWidget(project_name_text_, 0, 1);

    grid->addWidget(new QLabel(tr("Project folder"), this), 1, 0);
    grid->addWidget(project_path_text_, 1, 1);
    grid->addWidget(project_path_select, 1, 3);

    grid->addWidget(new QLabel(tr("Images folder"), this), 2, 0);
    grid->addWidget(image_path_text_, 2, 1);
    grid->addWidget(image_path_select, 2, 3);

    grid->addWidget(create_button, 3, 3);
}

bool NewProjectWidget::PathsValid() {
    return QDir(ImagePath()).exists() &&
           QDir(ProjectParentPath()).exists() &&
           QFileInfo(ProjectParentPath()).isWritable();
}

QString NewProjectWidget::ProjectName() const {
    return project_name_text_->text();
}

QString NewProjectWidget::ProjectParentPath() const {
    return EnsureTrailingSlash(project_path_text_->text());
}

QString NewProjectWidget::ProjectPath() const {
    return ProjectParentPath() + EnsureTrailingSlash(ProjectName());
}

QString NewProjectWidget::ImagePath() const {
    return EnsureTrailingSlash(image_path_text_->text());
}

void NewProjectWidget::SetProjectPath(const std::string& path) {
    project_path_text_->setText(QString::fromStdString(path));
}

void NewProjectWidget::SetImagePath(const std::string& path) {
    image_path_text_->setText(QString::fromStdString(path));
}

void NewProjectWidget::Create() {
    if (ProjectName() == "") {
        QMessageBox::critical(this, "", tr("You must set project name!"));
        return;
    }

    if (!PathsValid()) {
        QMessageBox::critical(this, "", tr("Invalid paths."));
        return;
    }

    if (QDir(ProjectPath()).exists()) {
        QMessageBox::critical(this, "",
                              tr("Project name corresponds with existent directory.\nPlease choose another project name / project directory"));
        return;
    }

    QDir(ProjectParentPath()).mkdir(ProjectName());

    if (main_window_->mapper_controller->NumModels() > 0) {
        if (!main_window_->OverwriteReconstruction()) {
            return;
        }
    }

    *options_->project_name = ProjectName().toStdString();
    *options_->project_path = ProjectPath().toStdString();
    *options_->database_path = (ProjectPath() + QString("base.db")).toStdString();
    *options_->image_path = ImagePath().toStdString();

    Database database;
    database.Open(*options_->database_path);

    main_window_->WriteProjectConfiguration();

    main_window_->UpdateProjectInfoStatusBar();

    main_window_->WriteAppConfig();

    hide();
}

void NewProjectWidget::SelectProjectPath() {
    project_path_text_->setText(QFileDialog::getExistingDirectory(
            this, tr("Select a directory for project..."), DefaultDirectory(),
            QFileDialog::ShowDirsOnly));
}

void NewProjectWidget::SelectImagePath() {
    image_path_text_->setText(QFileDialog::getExistingDirectory(
            this, tr("Select a directory with images..."), DefaultDirectory(),
            QFileDialog::ShowDirsOnly));
}

QString NewProjectWidget::DefaultDirectory() {
    QString directory_path = "";
    if (!prev_selected_ && !options_->project_path->empty()) {
        // Starting checking with project_path, than moving up.
        directory_path = QString::fromStdString(*options_->project_path);
        // Updating directory_path if cdUp() is successful.
        QDir(directory_path).cdUp();
    }
    prev_selected_ = true;
    return directory_path;
}
