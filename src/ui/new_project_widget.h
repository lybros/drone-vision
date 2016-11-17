#ifndef INC_3D_RECONSTRUCTION_NEW_PROJECT_WIDGET_H
#define INC_3D_RECONSTRUCTION_NEW_PROJECT_WIDGET_H

#include "main_window.h"

#include <QtWidgets>

class MainWindow;

class NewProjectWidget : public QWidget {
public:
    NewProjectWidget(MainWindow* parent, OptionManager* options);

    bool PathsValid();

    std::string ProjectName() const;
    std::string ProjectParentPath() const;
    std::string ProjectPath() const;
    std::string ImagePath() const;

    void SetProjectPath(const std::string& path);
    void SetImagePath(const std::string& path);

private:
    void Create();

    void SelectProjectPath();
    void SelectImagePath();

    QString DefaultDirectory();

    MainWindow* main_window_;

    OptionManager* options_;

    bool prev_selected_;

    QLineEdit* image_path_text_;
    QLineEdit* project_path_text_;
    QLineEdit* project_name_text_;
};

#endif //INC_3D_RECONSTRUCTION_NEW_PROJECT_WIDGET_H
