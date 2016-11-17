#ifndef INC_3D_RECONSTRUCTION_DATABASE_MANAGEMENT_WIDGET_H
#define INC_3D_RECONSTRUCTION_DATABASE_MANAGEMENT_WIDGET_H

#include "../opengl_window.h"
#include "matches_widget.h"
#include "image_viewer_widget.h"

#include <QtWidgets>

class MatchesWidget;
class BasicImageViewerWidget;

class ImageTab : public QWidget {
public:
    ImageTab(QWidget* parent, OptionManager* options, Database* database);

    void Update();

    void Save();

    void Clear();

private:
    void itemChanged(QTableWidgetItem* item);

    void ShowImage();

    void ShowMatches();

    void SetCamera();

    OptionManager* options_;
    Database* database_;

    std::vector<Image> images_;

    QTableWidget* table_widget_;
    QLabel* info_label_;

    MatchesWidget* matches_widget_;

    BasicImageViewerWidget* image_viewer_;
};

class CameraTab : public QWidget {
public:
    CameraTab(QWidget* parent, Database* database);

    void Update();

    void Save();

    void Clear();

private:
    void itemChanged(QTableWidgetItem* item);

    void Add();

    Database* database_;

    std::vector<Camera> cameras_;

    QTableWidget* table_widget_;
    QLabel* info_label_;
};

class DatabaseManagementWidget : public QWidget {
public:
    DatabaseManagementWidget(QWidget* parent, OptionManager* options);

private:
    void showEvent(QShowEvent* event);

    void hideEvent(QHideEvent* event);

    void Save();

    QWidget* parent_;

    OptionManager* options_;
    Database database_;

    QTabWidget* tab_widget_;
    ImageTab* image_tab_;
    CameraTab* camera_tab_;
};

#endif //INC_3D_RECONSTRUCTION_DATABASE_MANAGEMENT_WIDGET_H
