#ifndef INC_3D_RECONSTRUCTION_DATABASE_MANAGEMENT_WIDGET_H
#define INC_3D_RECONSTRUCTION_DATABASE_MANAGEMENT_WIDGET_H

#include "opengl_window.h"

#include <QtWidgets>

class OpenGLWindow;

class PointViewerWidget : public QWidget {
public:
    PointViewerWidget(QWidget* parent, OpenGLWindow* opengl_window,
                      OptionManager* option);

    void Show(const point3D_t point3D_id);

private:
    void closeEvent(QCloseEvent* event);

    void ClearLocations();

    void UpdateImages();

    void ZoomIn();

    void ZoomOut();

    void Delete();

    OpenGLWindow* opengl_window_;

    OptionManager* options_;

    QPushButton* delete_button_;

    point3D_t point3D_id_;

    QTableWidget* location_table_;
    std::vector<QPixmap> location_pixmaps_;
    std::vector<QLabel*> location_labels_;
    std::vector<double> image_ids_;
    std::vector<double> reproj_errors_;

    QPushButton* zoom_in_button_;
    QPushButton* zoom_out_button_;

    double zoom_;
};


class BasicImageViewerWidget : public QWidget {
public:
    BasicImageViewerWidget(QWidget* parent, const std::string& switch_text);

    void Show(const std::string& path, const FeatureKeypoints& keypoints,
              const std::vector<bool>& tri_mask);

protected:
    void closeEvent(QCloseEvent* event);

    void UpdateImage();

    void ZoomIn();

    void ZoomOut();

    void ShowOrHide();

    OpenGLWindow* opengl_window_;

    QGridLayout* grid_;
    QHBoxLayout* button_layout_;

    QPixmap image1_;
    QPixmap image2_;

    QPushButton* show_button_;
    QPushButton* zoom_in_button_;
    QPushButton* zoom_out_button_;
    QScrollArea* image_scroll_area_;
    QLabel* image_label_;

    int orig_width_;
    double zoom_;
    bool switch_;
    const std::string switch_text_;
};

class MatchesImageViewerWidget : public BasicImageViewerWidget {
public:
    MatchesImageViewerWidget(QWidget* parent);

    void Show(const std::string& path1, const std::string& path2,
              const FeatureKeypoints& keypoints1,
              const FeatureKeypoints& keypoints2, const FeatureMatches& matches);
};

class ImageViewerWidget : public BasicImageViewerWidget {
public:
    ImageViewerWidget(QWidget* parent, OpenGLWindow* opengl_window,
                      OptionManager* options);

    void Show(const image_t image_id);

private:
    void Resize();

    void Delete();

    OpenGLWindow* opengl_window_;

    OptionManager* options_;

    QPushButton* delete_button_;

    image_t image_id_;

    QTableWidget* table_widget_;
    QTableWidgetItem* image_id_item_;
    QTableWidgetItem* camera_id_item_;
    QTableWidgetItem* camera_model_item_;
    QTableWidgetItem* camera_params_item_;
    QTableWidgetItem* qvec_item_;
    QTableWidgetItem* tvec_item_;
    QTableWidgetItem* dimensions_item_;
    QTableWidgetItem* num_points2D_item_;
    QTableWidgetItem* num_points3D_item_;
    QTableWidgetItem* num_obs_item_;
    QTableWidgetItem* name_item_;
};

class MatchesTab : public QWidget {
public:
    MatchesTab() { }

    MatchesTab(QWidget* parent, OptionManager* options, Database* database);

    void Clear();

protected:
    void InitializeTable(const QStringList& table_header);

    void ShowMatches();

    void FillTable();

    OptionManager* options_;
    Database* database_;

    const Image* image_;
    std::vector<std::pair<const Image*, FeatureMatches>> matches_;
    std::vector<int> configs_;
    std::vector<size_t> sorted_matches_idxs_;

    QTableWidget* table_widget_;
    QLabel* info_label_;
    MatchesImageViewerWidget* matches_viewer_;
};

class RawMatchesTab : public MatchesTab {
public:
    RawMatchesTab(QWidget* parent, OptionManager* options, Database* database);

    void Update(const std::vector<Image>& images, const image_t image_id);
};

class InlierMatchesTab : public MatchesTab {
public:
    InlierMatchesTab(QWidget* parent, OptionManager* options, Database* database);

    void Update(const std::vector<Image>& images, const image_t image_id);
};

class MatchesWidget : public QWidget {
public:
    MatchesWidget(QWidget* parent, OptionManager* options, Database* database);

    void ShowMatches(const std::vector<Image>& images, const image_t image_id);

private:
    void closeEvent(QCloseEvent* event);

    QWidget* parent_;

    OptionManager* options_;

    QTabWidget* tab_widget_;
    RawMatchesTab* raw_matches_tab_;
    InlierMatchesTab* inlier_matches_tab_;
};

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
