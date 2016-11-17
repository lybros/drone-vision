#ifndef INC_3D_RECONSTRUCTION_BASIC_IMAGE_VIEWER_H
#define INC_3D_RECONSTRUCTION_BASIC_IMAGE_VIEWER_H

#include "database_management_widget.h"

#include <QtWidgets>

class OpenGLWindow;

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

#endif //INC_3D_RECONSTRUCTION_BASIC_IMAGE_VIEWER_H
