#ifndef INC_3D_RECONSTRUCTION_POINT_VIEWER_H
#define INC_3D_RECONSTRUCTION_POINT_VIEWER_H

#include "database_management_widget.h"

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

#endif //INC_3D_RECONSTRUCTION_POINT_VIEWER_H
