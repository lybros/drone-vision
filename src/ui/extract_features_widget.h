#ifndef INC_3D_RECONSTRUCTION_EXTRACT_FEATURES_WIDGET_H
#define INC_3D_RECONSTRUCTION_EXTRACT_FEATURES_WIDGET_H

#include "main_window.h"

#include <QtWidgets>
#include <QString>

class MainWindow;

class ExtractFeaturesWidget : public QWidget {
public:
    ExtractFeaturesWidget(MainWindow* parent);

private:
    void Create();

    MainWindow* main_window_;
};

#endif //INC_3D_RECONSTRUCTION_EXTRACT_FEATURES_WIDGET_H
