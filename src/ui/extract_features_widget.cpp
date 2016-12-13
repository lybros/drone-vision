#include "extract_features_widget.h"

ExtractFeaturesWidget::ExtractFeaturesWidget(MainWindow* parent) : main_window_(parent) {
    setWindowFlags(Qt::Dialog);
    setWindowModality(Qt::ApplicationModal);
    setWindowTitle("Select extractor feature");

    QPushButton* create_button = new QPushButton(tr("Start"), this);
    //connect(create_button, &QPushButton::released, this, &ExtractFeaturesWidget::Create);

    QGridLayout* grid = new QGridLayout(this);
    grid->addWidget(create_button, 3, 3);
}