#include "database_management_widget.h"

PointViewerWidget::PointViewerWidget(QWidget* parent,
                                     OpenGLWindow* opengl_window,
                                     OptionManager* options)
        : QWidget(parent),
          opengl_window_(opengl_window),
          options_(options),
          point3D_id_(kInvalidPoint3DId),
          zoom_(250.0 / 1024.0) {
    setWindowFlags(Qt::Window);
    resize(parent->size().width() - 20, parent->size().height() - 20);

    QFont font;
    font.setPointSize(10);
    setFont(font);

    QGridLayout* grid = new QGridLayout(this);
    grid->setContentsMargins(5, 5, 5, 5);

    location_table_ = new QTableWidget(this);
    location_table_->setColumnCount(3);
    QStringList table_header;
    table_header << "image_id"
                 << "reproj_error"
                 << "track_location";
    location_table_->setHorizontalHeaderLabels(table_header);
    location_table_->resizeColumnsToContents();
    location_table_->setShowGrid(true);
    location_table_->horizontalHeader()->setStretchLastSection(true);
    location_table_->verticalHeader()->setVisible(true);
    location_table_->setSelectionMode(QAbstractItemView::NoSelection);
    location_table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    location_table_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    location_table_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

    grid->addWidget(location_table_, 0, 0);

    QHBoxLayout* button_layout = new QHBoxLayout();

    zoom_in_button_ = new QPushButton(tr("+"), this);
    zoom_in_button_->setFont(font);
    zoom_in_button_->setFixedWidth(50);
    button_layout->addWidget(zoom_in_button_);
    connect(zoom_in_button_, &QPushButton::released, this,
            &PointViewerWidget::ZoomIn);

    zoom_out_button_ = new QPushButton(tr("-"), this);
    zoom_out_button_->setFont(font);
    zoom_out_button_->setFixedWidth(50);
    button_layout->addWidget(zoom_out_button_);
    connect(zoom_out_button_, &QPushButton::released, this,
            &PointViewerWidget::ZoomOut);

    delete_button_ = new QPushButton(tr("Delete"), this);
    button_layout->addWidget(delete_button_);
    connect(delete_button_, &QPushButton::released, this,
            &PointViewerWidget::Delete);

    grid->addLayout(button_layout, 1, 0, Qt::AlignRight);
}

void PointViewerWidget::Show(const point3D_t point3D_id) {
    location_pixmaps_.clear();
    image_ids_.clear();
    reproj_errors_.clear();

    if (opengl_window_->points3D.count(point3D_id) == 0) {
        point3D_id_ = kInvalidPoint3DId;
        ClearLocations();
        return;
    }

    point3D_id_ = point3D_id;

    setWindowTitle(QString::fromStdString("Point " + std::to_string(point3D_id)));

    const auto& point3D = opengl_window_->points3D[point3D_id];

    for (const auto& track_el : point3D.Track().Elements()) {
        const Image& image = opengl_window_->images[track_el.image_id];
        const Camera& camera = opengl_window_->cameras[image.CameraId()];
        const Point2D& point2D = image.Point2D(track_el.point2D_idx);

        const Eigen::Matrix3x4d proj_matrix = image.ProjectionMatrix();
        const double error = CalculateReprojectionError(point2D.XY(), point3D.XYZ(),
                                                        proj_matrix, camera);

        const std::string path =
                EnsureTrailingSlash(*options_->image_path) + image.Name();

        Bitmap bitmap;
        if (!bitmap.Read(path, true)) {
            std::cerr << "ERROR: Cannot read image at path " << path << std::endl;
            continue;
        }

        QPixmap pixmap = QPixmap::fromImage(BitmapToQImageRGB(bitmap));

        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing);
        QPen pen;
        pen.setWidth(3);
        pen.setColor(Qt::red);
        painter.setPen(pen);
        painter.drawEllipse(static_cast<int>(point2D.X() - 5),
                            static_cast<int>(point2D.Y() - 5), 10, 10);
        painter.drawEllipse(static_cast<int>(point2D.X() - 15),
                            static_cast<int>(point2D.Y() - 15), 30, 30);
        painter.drawEllipse(static_cast<int>(point2D.X() - 45),
                            static_cast<int>(point2D.Y() - 45), 90, 90);

        location_pixmaps_.push_back(pixmap);
        image_ids_.push_back(track_el.image_id);
        reproj_errors_.push_back(error);
    }

    UpdateImages();
}

void PointViewerWidget::closeEvent(QCloseEvent* event) {
    location_pixmaps_.clear();
    image_ids_.clear();
    reproj_errors_.clear();
    ClearLocations();
}

void PointViewerWidget::ClearLocations() {
    while (location_table_->rowCount() > 0) {
        location_table_->removeRow(0);
    }
    for (auto location_label : location_labels_) {
        delete location_label;
    }
    location_labels_.clear();
}

void PointViewerWidget::UpdateImages() {
    ClearLocations();

    location_table_->setRowCount(static_cast<int>(location_pixmaps_.size()));

    for (size_t i = 0; i < location_pixmaps_.size(); ++i) {
        QLabel* image_id_label = new QLabel(QString::number(image_ids_[i]), this);
        location_table_->setCellWidget(i, 0, image_id_label);
        location_labels_.push_back(image_id_label);

        QLabel* error_label = new QLabel(QString::number(reproj_errors_[i]), this);
        location_table_->setCellWidget(i, 1, error_label);
        location_labels_.push_back(error_label);

        const QPixmap& pixmap = location_pixmaps_[i];
        QLabel* image_label = new QLabel(this);
        image_label->setPixmap(
                pixmap.scaledToWidth(zoom_ * pixmap.width(), Qt::FastTransformation));
        location_table_->setCellWidget(i, 2, image_label);
        location_table_->resizeRowToContents(i);
        location_labels_.push_back(image_label);
    }
    location_table_->resizeColumnToContents(2);
}

void PointViewerWidget::ZoomIn() {
    zoom_ *= 1.33;
    UpdateImages();
}

void PointViewerWidget::ZoomOut() {
    zoom_ /= 1.3;
    UpdateImages();
}

void PointViewerWidget::Delete() {
    QMessageBox::StandardButton reply = QMessageBox::question(
            this, "", tr("Do you really want to delete this point?"),
            QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        if (opengl_window_->reconstruction->ExistsPoint3D(point3D_id_)) {
            opengl_window_->reconstruction->DeletePoint3D(point3D_id_);
        }
        opengl_window_->Update();
    }
}


static const double kZoomFactor = 1.33;

BasicImageViewerWidget::BasicImageViewerWidget(QWidget* parent,
                                               const std::string& switch_text)
        : QWidget(parent), zoom_(-1), switch_(true), switch_text_(switch_text) {
    setWindowFlags(Qt::Window);
    resize(parent->width() - 20, parent->height() - 20);

    QFont font;
    font.setPointSize(10);
    setFont(font);

    grid_ = new QGridLayout(this);
    grid_->setContentsMargins(5, 5, 5, 5);

    image_label_ = new QLabel(this);
    image_scroll_area_ = new QScrollArea(this);
    image_scroll_area_->setWidget(image_label_);

    grid_->addWidget(image_scroll_area_, 1, 0);

    button_layout_ = new QHBoxLayout();

    show_button_ =
            new QPushButton(tr(std::string("Hide " + switch_text_).c_str()), this);
    show_button_->setFont(font);
    button_layout_->addWidget(show_button_);
    connect(show_button_, &QPushButton::released, this,
            &BasicImageViewerWidget::ShowOrHide);

    zoom_in_button_ = new QPushButton(tr("+"), this);
    zoom_in_button_->setFont(font);
    zoom_in_button_->setFixedWidth(50);
    button_layout_->addWidget(zoom_in_button_);
    connect(zoom_in_button_, &QPushButton::released, this,
            &BasicImageViewerWidget::ZoomIn);

    zoom_out_button_ = new QPushButton(tr("-"), this);
    zoom_out_button_->setFont(font);
    zoom_out_button_->setFixedWidth(50);
    button_layout_->addWidget(zoom_out_button_);
    connect(zoom_out_button_, &QPushButton::released, this,
            &BasicImageViewerWidget::ZoomOut);

    grid_->addLayout(button_layout_, 2, 0, Qt::AlignRight);
}

void BasicImageViewerWidget::closeEvent(QCloseEvent* event) {
    image1_ = QPixmap();
    image2_ = QPixmap();
    image_label_->clear();
}

void BasicImageViewerWidget::Show(const std::string& path,
                                  const FeatureKeypoints& keypoints,
                                  const std::vector<bool>& tri_mask) {
    Bitmap bitmap;
    if (!bitmap.Read(path, true)) {
        std::cerr << "ERROR: Cannot read image at path " << path << std::endl;
        return;
    }

    image1_ = QPixmap::fromImage(BitmapToQImageRGB(bitmap));

    image2_ = image1_;

    const size_t num_tri_keypoints = std::count_if(
            tri_mask.begin(), tri_mask.end(), [](const bool tri) { return tri; });

    FeatureKeypoints keypoints_tri(num_tri_keypoints);
    FeatureKeypoints keypoints_not_tri(keypoints.size() - num_tri_keypoints);
    size_t i_tri = 0;
    size_t i_not_tri = 0;
    for (size_t i = 0; i < tri_mask.size(); ++i) {
        if (tri_mask[i]) {
            keypoints_tri[i_tri] = keypoints[i];
            i_tri += 1;
        } else {
            keypoints_not_tri[i_not_tri] = keypoints[i];
            i_not_tri += 1;
        }
    }

    DrawKeypoints(&image2_, keypoints_tri, Qt::magenta);
    DrawKeypoints(&image2_, keypoints_not_tri, Qt::red);

    orig_width_ = image1_.width();

    UpdateImage();
}

void BasicImageViewerWidget::UpdateImage() {
    if (zoom_ == -1) {
        zoom_ = (width() - 40) / static_cast<double>(orig_width_);
    }

    const Qt::TransformationMode tform_mode =
            zoom_ > 1.25 ? Qt::FastTransformation : Qt::SmoothTransformation;

    if (switch_) {
        image_label_->setPixmap(image2_.scaledToWidth(
                static_cast<int>(zoom_ * orig_width_), tform_mode));
    } else {
        image_label_->setPixmap(image1_.scaledToWidth(
                static_cast<int>(zoom_ * orig_width_), tform_mode));
    }
    image_label_->adjustSize();
}

void BasicImageViewerWidget::ZoomIn() {
    zoom_ *= kZoomFactor;
    UpdateImage();
}

void BasicImageViewerWidget::ZoomOut() {
    zoom_ /= kZoomFactor;
    UpdateImage();
}

void BasicImageViewerWidget::ShowOrHide() {
    if (switch_) {
        show_button_->setText(tr(std::string("Show " + switch_text_).c_str()));
    } else {
        show_button_->setText(tr(std::string("Hide " + switch_text_).c_str()));
    }
    switch_ = !switch_;
    UpdateImage();
}

MatchesImageViewerWidget::MatchesImageViewerWidget(QWidget* parent)
        : BasicImageViewerWidget(parent, "matches") { }

void MatchesImageViewerWidget::Show(const std::string& path1,
                                    const std::string& path2,
                                    const FeatureKeypoints& keypoints1,
                                    const FeatureKeypoints& keypoints2,
                                    const FeatureMatches& matches) {
    Bitmap bitmap1;
    Bitmap bitmap2;
    if (!bitmap1.Read(path1, true) || !bitmap2.Read(path2, true)) {
        std::cerr << "ERROR: Cannot read images at paths " << path1 << " and "
                  << path2 << std::endl;
        return;
    }

    const auto image1 = QPixmap::fromImage(BitmapToQImageRGB(bitmap1));
    const auto image2 = QPixmap::fromImage(BitmapToQImageRGB(bitmap2));

    image1_ = ShowImagesSideBySide(image1, image2);
    image2_ = DrawMatches(image1, image2, keypoints1, keypoints2, matches);

    orig_width_ = image1_.width();

    UpdateImage();
}

ImageViewerWidget::ImageViewerWidget(QWidget* parent,
                                     OpenGLWindow* opengl_window,
                                     OptionManager* options)
        : BasicImageViewerWidget(parent, "keypoints"),
          opengl_window_(opengl_window),
          options_(options) {
    setWindowTitle("Image information");

    table_widget_ = new QTableWidget(this);
    table_widget_->setColumnCount(2);
    table_widget_->setRowCount(11);

    QFont font;
    font.setPointSize(10);
    table_widget_->setFont(font);

    table_widget_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_widget_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_widget_->setShowGrid(true);

    table_widget_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_widget_->horizontalHeader()->setDisabled(true);
    table_widget_->verticalHeader()->setVisible(false);
    table_widget_->verticalHeader()->setDefaultSectionSize(18);

    QStringList table_header;
    table_header << "Property"
                 << "Value";
    table_widget_->setHorizontalHeaderLabels(table_header);

    int row = 0;

    table_widget_->setItem(row, 0, new QTableWidgetItem("image_id"));
    image_id_item_ = new QTableWidgetItem();
    table_widget_->setItem(row, 1, image_id_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("camera_id"));
    camera_id_item_ = new QTableWidgetItem();
    table_widget_->setItem(row, 1, camera_id_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("camera_model"));
    camera_model_item_ = new QTableWidgetItem();
    table_widget_->setItem(row, 1, camera_model_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("camera_params"));
    camera_params_item_ = new QTableWidgetItem();
    table_widget_->setItem(row, 1, camera_params_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("qw, qx, qy, qz"));
    qvec_item_ = new QTableWidgetItem();
    table_widget_->setItem(row, 1, qvec_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("tx, ty, ty"));
    tvec_item_ = new QTableWidgetItem();
    table_widget_->setItem(row, 1, tvec_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("dims"));
    dimensions_item_ = new QTableWidgetItem();
    table_widget_->setItem(row, 1, dimensions_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("num_points2D"));
    num_points2D_item_ = new QTableWidgetItem();
    num_points2D_item_->setForeground(Qt::red);
    table_widget_->setItem(row, 1, num_points2D_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("num_points3D"));
    num_points3D_item_ = new QTableWidgetItem();
    num_points3D_item_->setForeground(Qt::magenta);
    table_widget_->setItem(row, 1, num_points3D_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("num_observations"));
    num_obs_item_ = new QTableWidgetItem();
    table_widget_->setItem(row, 1, num_obs_item_);
    row += 1;

    table_widget_->setItem(row, 0, new QTableWidgetItem("name"));
    name_item_ = new QTableWidgetItem();
    table_widget_->setItem(row, 1, name_item_);
    row += 1;

    grid_->addWidget(table_widget_, 0, 0);

    delete_button_ = new QPushButton(tr("Delete"), this);
    delete_button_->setFont(font);
    button_layout_->addWidget(delete_button_);
    connect(delete_button_, &QPushButton::released, this,
            &ImageViewerWidget::Delete);
}

void ImageViewerWidget::Show(const image_t image_id) {
    if (opengl_window_->images.count(image_id) == 0) {
        return;
    }

    image_id_ = image_id;

    const Image& image = opengl_window_->images.at(image_id);
    const Camera& camera = opengl_window_->cameras.at(image.CameraId());

    image_id_item_->setText(QString::number(image_id));
    camera_id_item_->setText(QString::number(image.CameraId()));
    camera_model_item_->setText(QString::fromStdString(camera.ModelName()));
    camera_params_item_->setText(QString::fromStdString(camera.ParamsToString()));
    qvec_item_->setText(QString::number(image.Qvec(0)) + ", " +
                        QString::number(image.Qvec(1)) + ", " +
                        QString::number(image.Qvec(2)) + ", " +
                        QString::number(image.Qvec(3)));
    tvec_item_->setText(QString::number(image.Tvec(0)) + ", " +
                        QString::number(image.Tvec(1)) + ", " +
                        QString::number(image.Tvec(2)));
    dimensions_item_->setText(QString::number(camera.Width()) + "x" +
                              QString::number(camera.Height()));
    num_points2D_item_->setText(QString::number(image.NumPoints2D()));

    std::vector<bool> tri_mask(image.NumPoints2D());
    for (size_t i = 0; i < image.NumPoints2D(); ++i) {
        tri_mask[i] = image.Point2D(i).HasPoint3D();
    }

    num_points3D_item_->setText(QString::number(image.NumPoints3D()));
    num_obs_item_->setText(QString::number(image.NumObservations()));
    name_item_->setText(QString::fromStdString(image.Name()));

    FeatureKeypoints keypoints(image.NumPoints2D());
    for (point2D_t i = 0; i < image.NumPoints2D(); ++i) {
        keypoints[i].x = static_cast<float>(image.Point2D(i).X());
        keypoints[i].y = static_cast<float>(image.Point2D(i).Y());
    }

    const std::string path =
            EnsureTrailingSlash(*options_->image_path) + image.Name();
    BasicImageViewerWidget::Show(path, keypoints, tri_mask);

    Resize();
}

void ImageViewerWidget::Resize() {
    table_widget_->resizeColumnsToContents();
    int height = table_widget_->horizontalHeader()->height() +
                 2 * table_widget_->frameWidth();
    for (int i = 0; i < table_widget_->rowCount(); i++) {
        height += table_widget_->rowHeight(i);
    }
    table_widget_->setFixedHeight(height);
}

void ImageViewerWidget::Delete() {
    QMessageBox::StandardButton reply = QMessageBox::question(
            this, "", tr("Do you really want to delete this image?"),
            QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        if (opengl_window_->reconstruction->ExistsImage(image_id_)) {
            opengl_window_->reconstruction->DeRegisterImage(image_id_);
        }
        opengl_window_->Update();
    }
    hide();
}

MatchesTab::MatchesTab(QWidget* parent, OptionManager* options,
                       Database* database)
        : QWidget(parent),
          options_(options),
          database_(database),
          matches_viewer_(new MatchesImageViewerWidget(parent)) { }

void MatchesTab::Clear() {
    table_widget_->clearContents();
    matches_.clear();
    configs_.clear();
    sorted_matches_idxs_.clear();
}

void MatchesTab::InitializeTable(const QStringList& table_header) {
    QGridLayout* grid = new QGridLayout(this);

    info_label_ = new QLabel(this);
    grid->addWidget(info_label_, 0, 0);

    QPushButton* show_button = new QPushButton(tr("Show matches"), this);
    connect(show_button, &QPushButton::released, this, &MatchesTab::ShowMatches);
    grid->addWidget(show_button, 0, 1, Qt::AlignRight);

    table_widget_ = new QTableWidget(this);
    table_widget_->setColumnCount(table_header.size());
    table_widget_->setHorizontalHeaderLabels(table_header);

    table_widget_->setShowGrid(true);
    table_widget_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_widget_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_widget_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_widget_->horizontalHeader()->setStretchLastSection(true);
    table_widget_->verticalHeader()->setVisible(false);
    table_widget_->verticalHeader()->setDefaultSectionSize(20);

    grid->addWidget(table_widget_, 1, 0, 1, 2);
}

void MatchesTab::ShowMatches() {
    QItemSelectionModel* select = table_widget_->selectionModel();

    if (!select->hasSelection()) {
        QMessageBox::critical(this, "", tr("No image pair selected."));
        return;
    }

    if (select->selectedRows().size() > 1) {
        QMessageBox::critical(this, "", tr("Only one image pair may be selected."));
        return;
    }

    const size_t idx =
            sorted_matches_idxs_[select->selectedRows().begin()->row()];
    const auto& selection = matches_[idx];
    const std::string path1 =
            EnsureTrailingSlash(*options_->image_path) + image_->Name();
    const std::string path2 =
            EnsureTrailingSlash(*options_->image_path) + selection.first->Name();
    const auto keypoints1 = database_->ReadKeypoints(image_->ImageId());
    const auto keypoints2 = database_->ReadKeypoints(selection.first->ImageId());

    matches_viewer_->Show(path1, path2, keypoints1, keypoints2, selection.second);

    matches_viewer_->setWindowTitle(QString::fromStdString(
            "Matches for image pair " + std::to_string(image_->ImageId()) + " - " +
            std::to_string(selection.first->ImageId())));

    matches_viewer_->show();
    matches_viewer_->raise();
}

void MatchesTab::FillTable() {
    sorted_matches_idxs_.resize(matches_.size());
    std::iota(sorted_matches_idxs_.begin(), sorted_matches_idxs_.end(), 0);

    std::sort(sorted_matches_idxs_.begin(), sorted_matches_idxs_.end(),
              [&](const size_t idx1, const size_t idx2) {
                  return matches_[idx1].second.size() >
                         matches_[idx2].second.size();
              });

    QString info;
    info += QString("Matched images: ") + QString::number(matches_.size());
    info_label_->setText(info);

    table_widget_->clearContents();
    table_widget_->setRowCount(matches_.size());

    for (size_t i = 0; i < sorted_matches_idxs_.size(); ++i) {
        const size_t idx = sorted_matches_idxs_[i];

        QTableWidgetItem* image_id_item =
                new QTableWidgetItem(QString::number(matches_[idx].first->ImageId()));
        table_widget_->setItem(i, 0, image_id_item);

        QTableWidgetItem* num_matches_item =
                new QTableWidgetItem(QString::number(matches_[idx].second.size()));
        table_widget_->setItem(i, 1, num_matches_item);

        // config for inlier matches tab
        if (table_widget_->columnCount() == 3) {
            QTableWidgetItem* config_item =
                    new QTableWidgetItem(QString::number(configs_[idx]));
            table_widget_->setItem(i, 2, config_item);
        }
    }

    table_widget_->resizeColumnsToContents();
}

RawMatchesTab::RawMatchesTab(QWidget* parent, OptionManager* options,
                             Database* database)
        : MatchesTab(parent, options, database) {
    QStringList table_header;
    table_header << "image_id"
                 << "num_matches";
    InitializeTable(table_header);
}

void RawMatchesTab::Update(const std::vector<Image>& images,
                           const image_t image_id) {
    matches_.clear();

    for (const auto& image : images) {
        if (image.ImageId() == image_id) {
            image_ = &image;
            continue;
        }

        if (database_->ExistsMatches(image_id, image.ImageId())) {
            const auto matches = database_->ReadMatches(image_id, image.ImageId());

            if (matches.size() > 0) {
                matches_.emplace_back(&image, matches);
            }
        }
    }

    FillTable();
}

InlierMatchesTab::InlierMatchesTab(QWidget* parent, OptionManager* options,
                                   Database* database)
        : MatchesTab(parent, options, database) {
    QStringList table_header;
    table_header << "image_id"
                 << "num_matches"
                 << "config";
    InitializeTable(table_header);
}

void InlierMatchesTab::Update(const std::vector<Image>& images,
                              const image_t image_id) {
    matches_.clear();
    configs_.clear();

    for (const auto& image : images) {
        if (image.ImageId() == image_id) {
            image_ = &image;
            continue;
        }

        if (database_->ExistsInlierMatches(image_id, image.ImageId())) {
            const auto two_view_geometry =
                    database_->ReadInlierMatches(image_id, image.ImageId());

            if (two_view_geometry.inlier_matches.size() > 0) {
                matches_.emplace_back(&image, two_view_geometry.inlier_matches);
                configs_.push_back(two_view_geometry.config);
            }
        }
    }

    FillTable();
}

MatchesWidget::MatchesWidget(QWidget* parent, OptionManager* options,
                             Database* database)
        : parent_(parent), options_(options) {
    setWindowFlags(Qt::Window);
    resize(parent->size().width() - 20, parent->size().height() - 20);

    QGridLayout* grid = new QGridLayout(this);

    tab_widget_ = new QTabWidget(this);

    raw_matches_tab_ = new RawMatchesTab(this, options_, database);
    tab_widget_->addTab(raw_matches_tab_, tr("Raw matches"));

    inlier_matches_tab_ = new InlierMatchesTab(this, options_, database);
    tab_widget_->addTab(inlier_matches_tab_, tr("Inlier matches"));

    grid->addWidget(tab_widget_, 0, 0);

    QPushButton* close_button = new QPushButton(tr("Close"), this);
    connect(close_button, &QPushButton::released, this, &MatchesWidget::close);
    grid->addWidget(close_button, 1, 0, Qt::AlignRight);
}

void MatchesWidget::ShowMatches(const std::vector<Image>& images,
                                const image_t image_id) {
    parent_->setDisabled(true);

    setWindowTitle(
            QString::fromStdString("Matches for image " + std::to_string(image_id)));

    raw_matches_tab_->Update(images, image_id);
    inlier_matches_tab_->Update(images, image_id);
}

void MatchesWidget::closeEvent(QCloseEvent* event) {
    raw_matches_tab_->Clear();
    inlier_matches_tab_->Clear();
    parent_->setEnabled(true);
}

ImageTab::ImageTab(QWidget* parent, OptionManager* options, Database* database)
        : QWidget(parent), options_(options), database_(database) {
    QGridLayout* grid = new QGridLayout(this);

    info_label_ = new QLabel(this);
    grid->addWidget(info_label_, 0, 0);

    QPushButton* set_camera_button = new QPushButton(tr("Set camera"), this);
    connect(set_camera_button, &QPushButton::released, this,
            &ImageTab::SetCamera);
    grid->addWidget(set_camera_button, 0, 1, Qt::AlignRight);

    QPushButton* show_image_button = new QPushButton(tr("Show image"), this);
    connect(show_image_button, &QPushButton::released, this,
            &ImageTab::ShowImage);
    grid->addWidget(show_image_button, 0, 2, Qt::AlignRight);

    QPushButton* show_matches_button = new QPushButton(tr("Show matches"), this);
    connect(show_matches_button, &QPushButton::released, this,
            &ImageTab::ShowMatches);
    grid->addWidget(show_matches_button, 0, 3, Qt::AlignRight);

    table_widget_ = new QTableWidget(this);
    table_widget_->setColumnCount(10);

    QStringList table_header;
    table_header << "image_id"
                 << "name"
                 << "camera_id"
                 << "qw"
                 << "qx"
                 << "qy"
                 << "qz"
                 << "tx"
                 << "ty"
                 << "tz";
    table_widget_->setHorizontalHeaderLabels(table_header);

    table_widget_->setShowGrid(true);
    table_widget_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_widget_->horizontalHeader()->setStretchLastSection(true);
    table_widget_->verticalHeader()->setVisible(false);
    table_widget_->verticalHeader()->setDefaultSectionSize(20);

    connect(table_widget_, &QTableWidget::itemChanged, this,
            &ImageTab::itemChanged);

    grid->addWidget(table_widget_, 1, 0, 1, 4);

    grid->setColumnStretch(0, 2);

    image_viewer_ = new BasicImageViewerWidget(parent, "keypoints");
    matches_widget_ = new MatchesWidget(parent, options, database_);
}

void ImageTab::Update() {
    QString info;
    info += QString("Images: ") + QString::number(database_->NumImages());
    info += QString("\n");
    info += QString("Features: ") + QString::number(database_->NumKeypoints());
    info_label_->setText(info);

    images_ = database_->ReadAllImages();

    table_widget_->blockSignals(true);

    table_widget_->clearContents();
    table_widget_->setRowCount(images_.size());

    for (size_t i = 0; i < images_.size(); ++i) {
        const auto& image = images_[i];
        QTableWidgetItem* id_item =
                new QTableWidgetItem(QString::number(image.ImageId()));
        id_item->setFlags(Qt::ItemIsSelectable);
        table_widget_->setItem(i, 0, id_item);
        table_widget_->setItem(
                i, 1, new QTableWidgetItem(QString::fromStdString(image.Name())));
        table_widget_->setItem(
                i, 2, new QTableWidgetItem(QString::number(image.CameraId())));
        table_widget_->setItem(
                i, 3, new QTableWidgetItem(QString::number(image.QvecPrior(0))));
        table_widget_->setItem(
                i, 4, new QTableWidgetItem(QString::number(image.QvecPrior(1))));
        table_widget_->setItem(
                i, 5, new QTableWidgetItem(QString::number(image.QvecPrior(2))));
        table_widget_->setItem(
                i, 6, new QTableWidgetItem(QString::number(image.QvecPrior(2))));
        table_widget_->setItem(
                i, 7, new QTableWidgetItem(QString::number(image.TvecPrior(0))));
        table_widget_->setItem(
                i, 8, new QTableWidgetItem(QString::number(image.TvecPrior(1))));
        table_widget_->setItem(
                i, 9, new QTableWidgetItem(QString::number(image.TvecPrior(2))));
    }
    table_widget_->resizeColumnsToContents();

    table_widget_->blockSignals(false);
}

void ImageTab::Save() {
    database_->BeginTransaction();
    for (const auto& image : images_) {
        database_->UpdateImage(image);
    }
    database_->EndTransaction();
}

void ImageTab::Clear() {
    images_.clear();
    table_widget_->clearContents();
}

void ImageTab::itemChanged(QTableWidgetItem* item) {
    camera_t camera_id = kInvalidCameraId;

    switch (item->column()) {
        case 1:
            images_[item->row()].SetName(item->text().toUtf8().constData());
            break;
        case 2:
            camera_id = static_cast<camera_t>(item->data(Qt::DisplayRole).toInt());
            if (!database_->ExistsCamera(camera_id)) {
                QMessageBox::critical(this, "", tr("camera_id does not exist."));
                table_widget_->blockSignals(true);
                item->setText(QString::number(images_[item->row()].CameraId()));
                table_widget_->blockSignals(false);
            } else {
                images_[item->row()].SetCameraId(camera_id);
            }
            break;
        case 3:
            images_[item->row()].QvecPrior(0) = item->data(Qt::DisplayRole).toReal();
            break;
        case 4:
            images_[item->row()].QvecPrior(1) = item->data(Qt::DisplayRole).toReal();
            break;
        case 5:
            images_[item->row()].QvecPrior(2) = item->data(Qt::DisplayRole).toReal();
            break;
        case 6:
            images_[item->row()].QvecPrior(3) = item->data(Qt::DisplayRole).toReal();
            break;
        case 7:
            images_[item->row()].TvecPrior(0) = item->data(Qt::DisplayRole).toReal();
            break;
        case 8:
            images_[item->row()].TvecPrior(1) = item->data(Qt::DisplayRole).toReal();
            break;
        case 9:
            images_[item->row()].TvecPrior(2) = item->data(Qt::DisplayRole).toReal();
            break;
        default:
            break;
    }
}

void ImageTab::ShowImage() {
    QItemSelectionModel* select = table_widget_->selectionModel();

    if (!select->hasSelection()) {
        QMessageBox::critical(this, "", tr("No image selected."));
        return;
    }

    if (select->selectedRows().size() > 1) {
        QMessageBox::critical(this, "", tr("Only one image may be selected."));
        return;
    }

    const auto& image = images_[select->selectedRows().begin()->row()];

    const auto keypoints = database_->ReadKeypoints(image.ImageId());
    const std::vector<bool> tri_mask(keypoints.size(), false);

    image_viewer_->Show(EnsureTrailingSlash(*options_->image_path) + image.Name(),
                        keypoints, tri_mask);
    image_viewer_->setWindowTitle(
            QString::fromStdString("Image " + std::to_string(image.ImageId())));
    image_viewer_->show();
}

void ImageTab::ShowMatches() {
    QItemSelectionModel* select = table_widget_->selectionModel();

    if (!select->hasSelection()) {
        QMessageBox::critical(this, "", tr("No image selected."));
        return;
    }

    if (select->selectedRows().size() > 1) {
        QMessageBox::critical(this, "", tr("Only one image may be selected."));
        return;
    }

    const auto& image = images_[select->selectedRows().begin()->row()];

    matches_widget_->ShowMatches(images_, image.ImageId());
    matches_widget_->show();
    matches_widget_->raise();
}

void ImageTab::SetCamera() {
    QItemSelectionModel* select = table_widget_->selectionModel();

    if (!select->hasSelection()) {
        QMessageBox::critical(this, "", tr("No image selected."));
        return;
    }

    bool ok;
    const camera_t camera_id = static_cast<camera_t>(
            QInputDialog::getInt(this, "", tr("camera_id"), 0, 0, INT_MAX, 1, &ok));
    if (!ok) {
        return;
    }

    if (!database_->ExistsCamera(camera_id)) {
        QMessageBox::critical(this, "", tr("camera_id does not exist."));
        return;
    }

    table_widget_->blockSignals(true);

    for (QModelIndex& index : select->selectedRows()) {
        table_widget_->setItem(index.row(), 2,
                               new QTableWidgetItem(QString::number(camera_id)));
        images_[index.row()].SetCameraId(camera_id);
    }

    table_widget_->blockSignals(false);
}

CameraTab::CameraTab(QWidget* parent, Database* database)
        : QWidget(parent), database_(database) {
    QGridLayout* grid = new QGridLayout(this);

    info_label_ = new QLabel(this);
    grid->addWidget(info_label_, 0, 0);

    QPushButton* add_camera_button = new QPushButton(tr("Add camera"), this);
    connect(add_camera_button, &QPushButton::released, this, &CameraTab::Add);
    grid->addWidget(add_camera_button, 0, 1, Qt::AlignRight);

    table_widget_ = new QTableWidget(this);
    table_widget_->setColumnCount(6);

    QStringList table_header;
    table_header << "camera_id"
                 << "model"
                 << "width"
                 << "height"
                 << "params"
                 << "prior_focal_length";
    table_widget_->setHorizontalHeaderLabels(table_header);

    table_widget_->setShowGrid(true);
    table_widget_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_widget_->horizontalHeader()->setStretchLastSection(true);
    table_widget_->verticalHeader()->setVisible(false);
    table_widget_->verticalHeader()->setDefaultSectionSize(20);

    connect(table_widget_, &QTableWidget::itemChanged, this,
            &CameraTab::itemChanged);

    grid->addWidget(table_widget_, 1, 0, 1, 2);
}

void CameraTab::Update() {
    QString info;
    info += QString("Cameras: ") + QString::number(database_->NumCameras());
    info_label_->setText(info);

    cameras_ = database_->ReadAllCameras();

    table_widget_->blockSignals(true);

    table_widget_->clearContents();
    table_widget_->setRowCount(cameras_.size());

    std::sort(cameras_.begin(), cameras_.end(),
              [](const Camera& camera1, const Camera& camera2) {
                  return camera1.CameraId() < camera2.CameraId();
              });

    for (size_t i = 0; i < cameras_.size(); ++i) {
        const Camera& camera = cameras_[i];
        QTableWidgetItem* id_item =
                new QTableWidgetItem(QString::number(camera.CameraId()));
        id_item->setFlags(Qt::ItemIsSelectable);
        table_widget_->setItem(i, 0, id_item);

        QTableWidgetItem* model_item =
                new QTableWidgetItem(QString::fromStdString(camera.ModelName()));
        model_item->setFlags(Qt::ItemIsSelectable);
        table_widget_->setItem(i, 1, model_item);

        table_widget_->setItem(
                i, 2, new QTableWidgetItem(QString::number(camera.Width())));
        table_widget_->setItem(
                i, 3, new QTableWidgetItem(QString::number(camera.Height())));

        table_widget_->setItem(i, 4, new QTableWidgetItem(QString::fromStdString(
                VectorToCSV(camera.Params()))));
        table_widget_->setItem(
                i, 5,
                new QTableWidgetItem(QString::number(camera.HasPriorFocalLength())));
    }
    table_widget_->resizeColumnsToContents();

    table_widget_->blockSignals(false);
}

void CameraTab::Save() {
    database_->BeginTransaction();
    for (const Camera& camera : cameras_) {
        database_->UpdateCamera(camera);
    }
    database_->EndTransaction();
}

void CameraTab::Clear() {
    cameras_.clear();
    table_widget_->clearContents();
}

void CameraTab::itemChanged(QTableWidgetItem* item) {
    Camera& camera = cameras_.at(item->row());
    const std::vector<double> prev_params = camera.Params();

    switch (item->column()) {
        case 2:
            camera.SetWidth(static_cast<size_t>(item->data(Qt::DisplayRole).toInt()));
            break;
        case 3:
            camera.SetHeight(
                    static_cast<size_t>(item->data(Qt::DisplayRole).toInt()));
            break;
        case 4:
            if (!camera.SetParamsFromString(item->text().toUtf8().constData())) {
                QMessageBox::critical(this, "", tr("Invalid camera parameters."));
                table_widget_->blockSignals(true);
                item->setText(QString::fromStdString(VectorToCSV(prev_params)));
                table_widget_->blockSignals(false);
            }
            break;
        case 5:
            camera.SetPriorFocalLength(
                    static_cast<bool>(item->data(Qt::DisplayRole).toInt()));
            break;
        default:
            break;
    }
}

void CameraTab::Add() {
    QStringList camera_models;
    camera_models << QString::fromStdString(CameraModelIdToName(RadialCameraModel::model_id));

    bool ok;
    const QString camera_model = QInputDialog::getItem(
            this, "", tr("Model:"), camera_models, 0, false, &ok);
    if (!ok) {
        return;
    }

    Camera camera;
    const double kDefaultFocalLength = 1.0;
    const size_t kDefaultWidth = 1;
    const size_t kDefaultHeight = 1;
    camera.InitializeWithName(camera_model.toUtf8().constData(),
                              kDefaultFocalLength, kDefaultWidth, kDefaultHeight);
    database_->WriteCamera(camera);

    Update();

    table_widget_->selectRow(cameras_.size() - 1);
}

DatabaseManagementWidget::DatabaseManagementWidget(QWidget* parent, OptionManager* options)
        : parent_(parent), options_(options) {
    setWindowFlags(Qt::Window);
    setWindowTitle("Database management");
    resize(parent->size().width() - 20, parent->size().height() - 20);

    QGridLayout* grid = new QGridLayout(this);

    tab_widget_ = new QTabWidget(this);

    image_tab_ = new ImageTab(this, options_, &database_);
    tab_widget_->addTab(image_tab_, tr("Images"));

    camera_tab_ = new CameraTab(this, &database_);
    tab_widget_->addTab(camera_tab_, tr("Cameras"));

    grid->addWidget(tab_widget_, 0, 0, 1, 2);

    QPushButton* save_button = new QPushButton(tr("Save"), this);
    connect(save_button, &QPushButton::released, this,
            &DatabaseManagementWidget::Save);
    grid->addWidget(save_button, 1, 0, Qt::AlignRight);

    QPushButton* cancel_button = new QPushButton(tr("Cancel"), this);
    connect(cancel_button, &QPushButton::released, this,
            &DatabaseManagementWidget::close);
    grid->addWidget(cancel_button, 1, 1, Qt::AlignRight);

    grid->setColumnStretch(0, 1);
}

void DatabaseManagementWidget::showEvent(QShowEvent* event) {
    parent_->setDisabled(true);

    database_.Open(*options_->database_path);

    image_tab_->Update();
    camera_tab_->Update();
}

void DatabaseManagementWidget::hideEvent(QHideEvent* event) {
    parent_->setEnabled(true);

    image_tab_->Clear();
    camera_tab_->Clear();

    database_.Close();
}

void DatabaseManagementWidget::Save() {
    image_tab_->Save();
    camera_tab_->Save();

    QMessageBox::information(this, "", tr("Saved changes"));
}
