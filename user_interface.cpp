#include "user_interface.h"

Eigen::Matrix4f QMatrixToEigen(const QMatrix4x4& matrix) {
    Eigen::Matrix4f eigen;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            eigen(r, c) = matrix(r, c);
        }
    }
    return eigen;
}

QMatrix4x4 EigenToQMatrix(const Eigen::Matrix4f& matrix) {
    QMatrix4x4 qt;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            qt(r, c) = matrix(r, c);
        }
    }
    return qt;
}

QImage BitmapToQImageRGB(const Bitmap& bitmap) {
    QImage pixmap(bitmap.Width(), bitmap.Height(), QImage::Format_RGB32);
    for (int y = 0; y < pixmap.height(); ++y) {
        QRgb* pixmap_line = (QRgb*)pixmap.scanLine(y);
        for (int x = 0; x < pixmap.width(); ++x) {
            Eigen::Vector3ub rgb;
            if (bitmap.GetPixel(x, y, &rgb)) {
                pixmap_line[x] = qRgba(rgb(0), rgb(1), rgb(2), 255);
            }
        }
    }
    return pixmap;
}

QPixmap ShowImagesSideBySide(const QPixmap& image1, const QPixmap& image2) {
    QPixmap image = QPixmap(QSize(image1.width() + image2.width(),
                                  std::max(image1.height(), image2.height())));

    image.fill(Qt::black);

    QPainter painter(&image);
    painter.drawImage(0, 0, image1.toImage());
    painter.drawImage(image1.width(), 0, image2.toImage());

    return image;
}

void DrawKeypoints(QPixmap* image, const FeatureKeypoints& points,
                   const QColor& color) {
    const int pen_width = std::max(image->width(), image->height()) / 2048 + 1;
    const int radius = 3 * pen_width + (3 * pen_width) % 2;
    const float radius2 = radius / 2.0f;

    QPainter painter(image);
    painter.setRenderHint(QPainter::Antialiasing);

    QPen pen;
    pen.setWidth(pen_width);
    pen.setColor(color);
    painter.setPen(pen);

    for (const auto& point : points) {
        painter.drawEllipse(point.x - radius2, point.y - radius2, radius, radius);
    }
}

QPixmap DrawMatches(const QPixmap& image1, const QPixmap& image2,
                    const FeatureKeypoints& points1,
                    const FeatureKeypoints& points2,
                    const FeatureMatches& matches,
                    const QColor& keypoints_color) {
    QPixmap image = ShowImagesSideBySide(image1, image2);

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);

    const int pen_width = std::max(image.width(), image.height()) / 2048 + 1;
    const int radius = 3 * pen_width + (3 * pen_width) % 2;
    const float radius2 = radius / 2.0f;

    QPen pen;
    pen.setWidth(pen_width);
    pen.setColor(keypoints_color);
    painter.setPen(pen);

    for (const auto& point : points1) {
        painter.drawEllipse(point.x - radius2, point.y - radius2, radius, radius);
    }
    for (const auto& point : points2) {
        painter.drawEllipse(image1.width() + point.x - radius2, point.y - radius2,
                            radius, radius);
    }

    pen.setWidth(std::max(pen_width / 2, 1));

    for (const auto& match : matches) {
        const point2D_t idx1 = match.point2D_idx1;
        const point2D_t idx2 = match.point2D_idx2;
        pen.setColor(QColor(0, 255, 0));
        painter.setPen(pen);
        painter.drawLine(QPoint(points1[idx1].x, points1[idx1].y),
                         QPoint(image1.width() + points2[idx2].x, points2[idx2].y));
    }

    return image;
}

float JetColormap::Red(const float gray) { return Base(gray - 0.25f); }

float JetColormap::Green(const float gray) { return Base(gray); }

float JetColormap::Blue(const float gray) { return Base(gray + 0.25f); }

float JetColormap::Base(const float val) {
    if (val <= 0.125f) {
        return 0.0f;
    } else if (val <= 0.375f) {
        return Interpolate(2.0f * val - 1.0f, 0.0f, -0.75f, 1.0f, -0.25f);
    } else if (val <= 0.625f) {
        return 1.0f;
    } else if (val <= 0.87f) {
        return Interpolate(2.0f * val - 1.0f, 1.0f, 0.25f, 0.0f, 0.75f);
    } else {
        return 0.0f;
    }
}

float JetColormap::Interpolate(const float val, const float y0, const float x0,
                               const float y1, const float x1) {
    return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

PointColormapBase::PointColormapBase()
        : scale(1.0f),
          min(0.0f),
          max(0.0f),
          range(0.0f),
          min_q(0.0f),
          max_q(1.0f) {}

void PointColormapBase::UpdateScale(std::vector<float>* values) {
    if (values->empty()) {
        min = 0.0f;
        max = 0.0f;
        range = 0.0f;
    } else {
        std::sort(values->begin(), values->end());
        min = (*values)[static_cast<size_t>(min_q * (values->size() - 1))];
        max = (*values)[static_cast<size_t>(max_q * (values->size() - 1))];
        range = max - min;
    }
}

float PointColormapBase::AdjustScale(const float gray) {
    if (range == 0.0f) {
        return 0.0f;
    } else {
        const float gray_clipped = std::min(std::max(gray, min), max);
        const float gray_scaled = (gray_clipped - min) / range;
        return std::pow(gray_scaled, scale);
    }
}

void PointColormapPhotometric::Prepare(
        std::unordered_map<camera_t, Camera>& cameras,
        std::unordered_map<image_t, Image>& images,
        std::unordered_map<point3D_t, Point_3D>& points3D,
        std::vector<image_t>& reg_image_ids) {}

Eigen::Vector3f PointColormapPhotometric::ComputeColor(
        const point3D_t point3D_id, const Point_3D& point3D) {
    return Eigen::Vector3f(point3D.Color(0) / 255.0f, point3D.Color(1) / 255.0f,
                           point3D.Color(2) / 255.0f);
}


PointPainter::PointPainter() : num_geoms_(0) {}

PointPainter::~PointPainter() {
    vao_.destroy();
    vbo_.destroy();
}

void PointPainter::Setup() {
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                            ":/shaders/points.v.glsl");
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                            ":/shaders/points.f.glsl");
    shader_program_.link();
    shader_program_.bind();

    vao_.create();
    vbo_.create();

#if DEBUG
    glDebugLog();
#endif
}

void PointPainter::Upload(const std::vector<PointPainter::Data>& data) {
    num_geoms_ = data.size();

    vao_.bind();
    vbo_.bind();

    vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vbo_.allocate(data.data(),
                  static_cast<int>(data.size() * sizeof(PointPainter::Data)));

    shader_program_.enableAttributeArray(0);
    shader_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3,
                                       sizeof(PointPainter::Data));

    shader_program_.enableAttributeArray(1);
    shader_program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(GLfloat), 4,
                                       sizeof(PointPainter::Data));

    vbo_.release();
    vao_.release();

#if DEBUG
    glDebugLog();
#endif
}

void PointPainter::Render(const QMatrix4x4& pmv_matrix,
                          const float point_size) {
    if (num_geoms_ == 0) {
        return;
    }

    shader_program_.bind();
    vao_.bind();

    shader_program_.setUniformValue("u_pmv_matrix", pmv_matrix);
    shader_program_.setUniformValue("u_point_size", point_size);

    glDrawArrays(GL_POINTS, 0, (GLsizei)num_geoms_);

    // Make sure the VAO is not changed from the outside
    vao_.release();

#if DEBUG
    glDebugLog();
#endif
}


LinePainter::LinePainter() : num_geoms_(0) {}

LinePainter::~LinePainter() {
    vao_.destroy();
    vbo_.destroy();
}

void LinePainter::Setup() {
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                            ":/shaders/lines.v.glsl");
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Geometry,
                                            ":/shaders/lines.g.glsl");
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                            ":/shaders/lines.f.glsl");
    shader_program_.link();
    shader_program_.bind();

    vao_.create();
    vbo_.create();

#if DEBUG
    glDebugLog();
#endif
}

void LinePainter::Upload(const std::vector<LinePainter::Data>& data) {
    num_geoms_ = data.size();

    vao_.bind();
    vbo_.bind();

    vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vbo_.allocate(data.data(),
                  static_cast<int>(data.size() * sizeof(LinePainter::Data)));

    shader_program_.enableAttributeArray(0);
    shader_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3,
                                       sizeof(PointPainter::Data));

    shader_program_.enableAttributeArray(1);
    shader_program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(GLfloat), 4,
                                       sizeof(PointPainter::Data));

    vbo_.release();
    vao_.release();

#if DEBUG
    glDebugLog();
#endif
}

void LinePainter::Render(const QMatrix4x4& pmv_matrix, const int width,
                         const int height, const float line_width) {
    if (num_geoms_ == 0) {
        return;
    }

    shader_program_.bind();
    vao_.bind();

    shader_program_.setUniformValue("u_pmv_matrix", pmv_matrix);
    shader_program_.setUniformValue("u_inv_viewport",
                                    QVector2D(1.0f / width, 1.0f / height));
    shader_program_.setUniformValue("u_line_width", line_width);

    glDrawArrays(GL_LINES, 0, (GLsizei)(2 * num_geoms_));

    vao_.release();

#if DEBUG
    glDebugLog();
#endif
}


TrianglePainter::TrianglePainter() : num_geoms_(0) {}

TrianglePainter::~TrianglePainter() {
    vao_.destroy();
    vbo_.destroy();
}

void TrianglePainter::Setup() {
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                            ":/shaders/triangles.v.glsl");
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                            ":/shaders/triangles.f.glsl");
    shader_program_.link();
    shader_program_.bind();

    vao_.create();
    vbo_.create();

#if DEBUG
    glDebugLog();
#endif
}

void TrianglePainter::Upload(const std::vector<TrianglePainter::Data>& data) {
    num_geoms_ = data.size();

    vao_.bind();
    vbo_.bind();

    vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vbo_.allocate(data.data(),
                  static_cast<int>(data.size() * sizeof(TrianglePainter::Data)));

    shader_program_.enableAttributeArray(0);
    shader_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3,
                                       sizeof(PointPainter::Data));

    shader_program_.enableAttributeArray(1);
    shader_program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(GLfloat), 4,
                                       sizeof(PointPainter::Data));

    vbo_.release();
    vao_.release();

#if DEBUG
    glDebugLog();
#endif
}

void TrianglePainter::Render(const QMatrix4x4& pmv_matrix) {
    if (num_geoms_ == 0) {
        return;
    }

    shader_program_.bind();
    vao_.bind();

    shader_program_.setUniformValue("u_pmv_matrix", pmv_matrix);

    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(3 * num_geoms_));

    vao_.release();

#if DEBUG
    glDebugLog();
#endif
}


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
        : BasicImageViewerWidget(parent, "matches") {}

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


#define POINT_SELECTED_R 0
#define POINT_SELECTED_G 1
#define POINT_SELECTED_B 0
#define IMAGE_R 1
#define IMAGE_G 0.1
#define IMAGE_B 0
#define IMAGE_A 0.6
#define IMAGE_SELECTED_R 1
#define IMAGE_SELECTED_G 0
#define IMAGE_SELECTED_B 1
#define IMAGE_SELECTED_A 0.6
#define SELECTION_BUFFER_IMAGE 0
#define SELECTION_BUFFER_POINT 1

#define GRID_RGBA 0.2, 0.2, 0.2, 0.6
#define X_AXIS_RGBA 0.9, 0, 0, 0.5
#define Y_AXIS_RGBA 0, 0.9, 0, 0.5
#define Z_AXIS_RGBA 0, 0, 0.9, 0.5


namespace {

    size_t RGBToIndex(const uint8_t r, const uint8_t g, const uint8_t b) {
        return static_cast<size_t>(r) + static_cast<size_t>(g) * 256 +
               static_cast<size_t>(b) * 65536;
    }

    void IndexToRGB(const size_t index, float& r, float& g, float& b) {
        r = ((index & 0x000000FF) >> 0) / 255.0f;
        g = ((index & 0x0000FF00) >> 8) / 255.0f;
        b = ((index & 0x00FF0000) >> 16) / 255.0f;
    }

    void FrameBufferToQImage(QImage& image) {
        if (QSysInfo::ByteOrder == QSysInfo::BigEndian) {
            uint* p = (uint*)image.bits();
            uint* end = p + image.width() * image.height();
            while (p < end) {
                uint a = *p << 24;
                *p = (*p >> 8) | a;
                p++;
            }
        } else {
            for (int y = 0; y < image.height(); y++) {
                uint* q = (uint*)image.scanLine(y);
                for (int x = 0; x < image.width(); ++x) {
                    const uint pixel = *q;
                    *q = ((pixel << 16) & 0xff0000) | ((pixel >> 16) & 0xff) |
                         (pixel & 0xff00ff00);
                    q++;
                }
            }
        }
        image = image.mirrored();
    }

    void BuildImageModel(const Image& image, const Camera& camera,
                         const float image_size, const float r, const float g,
                         const float b, const float a, LinePainter::Data& line1,
                         LinePainter::Data& line2, LinePainter::Data& line3,
                         LinePainter::Data& line4, LinePainter::Data& line5,
                         LinePainter::Data& line6, LinePainter::Data& line7,
                         LinePainter::Data& line8, TrianglePainter::Data& triangle1,
                         TrianglePainter::Data& triangle2) {
        const float image_width = image_size * camera.Width() / 1024.0f;
        const float image_height =
                image_width * static_cast<float>(camera.Height()) / camera.Width();
        const float image_extent = std::max(image_width, image_height);
        const float camera_extent = std::max(camera.Width(), camera.Height());
        const float camera_extent_world =
                static_cast<float>(camera.ImageToWorldThreshold(camera_extent));
        const float focal_length = 2.0f * image_extent / camera_extent_world;

        const Eigen::Matrix<float, 3, 4> inv_proj_matrix =
                image.InverseProjectionMatrix().cast<float>();

        const Eigen::Vector3f pc = inv_proj_matrix.rightCols<1>();
        const Eigen::Vector3f tl =
                inv_proj_matrix *
                Eigen::Vector4f(-image_width, image_height, focal_length, 1);
        const Eigen::Vector3f tr =
                inv_proj_matrix *
                Eigen::Vector4f(image_width, image_height, focal_length, 1);
        const Eigen::Vector3f br =
                inv_proj_matrix *
                Eigen::Vector4f(image_width, -image_height, focal_length, 1);
        const Eigen::Vector3f bl =
                inv_proj_matrix *
                Eigen::Vector4f(-image_width, -image_height, focal_length, 1);

        line1.point1 = PointPainter::Data(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
        line1.point2 = PointPainter::Data(tl(0), tl(1), tl(2), 0.8f * r, g, b, 1);

        line2.point1 = PointPainter::Data(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
        line2.point2 = PointPainter::Data(tr(0), tr(1), tr(2), 0.8f * r, g, b, 1);

        line3.point1 = PointPainter::Data(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
        line3.point2 = PointPainter::Data(br(0), br(1), br(2), 0.8f * r, g, b, 1);

        line4.point1 = PointPainter::Data(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
        line4.point2 = PointPainter::Data(bl(0), bl(1), bl(2), 0.8f * r, g, b, 1);

        line5.point1 = PointPainter::Data(tl(0), tl(1), tl(2), 0.8f * r, g, b, 1);
        line5.point2 = PointPainter::Data(tr(0), tr(1), tr(2), 0.8f * r, g, b, 1);

        line6.point1 = PointPainter::Data(tr(0), tr(1), tr(2), 0.8f * r, g, b, 1);
        line6.point2 = PointPainter::Data(br(0), br(1), br(2), 0.8f * r, g, b, 1);

        line7.point1 = PointPainter::Data(br(0), br(1), br(2), 0.8f * r, g, b, 1);
        line7.point2 = PointPainter::Data(bl(0), bl(1), bl(2), 0.8f * r, g, b, 1);

        line8.point1 = PointPainter::Data(bl(0), bl(1), bl(2), 0.8f * r, g, b, 1);
        line8.point2 = PointPainter::Data(tl(0), tl(1), tl(2), 0.8f * r, g, b, 1);

        triangle1.point1 = PointPainter::Data(tl(0), tl(1), tl(2), r, g, b, a);
        triangle1.point2 = PointPainter::Data(tr(0), tr(1), tr(2), r, g, b, a);
        triangle1.point3 = PointPainter::Data(bl(0), bl(1), bl(2), r, g, b, a);

        triangle2.point1 = PointPainter::Data(bl(0), bl(1), bl(2), r, g, b, a);
        triangle2.point2 = PointPainter::Data(tr(0), tr(1), tr(2), r, g, b, a);
        triangle2.point3 = PointPainter::Data(br(0), br(1), br(2), r, g, b, a);
    }

}

OpenGLWindow::OpenGLWindow(QWidget* parent, OptionManager* options,
                           QScreen* screen)
        : QWindow(screen),
          options_(options),
          point_viewer_widget_(new PointViewerWidget(parent, this, options)),
          image_viewer_widget_(new ImageViewerWidget(parent, this, options)),
          projection_type_(ProjectionType::ORTHOGRAPHIC),
          mouse_is_pressed_(false),
          focus_distance_(kInitFocusDistance),
          selected_image_id_(kInvalidImageId),
          selected_point3D_id_(kInvalidPoint3DId),
          coordinate_grid_enabled_(true),
          near_plane_(kInitNearPlane) {
    bg_color_[0] = 1.0f;
    bg_color_[1] = 1.0f;
    bg_color_[2] = 1.0f;

    SetupGL();
    ResizeGL();

    SetPointColormap(new PointColormapPhotometric());

    image_size_ = static_cast<float>(devicePixelRatio() * image_size_);
    point_size_ = static_cast<float>(devicePixelRatio() * point_size_);
}

void OpenGLWindow::Update() {
    cameras = reconstruction->Cameras();
    points3D = reconstruction->Points3D();
    reg_image_ids = reconstruction->RegImageIds();

    images.clear();
    for (const image_t image_id : reg_image_ids) {
        images[image_id] = reconstruction->Image(image_id);
    }

    statusbar_status_label->setText(QString().sprintf(
            "%d Images - %d Points", static_cast<int>(reg_image_ids.size()),
            static_cast<int>(points3D.size())));

    Upload();
}

void OpenGLWindow::Upload() {
    point_colormap_->Prepare(cameras, images, points3D, reg_image_ids);

    UploadPointData();
    UploadImageData();
    UploadPointConnectionData();
    UploadImageConnectionData();
    PaintGL();
}

void OpenGLWindow::Clear() {
    cameras.clear();
    images.clear();
    points3D.clear();
    reg_image_ids.clear();
    Upload();
}

OpenGLWindow::ProjectionType OpenGLWindow::GetProjectionType() const {
    return projection_type_;
}

void OpenGLWindow::SetProjectionType(const ProjectionType type) {
    projection_type_ = type;
    ComposeProjectionMatrix();
    PaintGL();
}

void OpenGLWindow::SetPointColormap(PointColormapBase* colormap) {
    point_colormap_.reset(colormap);
}

void OpenGLWindow::EnableCoordinateGrid() {
    coordinate_grid_enabled_ = true;
    PaintGL();
}

void OpenGLWindow::DisableCoordinateGrid() {
    coordinate_grid_enabled_ = false;
    PaintGL();
}

void OpenGLWindow::ChangeFocusDistance(const float delta) {
    if (delta == 0.0f) {
        return;
    }
    const float prev_focus_distance = focus_distance_;
    float diff = delta * ZoomScale() * kFocusSpeed;
    focus_distance_ -= diff;
    if (focus_distance_ < kMinFocusDistance) {
        focus_distance_ = kMinFocusDistance;
        diff = prev_focus_distance - focus_distance_;
    } else if (focus_distance_ > kMaxFocusDistance) {
        focus_distance_ = kMaxFocusDistance;
        diff = prev_focus_distance - focus_distance_;
    }
    const Eigen::Matrix4f vm_mat = QMatrixToEigen(model_view_matrix_).inverse();
    const Eigen::Vector3f tvec(0, 0, diff);
    const Eigen::Vector3f tvec_rot = vm_mat.block<3, 3>(0, 0) * tvec;
    model_view_matrix_.translate(tvec_rot(0), tvec_rot(1), tvec_rot(2));
    ComposeProjectionMatrix();
    UploadCoordinateGridData();
    PaintGL();
}

void OpenGLWindow::ChangeNearPlane(const float delta) {
    if (delta == 0.0f) {
        return;
    }
    near_plane_ *= (1.0f + delta / 100.0f * kNearPlaneScaleSpeed);
    near_plane_ = std::max(kMinNearPlane, std::min(kMaxNearPlane, near_plane_));
    ComposeProjectionMatrix();
    UploadCoordinateGridData();
    PaintGL();
}

void OpenGLWindow::ChangePointSize(const float delta) {
    if (delta == 0.0f) {
        return;
    }
    point_size_ *= (1.0f + delta / 100.0f * kPointScaleSpeed);
    point_size_ = std::max(kMinPointSize, std::min(kMaxPointSize, point_size_));
    PaintGL();
}

void OpenGLWindow::RotateView(const float x, const float y, const float prev_x,
                              const float prev_y) {
    if (x - prev_x == 0 && y - prev_y == 0) {
        return;
    }

    const Eigen::Vector3f u = PositionToArcballVector(x, y);
    const Eigen::Vector3f v = PositionToArcballVector(prev_x, prev_y);

    const float angle = 2.0f * std::acos(std::min(1.0f, u.dot(v)));

    const float kMinAngle = 1e-3f;
    if (angle > kMinAngle) {
        const Eigen::Matrix4f vm_mat = QMatrixToEigen(model_view_matrix_).inverse();

        Eigen::Vector3f axis = vm_mat.block<3, 3>(0, 0) * v.cross(u);
        axis = axis.normalized();
        const Eigen::Vector4f rot_center =
                vm_mat * Eigen::Vector4f(0, 0, -focus_distance_, 1);
        model_view_matrix_.translate(rot_center(0), rot_center(1), rot_center(2));
        model_view_matrix_.rotate(RadToDeg(angle), axis(0), axis(1), axis(2));
        model_view_matrix_.translate(-rot_center(0), -rot_center(1),
                                     -rot_center(2));
        PaintGL();
    }
}

void OpenGLWindow::TranslateView(const float x, const float y,
                                 const float prev_x, const float prev_y) {
    if (x - prev_x == 0 && y - prev_y == 0) {
        return;
    }

    Eigen::Vector3f tvec(x - prev_x, prev_y - y, 0.0f);

    if (projection_type_ == ProjectionType::PERSPECTIVE) {
        tvec *= ZoomScale();
    } else if (projection_type_ == ProjectionType::ORTHOGRAPHIC) {
        tvec *= 2.0f * OrthographicWindowExtent() / height();
    }

    const Eigen::Matrix4f vm_mat = QMatrixToEigen(model_view_matrix_).inverse();

    const Eigen::Vector3f tvec_rot = vm_mat.block<3, 3>(0, 0) * tvec;
    model_view_matrix_.translate(tvec_rot(0), tvec_rot(1), tvec_rot(2));

    PaintGL();
}

void OpenGLWindow::ChangeImageSize(const float delta) {
    if (delta == 0.0f) {
        return;
    }
    image_size_ *= (1.0f + delta / 100.0f * kImageScaleSpeed);
    image_size_ = std::max(kMinImageSize, std::min(kMaxImageSize, image_size_));
    UploadImageData();
    PaintGL();
}

void OpenGLWindow::ResetView() {
    InitializeView();
    Upload();
}

QMatrix4x4 OpenGLWindow::ModelViewMatrix() const { return model_view_matrix_; }

void OpenGLWindow::SetModelViewMatrix(const QMatrix4x4& matrix) {
    model_view_matrix_ = matrix;
    PaintGL();
}

void OpenGLWindow::SelectObject(const int x, const int y) {
    glClearColor(bg_color_[0], bg_color_[1], bg_color_[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glDisable(GL_MULTISAMPLE);

    UploadImageData(true);
    UploadPointData(true);

    const QMatrix4x4 pmv_matrix = projection_matrix_ * model_view_matrix_;
    image_triangle_painter_.Render(pmv_matrix);
    point_painter_.Render(pmv_matrix, 2 * point_size_);

    const Eigen::Vector4ub rgba = ReadPixelColor(x, y);
    const size_t index = RGBToIndex(rgba[0], rgba[1], rgba[2]);

    if (index < selection_buffer_.size()) {
        const char buffer_type = selection_buffer_[index].second;
        if (buffer_type == SELECTION_BUFFER_IMAGE) {
            selected_image_id_ = static_cast<image_t>(selection_buffer_[index].first);
            selected_point3D_id_ = kInvalidPoint3DId;
            ShowImageInfo(selected_image_id_);
        } else if (buffer_type == SELECTION_BUFFER_POINT) {
            selected_image_id_ = kInvalidImageId;
            selected_point3D_id_ = selection_buffer_[index].first;
            ShowPointInfo(selection_buffer_[index].first);
        } else {
            selected_image_id_ = kInvalidImageId;
            selected_point3D_id_ = kInvalidPoint3DId;
            image_viewer_widget_->hide();
        }
    } else {
        selected_image_id_ = kInvalidImageId;
        selected_point3D_id_ = kInvalidPoint3DId;
        image_viewer_widget_->hide();
    }

    glEnable(GL_MULTISAMPLE);

    selection_buffer_.clear();

    UploadPointData();
    UploadImageData();
    UploadPointConnectionData();
    UploadImageConnectionData();

    PaintGL();
}

QImage OpenGLWindow::GrabImage() {
    DisableCoordinateGrid();

    const int scaled_width = static_cast<int>(devicePixelRatio() * width());
    const int scaled_height = static_cast<int>(devicePixelRatio() * height());

    QImage image(scaled_width, scaled_height, QImage::Format_ARGB32);
    glReadPixels(0, 0, scaled_width, scaled_height, GL_RGBA, GL_UNSIGNED_BYTE,
                 image.bits());

    FrameBufferToQImage(image);

    EnableCoordinateGrid();

    return image;
}

void OpenGLWindow::ShowPointInfo(const point3D_t point3D_id) {
    point_viewer_widget_->Show(point3D_id);
    point_viewer_widget_->show();
}

void OpenGLWindow::ShowImageInfo(const image_t image_id) {
    image_viewer_widget_->Show(image_id);
    image_viewer_widget_->show();
}

float OpenGLWindow::PointSize() const { return point_size_; }

float OpenGLWindow::ImageSize() const { return image_size_; }

void OpenGLWindow::SetPointSize(const float point_size) {
    point_size_ = point_size;
}

void OpenGLWindow::SetImageSize(const float image_size) {
    image_size_ = image_size;
    UploadImageData();
}

void OpenGLWindow::SetBackgroundColor(const float r, const float g,
                                      const float b) {
    bg_color_[0] = r;
    bg_color_[1] = g;
    bg_color_[2] = b;
    glClearColor(bg_color_[0], bg_color_[1], bg_color_[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLWindow::exposeEvent(QExposeEvent*) { PaintGL(); }

void OpenGLWindow::mousePressEvent(QMouseEvent* event) {
    if (mouse_press_timer_.isActive()) {
        mouse_is_pressed_ = false;
        mouse_press_timer_.stop();
        selection_buffer_.clear();
        SelectObject(event->pos().x(), event->pos().y());
    } else {
        mouse_press_timer_.setSingleShot(true);
        mouse_press_timer_.start(kDoubleClickInterval);
        mouse_is_pressed_ = true;
        prev_mouse_pos_ = event->pos();
    }
    event->accept();
}

void OpenGLWindow::mouseReleaseEvent(QMouseEvent* event) {
    mouse_is_pressed_ = false;
    event->accept();
}

void OpenGLWindow::mouseMoveEvent(QMouseEvent* event) {
    if (mouse_is_pressed_) {
        if (event->buttons() & Qt::RightButton ||
            (event->buttons() & Qt::LeftButton &&
             event->modifiers() & Qt::ControlModifier)) {
            TranslateView(event->pos().x(), event->pos().y(), prev_mouse_pos_.x(),
                          prev_mouse_pos_.y());
        } else if (event->buttons() & Qt::LeftButton) {
            RotateView(event->pos().x(), event->pos().y(), prev_mouse_pos_.x(),
                       prev_mouse_pos_.y());
        }
    }
    prev_mouse_pos_ = event->pos();
    event->accept();
}

void OpenGLWindow::wheelEvent(QWheelEvent* event) {
    if (event->modifiers() & Qt::ControlModifier) {
        ChangePointSize(event->delta());
    } else if (event->modifiers() & Qt::AltModifier) {
        ChangeImageSize(event->delta());
    } else if (event->modifiers() & Qt::ShiftModifier) {
        ChangeNearPlane(event->delta());
    } else {
        ChangeFocusDistance(event->delta());
    }
    event->accept();
}

void OpenGLWindow::SetupGL() {
    setSurfaceType(OpenGLSurface);

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setMajorVersion(3);
    format.setMinorVersion(2);
    format.setSamples(4);
    format.setProfile(QSurfaceFormat::CoreProfile);
#ifdef DEBUG
    format.setOption(QSurfaceFormat::DebugContext);
#endif

    setFormat(format);
    create();

    context_ = new QOpenGLContext(this);
    context_->setFormat(format);
    context_->create();

    InitializeGL();

    connect(this, &QWindow::widthChanged, this, &OpenGLWindow::ResizeGL);
    connect(this, &QWindow::heightChanged, this, &OpenGLWindow::ResizeGL);

    SetupPainters();

#ifdef DEBUG
    std::cout << "Selected OpenGL version: " << format.majorVersion() << "."
        << format.minorVersion() << std::endl;
std::cout << "Context validity: " << context_->isValid() << std::endl;
std::cout << "Used OpenGL version: " << context_->format().majorVersion()
        << "." << context_->format().minorVersion() << std::endl;
std::cout << "OpenGL information: VENDOR:       "
        << (const char*)glGetString(GL_VENDOR) << std::endl;
std::cout << "                    RENDERDER:    "
        << (const char*)glGetString(GL_RENDERER) << std::endl;
std::cout << "                    VERSION:      "
        << (const char*)glGetString(GL_VERSION) << std::endl;
std::cout << "                    GLSL VERSION: "
        << (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION);
std::cout << std::endl;

auto extensions = context_->extensions().toList();
qSort(extensions);
std::cout << "Supported extensions (" << extensions.count()
        << "):" << std::endl;
foreach (const QByteArray& extension, extensions)
std::cout << "    " << extension.data() << std::endl;
#endif
}

void OpenGLWindow::SetupPainters() {
    coordinate_axes_painter_.Setup();
    coordinate_grid_painter_.Setup();

    point_painter_.Setup();
    point_connection_painter_.Setup();

    image_line_painter_.Setup();
    image_triangle_painter_.Setup();
    image_connection_painter_.Setup();
}

void OpenGLWindow::InitializeGL() {
    context_->makeCurrent(this);
    InitializeSettings();
    InitializeView();
}

void OpenGLWindow::PaintGL() {
    context_->makeCurrent(this);

    glClearColor(bg_color_[0], bg_color_[1], bg_color_[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const QMatrix4x4 pmv_matrix = projection_matrix_ * model_view_matrix_;

    QMatrix4x4 model_view_center_matrix = model_view_matrix_;
    const Eigen::Vector4f rot_center =
            QMatrixToEigen(model_view_matrix_).inverse() *
            Eigen::Vector4f(0, 0, -focus_distance_, 1);
    model_view_center_matrix.translate(rot_center(0), rot_center(1),
                                       rot_center(2));
    const QMatrix4x4 pmvc_matrix = projection_matrix_ * model_view_center_matrix;

    if (coordinate_grid_enabled_) {
        coordinate_axes_painter_.Render(pmv_matrix, width(), height(), 2);
        coordinate_grid_painter_.Render(pmvc_matrix, width(), height(), 1);
    }

    point_painter_.Render(pmv_matrix, point_size_);
    point_connection_painter_.Render(pmv_matrix, width(), height(), 1);

    image_line_painter_.Render(pmv_matrix, width(), height(), 1);
    image_triangle_painter_.Render(pmv_matrix);
    image_connection_painter_.Render(pmv_matrix, width(), height(), 1);

    context_->swapBuffers(this);
}

void OpenGLWindow::ResizeGL() {
    context_->makeCurrent(this);
    glViewport(0, 0, width(), height());
    ComposeProjectionMatrix();
    UploadCoordinateGridData();
}

void OpenGLWindow::InitializeSettings() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
}

void OpenGLWindow::InitializeView() {
    point_size_ = kInitPointSize;
    image_size_ = kInitImageSize;
    focus_distance_ = kInitFocusDistance;
    model_view_matrix_.setToIdentity();
    model_view_matrix_.translate(0, 0, -focus_distance_);
    model_view_matrix_.rotate(225, 1, 0, 0);
    model_view_matrix_.rotate(-45, 0, 1, 0);
}

void OpenGLWindow::UploadCoordinateGridData() {
    const float scale = ZoomScale();

    std::vector<LinePainter::Data> grid_data(3);

    grid_data[0].point1 = PointPainter::Data(-20 * scale, 0, 0, GRID_RGBA);
    grid_data[0].point2 = PointPainter::Data(20 * scale, 0, 0, GRID_RGBA);

    grid_data[1].point1 = PointPainter::Data(0, -20 * scale, 0, GRID_RGBA);
    grid_data[1].point2 = PointPainter::Data(0, 20 * scale, 0, GRID_RGBA);

    grid_data[2].point1 = PointPainter::Data(0, 0, -20 * scale, GRID_RGBA);
    grid_data[2].point2 = PointPainter::Data(0, 0, 20 * scale, GRID_RGBA);

    coordinate_grid_painter_.Upload(grid_data);

    std::vector<LinePainter::Data> axes_data(3);

    axes_data[0].point1 = PointPainter::Data(0, 0, 0, X_AXIS_RGBA);
    axes_data[0].point2 = PointPainter::Data(50 * scale, 0, 0, X_AXIS_RGBA);

    axes_data[1].point1 = PointPainter::Data(0, 0, 0, Y_AXIS_RGBA);
    axes_data[1].point2 = PointPainter::Data(0, 50 * scale, 0, Y_AXIS_RGBA);

    axes_data[2].point1 = PointPainter::Data(0, 0, 0, Z_AXIS_RGBA);
    axes_data[2].point2 = PointPainter::Data(0, 0, 50 * scale, Z_AXIS_RGBA);

    coordinate_axes_painter_.Upload(axes_data);
}

void OpenGLWindow::UploadPointData(const bool selection_mode) {
    std::vector<PointPainter::Data> data;

    data.reserve(points3D.size());

    const size_t min_track_len =
            static_cast<size_t>(options_->render_options->min_track_len);

    if (selected_image_id_ == kInvalidImageId &&
        images.count(selected_image_id_) == 0) {
        for (const auto& point3D : points3D) {
            if (point3D.second.Error() <= options_->render_options->max_error &&
                point3D.second.Track().Length() >= min_track_len) {
                PointPainter::Data painter_point;
                painter_point.x = static_cast<float>(point3D.second.XYZ(0));
                painter_point.y = static_cast<float>(point3D.second.XYZ(1));
                painter_point.z = static_cast<float>(point3D.second.XYZ(2));
                if (selection_mode) {
                    const size_t index = selection_buffer_.size();
                    selection_buffer_.push_back(
                            std::make_pair(point3D.first, SELECTION_BUFFER_POINT));
                    IndexToRGB(index, painter_point.r, painter_point.g, painter_point.b);
                } else if (point3D.first == selected_point3D_id_) {
                    painter_point.r = POINT_SELECTED_R;
                    painter_point.g = POINT_SELECTED_G;
                    painter_point.b = POINT_SELECTED_B;
                } else {
                    const Eigen::Vector3f& rgb =
                            point_colormap_->ComputeColor(point3D.first, point3D.second);
                    painter_point.r = rgb(0);
                    painter_point.g = rgb(1);
                    painter_point.b = rgb(2);
                }
                painter_point.a = 1;
                data.push_back(painter_point);
            }
        }
    } else {
        const auto& selected_image = images[selected_image_id_];
        for (const auto& point3D : points3D) {
            if (point3D.second.Error() <= options_->render_options->max_error &&
                point3D.second.Track().Length() >= min_track_len) {
                PointPainter::Data painter_point;
                painter_point.x = static_cast<float>(point3D.second.XYZ(0));
                painter_point.y = static_cast<float>(point3D.second.XYZ(1));
                painter_point.z = static_cast<float>(point3D.second.XYZ(2));
                if (selection_mode) {
                    const size_t index = selection_buffer_.size();
                    selection_buffer_.push_back(
                            std::make_pair(point3D.first, SELECTION_BUFFER_POINT));
                    IndexToRGB(index, painter_point.r, painter_point.g, painter_point.b);
                } else if (selected_image.HasPoint3D(point3D.first)) {
                    painter_point.r = IMAGE_SELECTED_R;
                    painter_point.g = IMAGE_SELECTED_G;
                    painter_point.b = IMAGE_SELECTED_B;
                } else if (point3D.first == selected_point3D_id_) {
                    painter_point.r = POINT_SELECTED_R;
                    painter_point.g = POINT_SELECTED_G;
                    painter_point.b = POINT_SELECTED_B;
                } else {
                    const Eigen::Vector3f& rgb =
                            point_colormap_->ComputeColor(point3D.first, point3D.second);
                    painter_point.r = rgb(0);
                    painter_point.g = rgb(1);
                    painter_point.b = rgb(2);
                }
                painter_point.a = 1;
                data.push_back(painter_point);
            }
        }
    }

    point_painter_.Upload(data);
}

void OpenGLWindow::UploadPointConnectionData() {
    std::vector<LinePainter::Data> line_data;

    if (selected_point3D_id_ == kInvalidPoint3DId) {
        point_connection_painter_.Upload(line_data);
        return;
    }

    const auto& point3D = points3D[selected_point3D_id_];

    LinePainter::Data line;
    line.point1 = PointPainter::Data(
            static_cast<float>(point3D.XYZ(0)), static_cast<float>(point3D.XYZ(1)),
            static_cast<float>(point3D.XYZ(2)), POINT_SELECTED_R, POINT_SELECTED_G,
            POINT_SELECTED_B, 0.8);

    for (const auto& track_el : point3D.Track().Elements()) {
        const Image& conn_image = images[track_el.image_id];
        const Eigen::Vector3f conn_proj_center =
                conn_image.ProjectionCenter().cast<float>();
        line.point2 = PointPainter::Data(conn_proj_center(0), conn_proj_center(1),
                                         conn_proj_center(2), POINT_SELECTED_R,
                                         POINT_SELECTED_G, POINT_SELECTED_B, 1);
        line_data.push_back(line);
    }

    point_connection_painter_.Upload(line_data);
}

void OpenGLWindow::UploadImageData(const bool selection_mode) {
    std::vector<LinePainter::Data> line_data;
    line_data.reserve(8 * reg_image_ids.size());

    std::vector<TrianglePainter::Data> triangle_data;
    triangle_data.reserve(2 * reg_image_ids.size());

    for (const image_t image_id : reg_image_ids) {
        const Image& image = images[image_id];
        const Camera& camera = cameras[image.CameraId()];

        float r, g, b, a;
        if (selection_mode) {
            const size_t index = selection_buffer_.size();
            selection_buffer_.push_back(
                    std::make_pair(image_id, SELECTION_BUFFER_IMAGE));
            IndexToRGB(index, r, g, b);
            a = 1;
        } else {
            if (image_id == selected_image_id_) {
                r = IMAGE_SELECTED_R;
                g = IMAGE_SELECTED_G;
                b = IMAGE_SELECTED_B;
                a = IMAGE_SELECTED_A;
            } else {
                r = IMAGE_R;
                g = IMAGE_G;
                b = IMAGE_B;
                a = IMAGE_A;
            }
        }

        LinePainter::Data line1, line2, line3, line4, line5, line6, line7, line8;
        TrianglePainter::Data triangle1, triangle2;
        BuildImageModel(image, camera, image_size_, r, g, b, a, line1, line2, line3,
                        line4, line5, line6, line7, line8, triangle1, triangle2);

        if (!selection_mode) {
            line_data.push_back(line1);
            line_data.push_back(line2);
            line_data.push_back(line3);
            line_data.push_back(line4);
            line_data.push_back(line5);
            line_data.push_back(line6);
            line_data.push_back(line7);
            line_data.push_back(line8);
        }

        triangle_data.push_back(triangle1);
        triangle_data.push_back(triangle2);
    }

    image_line_painter_.Upload(line_data);
    image_triangle_painter_.Upload(triangle_data);
}

void OpenGLWindow::UploadImageConnectionData() {
    std::vector<LinePainter::Data> line_data;
    std::vector<image_t> image_ids;

    if (selected_image_id_ != kInvalidImageId) {
        image_ids.push_back(selected_image_id_);
    } else if (options_->render_options->image_connections) {
        image_ids = reg_image_ids;
    } else {
        image_connection_painter_.Upload(line_data);
        return;
    }

    for (const image_t image_id : image_ids) {
        const Image& image = images.at(image_id);

        const Eigen::Vector3f proj_center = image.ProjectionCenter().cast<float>();

        std::unordered_set<image_t> conn_image_ids;

        for (const Point2D& point2D : image.Points2D()) {
            if (point2D.HasPoint3D()) {
                const Point_3D& point3D = points3D[point2D.Point3DId()];
                for (const auto& track_elem : point3D.Track().Elements()) {
                    conn_image_ids.insert(track_elem.image_id);
                }
            }
        }

        LinePainter::Data line;
        line.point1 = PointPainter::Data(proj_center(0), proj_center(1),
                                         proj_center(2), IMAGE_SELECTED_R,
                                         IMAGE_SELECTED_G, IMAGE_SELECTED_B, 0.8);

        for (const image_t conn_image_id : conn_image_ids) {
            const Image& conn_image = images[conn_image_id];
            const Eigen::Vector3f conn_proj_center =
                    conn_image.ProjectionCenter().cast<float>();
            line.point2 = PointPainter::Data(conn_proj_center(0), conn_proj_center(1),
                                             conn_proj_center(2), IMAGE_SELECTED_R,
                                             IMAGE_SELECTED_G, IMAGE_SELECTED_B, 0.8);
            line_data.push_back(line);
        }
    }

    image_connection_painter_.Upload(line_data);
}

void OpenGLWindow::ComposeProjectionMatrix() {
    projection_matrix_.setToIdentity();
    if (projection_type_ == ProjectionType::PERSPECTIVE) {
        projection_matrix_.perspective(kFieldOfView, AspectRatio(), near_plane_,
                                       kFarPlane);
    } else if (projection_type_ == ProjectionType::ORTHOGRAPHIC) {
        const float extent = OrthographicWindowExtent();
        projection_matrix_.ortho(-AspectRatio() * extent, AspectRatio() * extent,
                                 -extent, extent, near_plane_, kFarPlane);
    }
}

float OpenGLWindow::ZoomScale() const {
    return 2.0f * std::tan(static_cast<float>(DegToRad(kFieldOfView)) / 2.0f) *
           std::abs(focus_distance_) / height();
}

float OpenGLWindow::AspectRatio() const {
    return static_cast<float>(width()) / static_cast<float>(height());
}

float OpenGLWindow::OrthographicWindowExtent() const {
    return std::tan(DegToRad(kFieldOfView) / 2.0f) * focus_distance_;
}

Eigen::Vector4ub OpenGLWindow::ReadPixelColor(int x, int y) const {
    x = static_cast<int>(devicePixelRatio() * x);
    y = static_cast<int>(devicePixelRatio() * (height() - y - 1));
    Eigen::Vector4ub rgba;
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    return rgba;
}

Eigen::Vector3f OpenGLWindow::PositionToArcballVector(const float x,
                                                      const float y) const {
    Eigen::Vector3f vec(2.0f * x / width() - 1, 1 - 2.0f * y / height(), 0.0f);
    const float norm2 = vec.squaredNorm();
    if (norm2 <= 1.0f) {
        vec.z() = std::sqrt(1.0f - norm2);
    } else {
        vec = vec.normalized();
    }
    return vec;
}


NewProjectWidget::NewProjectWidget(MainWindow* parent, OptionManager* options)
        : main_window_(parent), options_(options), prev_selected_(false) {
    setWindowFlags(Qt::Dialog);
    setWindowModality(Qt::ApplicationModal);
    setWindowTitle("New project");

    QPushButton* image_path_select = new QPushButton(tr("Select"), this);
    connect(image_path_select, &QPushButton::released, this,
            &NewProjectWidget::SelectImagePath);
    image_path_text_ = new QLineEdit(this);
    image_path_text_->setText(QString::fromStdString(*options_->image_path));

    QPushButton* create_button = new QPushButton(tr("Create"), this);
    connect(create_button, &QPushButton::released, this,
            &NewProjectWidget::Create);

    QGridLayout* grid = new QGridLayout(this);

    grid->addWidget(new QLabel(tr("Images"), this), 1, 0);
    grid->addWidget(image_path_text_, 1, 1);
    grid->addWidget(image_path_select, 1, 2);

    grid->addWidget(create_button, 2, 2);
}

bool NewProjectWidget::IsValid() {
    return boost::filesystem::is_directory(ImagePath());
}

std::string NewProjectWidget::ImagePath() const {
    return EnsureTrailingSlash(image_path_text_->text().toUtf8().constData());
}

void NewProjectWidget::SetImagePath(const std::string& path) {
    image_path_text_->setText(QString::fromStdString(path));
}

void NewProjectWidget::Create() {
    if (!IsValid()) {
        QMessageBox::critical(this, "", tr("Invalid paths."));
    } else {
        if (main_window_->mapper_controller->NumModels() > 0) {
            if (!main_window_->OverwriteReconstruction()) {
                return;
            }
        }
        boost::uuids::basic_random_generator<boost::mt19937> gen;
        boost::uuids::uuid u = gen();

        std::string s1 = boost::uuids::to_string(u);
        boost::filesystem::path directory = boost::filesystem::temp_directory_path() / boost::filesystem::path(s1);
        boost::filesystem::create_directory(directory);
        *options_->database_path = (directory / boost::filesystem::path("base.db")).string();
        *options_->image_path = ImagePath();

        Database database;
        database.Open(*options_->database_path);

        hide();
    }
}

void NewProjectWidget::SelectImagePath() {
    image_path_text_->setText(QFileDialog::getExistingDirectory(
            this, tr("Select image path..."), DefaultDirectory(),
            QFileDialog::ShowDirsOnly));
}

QString NewProjectWidget::DefaultDirectory() {
    std::string directory_path = "";
    if (!prev_selected_ && !options_->project_path->empty()) {
        const boost::filesystem::path parent_path =
                boost::filesystem::path(*options_->project_path).parent_path();
        if (boost::filesystem::is_directory(parent_path)) {
            directory_path = parent_path.string();
        }
    }
    prev_selected_ = true;
    return QString::fromStdString(directory_path);
}


MatchesTab::MatchesTab(QWidget* parent, OptionManager* options,
                       Database* database)
        : QWidget(parent),
          options_(options),
          database_(database),
          matches_viewer_(new MatchesImageViewerWidget(parent)) {}

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

DatabaseManagementWidget::DatabaseManagementWidget(QWidget* parent,
                                                   OptionManager* options)
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


const size_t ModelManagerWidget::kNewestModelIdx =
        std::numeric_limits<size_t>::max();

ModelManagerWidget::ModelManagerWidget(QWidget* parent) : QComboBox(parent) {
    QFont font;
    font.setPointSize(10);
    setFont(font);
}

size_t ModelManagerWidget::ModelIdx() const {
    if (model_idxs_.empty()) {
        return kNewestModelIdx;
    } else {
        return model_idxs_[currentIndex()];
    }
}

void ModelManagerWidget::SetModelIdx(const size_t idx) {
    for (size_t i = 0; i < model_idxs_.size(); ++i) {
        if (model_idxs_[i] == idx) {
            blockSignals(true);
            setCurrentIndex(i);
            blockSignals(false);
        }
    }
}

void ModelManagerWidget::UpdateModels(
        const std::vector<std::unique_ptr<Reconstruction>>& models) {
    if (view()->isVisible()) {
        return;
    }

    blockSignals(true);

    const int prev_idx = currentIndex();
    const size_t prev_num_models = model_idxs_.size();

    clear();
    model_idxs_.clear();

    addItem("Newest model");
    model_idxs_.push_back(ModelManagerWidget::kNewestModelIdx);

    int max_width = 0;
    QFontMetrics font_metrics(view()->font());

    for (size_t i = 0; i < models.size(); ++i) {
        const QString item = QString().sprintf(
                "Model %d (%d images, %d points)", static_cast<int>(i + 1),
                static_cast<int>(models[i]->NumRegImages()),
                static_cast<int>(models[i]->NumPoints3D()));
        max_width = std::max(max_width, font_metrics.width(item));
        addItem(item);
        model_idxs_.push_back(i);
    }

    view()->setMinimumWidth(max_width);

    if (prev_num_models <= 0 || models.size() == 0) {
        setCurrentIndex(0);
    } else {
        setCurrentIndex(prev_idx);
    }
    blockSignals(false);
}


MainWindow::MainWindow(const std::string& binary_path)
        : options_(),
          binary_path_(binary_path),
          working_directory_(),
          import_watcher_(nullptr),
          export_watcher_(nullptr),
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

    database_management_widget_ = new DatabaseManagementWidget(this, &options_);
    model_manager_widget_ = new ModelManagerWidget(this);
}

void MainWindow::CreateActions() {
    action_new_project_ =
            new QAction(QIcon(":/media/project-new.png"), tr("New project"), this);
    action_new_project_->setShortcuts(QKeySequence::New);
    connect(action_new_project_, &QAction::triggered, this,
            &MainWindow::NewProject);

    action_import_ =
            new QAction(QIcon(":/media/import.png"), tr("Import model"), this);
    connect(action_import_, &QAction::triggered, this, &MainWindow::Import);
    blocking_actions_.push_back(action_import_);

    action_export_ =
            new QAction(QIcon(":/media/export.png"), tr("Export model"), this);
    connect(action_export_, &QAction::triggered, this, &MainWindow::Export);
    blocking_actions_.push_back(action_export_);

    action_quit_ = new QAction(tr("Quit"), this);
    connect(action_quit_, &QAction::triggered, this, &MainWindow::close);

    action_feature_extraction_ = new QAction(
            QIcon(":/media/feature-extraction.png"), tr("Extract features"), this);
    connect(action_feature_extraction_, &QAction::triggered, this,
            &MainWindow::FeatureExtraction);
    blocking_actions_.push_back(action_feature_extraction_);

    action_feature_matching_ = new QAction(QIcon(":/media/feature-matching.png"),
                                           tr("Match features"), this);
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
    connect(action_render_now_, &QAction::triggered, this, &MainWindow::RenderNow,
            Qt::BlockingQueuedConnection);

    action_reconstruction_finish_ =
            new QAction(tr("Finish reconstruction"), this);
    connect(action_reconstruction_finish_, &QAction::triggered, this,
            &MainWindow::ReconstructionFinish, Qt::BlockingQueuedConnection);
}

void MainWindow::CreateToolbar() {
    file_toolbar_ = addToolBar(tr("File"));
    file_toolbar_->addAction(action_new_project_);
    file_toolbar_->addAction(action_import_);
    file_toolbar_->addAction(action_export_);
    file_toolbar_->setIconSize(QSize(16, 16));

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
    font.setPointSize(11);

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
    import_watcher_ = new QFutureWatcher<void>(this);
    connect(import_watcher_, &QFutureWatcher<void>::finished, this,
            &MainWindow::ImportFinished);

    export_watcher_ = new QFutureWatcher<void>(this);
    connect(export_watcher_, &QFutureWatcher<void>::finished, this,
            &MainWindow::ExportFinished);
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

void MainWindow::Import() {
    if (!OverwriteReconstruction()) {
        return;
    }

    std::string path =
            QFileDialog::getOpenFileName(this, tr("Select source..."), "")
                    .toUtf8()
                    .constData();

    if (path == "") {
        return;
    }

    if (!boost::filesystem::is_regular_file(path)) {
        QMessageBox::critical(this, "", tr("Invalid file"));
        return;
    }

    if (!HasFileExtension(path, ".ply")) {
        QMessageBox::critical(this, "",
                              tr("Invalid file format (supported formats: PLY)"));
        return;
    }

    progress_bar_->setLabelText(tr("Importing model"));
    progress_bar_->raise();
    progress_bar_->show();

    import_watcher_->setFuture(QtConcurrent::run([this, path]() {
        const size_t model_idx = this->mapper_controller->AddModel();
        this->mapper_controller->Model(model_idx).ImportPLY(path, false);
        this->options_.render_options->min_track_len = 0;
        model_manager_widget_->UpdateModels(mapper_controller->Models());
        model_manager_widget_->SetModelIdx(model_idx);
    }));
}

void MainWindow::ImportFinished() {
    RenderSelectedModel();
    progress_bar_->hide();
}

void MainWindow::Export() {
    if (!IsSelectedModelValid()) {
        return;
    }

    QString default_filter("PLY (*.ply)");
    const std::string path =
            QFileDialog::getSaveFileName(
                    this, tr("Select project file"), "",
                    "PLY (*.ply)",
                    &default_filter)
                    .toUtf8()
                    .constData();

    if (path == "") {
        return;
    }

    progress_bar_->setLabelText(tr("Exporting model"));
    progress_bar_->raise();
    progress_bar_->show();

    export_watcher_->setFuture(QtConcurrent::run([this, path, default_filter]() {
        const Reconstruction& model = mapper_controller->Model(SelectedModelIdx());
        try {
            model.ExportPLY(path);
        } catch (std::domain_error& error) {
            std::cerr << "ERROR: " << error.what() << std::endl;
        }
    }));
}

void MainWindow::ExportFinished() { progress_bar_->hide(); }

void MainWindow::FeatureExtraction() {
    if (options_.Check()) {
        FeatureExtractor* feature_extractor = new SiftGPUFeatureExtractor(
                options_.extraction_options->Options(),
                options_.extraction_options->sift_options, *options_.database_path,
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
        QMessageBox::critical(this, "",
                              tr("Reset reconstruction before starting."));
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

    boost::filesystem::path output_path = boost::filesystem::path(*options_.database_path).parent_path();
    ImageDensifier* densifier = new ImageDensifier(mapper_controller->Model(0), *options_.image_path,
                                                   output_path.string(), binary_path_);
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
                        const std::string path = ((output_path / boost::filesystem::path("pmvs/models"))
                                                  / boost::filesystem::path(option + ".ply")).string();
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

void MainWindow::SurfaceReconstructModel() {
    boost::filesystem::path output_path = boost::filesystem::path(*options_.database_path).parent_path()
                                          / boost::filesystem::path("pmvs/models");
    SurfaceReconstructer* reconstructer = new SurfaceReconstructer(output_path.string(), binary_path_);
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
                    const std::string path = (output_path / boost::filesystem::path("output.ply")).string();
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
