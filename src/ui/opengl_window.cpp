#include "opengl_window.h"

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
        QRgb* pixmap_line = (QRgb*) pixmap.scanLine(y);
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
            uint* p = (uint*) image.bits();
            uint* end = p + image.width() * image.height();
            while (p < end) {
                uint a = *p << 24;
                *p = (*p >> 8) | a;
                p++;
            }
        } else {
            for (int y = 0; y < image.height(); y++) {
                uint* q = (uint*) image.scanLine(y);
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
          max_q(1.0f) { }

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
        std::vector<image_t>& reg_image_ids) { }

Eigen::Vector3f PointColormapPhotometric::ComputeColor(
        const point3D_t point3D_id, const Point_3D& point3D) {
    return Eigen::Vector3f(point3D.Color(0) / 255.0f, point3D.Color(1) / 255.0f,
                           point3D.Color(2) / 255.0f);
}


PointPainter::PointPainter() : num_geoms_(0) { }

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

    glDrawArrays(GL_POINTS, 0, (GLsizei) num_geoms_);

    // Make sure the VAO is not changed from the outside
    vao_.release();

#if DEBUG
    glDebugLog();
#endif
}


LinePainter::LinePainter() : num_geoms_(0) { }

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

    glDrawArrays(GL_LINES, 0, (GLsizei) (2 * num_geoms_));

    vao_.release();

#if DEBUG
    glDebugLog();
#endif
}


TrianglePainter::TrianglePainter() : num_geoms_(0) { }

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

    glDrawArrays(GL_TRIANGLES, 0, (GLsizei) (3 * num_geoms_));

    vao_.release();

#if DEBUG
    glDebugLog();
#endif
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