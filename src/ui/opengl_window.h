#ifndef INC_3D_RECONSTRUCTION_OPENGL_WINDOW_H
#define INC_3D_RECONSTRUCTION_OPENGL_WINDOW_H

#include "../processor/incremental_mapper_controller.h"

#include "db_widgets/database_management_widget.h"
#include "db_widgets/point_viewer_widget.h"
#include "db_widgets/image_viewer_widget.h"

#include <QtWidgets>
#include <QtConcurrent/QtConcurrentRun>

class PointViewerWidget;
class ImageViewerWidget;

Eigen::Matrix4f QMatrixToEigen(const QMatrix4x4& matrix);

QMatrix4x4 EigenToQMatrix(const Eigen::Matrix4f& matrix);

QImage BitmapToQImageRGB(const Bitmap& bitmap);

void DrawKeypoints(QPixmap* image, const FeatureKeypoints& points,
                   const QColor& color = Qt::red);

QPixmap ShowImagesSideBySide(const QPixmap& image1, const QPixmap& image2);

QPixmap DrawMatches(const QPixmap& image1, const QPixmap& image2,
                    const FeatureKeypoints& points1,
                    const FeatureKeypoints& points2,
                    const FeatureMatches& matches,
                    const QColor& keypoints_color = Qt::red);

class PointColormapBase {
public:
    PointColormapBase();

    virtual void Prepare(std::unordered_map<camera_t, Camera>& cameras,
                         std::unordered_map<image_t, Image>& images,
                         std::unordered_map<point3D_t, Point_3D>& points3D,
                         std::vector<image_t>& reg_image_ids) = 0;

    virtual Eigen::Vector3f ComputeColor(const point3D_t point3D_id,
                                         const Point_3D& point3D) = 0;

    void UpdateScale(std::vector<float>* values);

    float AdjustScale(const float gray);

    float scale;
    float min;
    float max;
    float range;
    float min_q;
    float max_q;
};


class PointPainter {
public:
    PointPainter();

    ~PointPainter();

    struct Data {
        Data() : x(0), y(0), z(0), r(0), g(0), b(0), a(0) { }

        Data(const float x_, const float y_, const float z_, const float r_,
             const float g_, const float b_, const float a_)
                : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_), a(a_) { }

        float x, y, z;
        float r, g, b, a;
    };

    void Setup();

    void Upload(const std::vector<PointPainter::Data>& data);

    void Render(const QMatrix4x4& pmv_matrix, const float point_size);

private:
    QOpenGLShaderProgram shader_program_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;

    size_t num_geoms_;
};


class LinePainter {
public:
    LinePainter();

    ~LinePainter();

    struct Data {
        Data() { }

        Data(const PointPainter::Data& p1, const PointPainter::Data& p2)
                : point1(p1), point2(p2) { }

        PointPainter::Data point1;
        PointPainter::Data point2;
    };

    void Setup();

    void Upload(const std::vector<LinePainter::Data>& data);

    void Render(const QMatrix4x4& pmv_matrix, const int width, const int height,
                const float line_width);

private:
    QOpenGLShaderProgram shader_program_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;

    size_t num_geoms_;
};


class PointColormapPhotometric : public PointColormapBase {
public:
    void Prepare(std::unordered_map<camera_t, Camera>& cameras,
                 std::unordered_map<image_t, Image>& images,
                 std::unordered_map<point3D_t, Point_3D>& points3D,
                 std::vector<image_t>& reg_image_ids);

    Eigen::Vector3f ComputeColor(const point3D_t point3D_id,
                                 const Point_3D& point3D);
};


class TrianglePainter {
public:
    TrianglePainter();

    ~TrianglePainter();

    struct Data {
        Data() { }

        Data(const PointPainter::Data& p1, const PointPainter::Data& p2,
             const PointPainter::Data& p3)
                : point1(p1), point2(p2), point3(p3) { }

        PointPainter::Data point1;
        PointPainter::Data point2;
        PointPainter::Data point3;
    };

    void Setup();

    void Upload(const std::vector<TrianglePainter::Data>& data);

    void Render(const QMatrix4x4& pmv_matrix);

private:
    QOpenGLShaderProgram shader_program_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;

    size_t num_geoms_;
};

class OpenGLWindow : public QWindow {
public:
    enum class ProjectionType {
        PERSPECTIVE,
        ORTHOGRAPHIC,
    };

    const float kInitNearPlane = 1.0f;
    const float kMinNearPlane = 1e-3f;
    const float kMaxNearPlane = 1e5f;
    const float kNearPlaneScaleSpeed = 0.02f;
    const float kFarPlane = 1e5f;
    const float kInitFocusDistance = 100.0f;
    const float kMinFocusDistance = 1e-5f;
    const float kMaxFocusDistance = 1e8f;
    const float kFieldOfView = 25.0f;
    const float kFocusSpeed = 2.0f;
    const float kInitPointSize = 1.0f;
    const float kMinPointSize = 0.5f;
    const float kMaxPointSize = 100.0f;
    const float kPointScaleSpeed = 0.1f;
    const float kInitImageSize = 0.2f;
    const float kMinImageSize = 1e-6f;
    const float kMaxImageSize = 1e3f;
    const float kImageScaleSpeed = 0.1f;
    const int kDoubleClickInterval = 250;

    OpenGLWindow(QWidget* parent, OptionManager* options, QScreen* screen = 0);

    void Update();

    void Upload();

    void Clear();

    ProjectionType GetProjectionType() const;

    void SetProjectionType(const ProjectionType type);

    void SetPointColormap(PointColormapBase* colormap);

    void EnableCoordinateGrid();

    void DisableCoordinateGrid();

    void ChangeFocusDistance(const float delta);

    void ChangeNearPlane(const float delta);

    void ChangePointSize(const float delta);

    void ChangeImageSize(const float delta);

    void RotateView(const float x, const float y, const float prev_x,
                    const float prev_y);

    void TranslateView(const float x, const float y, const float prev_x,
                       const float prev_y);

    void ResetView();

    QMatrix4x4 ModelViewMatrix() const;

    void SetModelViewMatrix(const QMatrix4x4& matrix);

    void SelectObject(const int x, const int y);

    QImage GrabImage();

    void ShowPointInfo(const point3D_t point3D_id);

    void ShowImageInfo(const image_t image_id);

    float PointSize() const;

    float ImageSize() const;

    void SetPointSize(const float point_size);

    void SetImageSize(const float image_size);

    void SetBackgroundColor(const float r, const float g, const float b);

    Reconstruction* reconstruction;
    std::unordered_map<camera_t, Camera> cameras;
    std::unordered_map<image_t, Image> images;
    std::unordered_map<point3D_t, Point_3D> points3D;
    std::vector<image_t> reg_image_ids;

    QLabel* statusbar_status_label;

private:
    void exposeEvent(QExposeEvent* event);

    void mousePressEvent(QMouseEvent* event);

    void mouseReleaseEvent(QMouseEvent* event);

    void mouseMoveEvent(QMouseEvent* event);

    void wheelEvent(QWheelEvent* event);

    void SetupGL();

    void InitializeGL();

    void ResizeGL();

    void PaintGL();

    void SetupPainters();

    void InitializeSettings();

    void InitializeView();

    void UploadCoordinateGridData();

    void UploadPointData(const bool selection_mode = false);

    void UploadPointConnectionData();

    void UploadImageData(const bool selection_mode = false);

    void UploadImageConnectionData();

    void ComposeProjectionMatrix();

    float ZoomScale() const;

    float AspectRatio() const;

    float OrthographicWindowExtent() const;

    Eigen::Vector4ub ReadPixelColor(int x, int y) const;

    Eigen::Vector3f PositionToArcballVector(const float x, const float y) const;

    OptionManager* options_;
    QOpenGLContext* context_;

    QMatrix4x4 model_view_matrix_;
    QMatrix4x4 projection_matrix_;

    LinePainter coordinate_axes_painter_;
    LinePainter coordinate_grid_painter_;

    PointPainter point_painter_;
    LinePainter point_connection_painter_;

    LinePainter image_line_painter_;
    TrianglePainter image_triangle_painter_;
    LinePainter image_connection_painter_;

    PointViewerWidget* point_viewer_widget_;
    ImageViewerWidget* image_viewer_widget_;

    ProjectionType projection_type_;

    std::unique_ptr<PointColormapBase> point_colormap_;

    bool mouse_is_pressed_;
    QTimer mouse_press_timer_;
    QPoint prev_mouse_pos_;

    float focus_distance_;

    std::vector<std::pair<size_t, char>> selection_buffer_;
    image_t selected_image_id_;
    point3D_t selected_point3D_id_;

    bool coordinate_grid_enabled_;

    float point_size_;
    float image_size_;
    float near_plane_;

    float bg_color_[3];
};

#endif //INC_3D_RECONSTRUCTION_OPENGL_WINDOW_H
