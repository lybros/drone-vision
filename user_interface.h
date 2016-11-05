#ifndef INC_3D_RECONSTRUCTION_USER_INTERFACE_H
#define INC_3D_RECONSTRUCTION_USER_INTERFACE_H

#include "controllers.h"
#include "densify.h"
#include "surface_reconstruct.h"

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLabel>
#include <QtWidgets/QProgressDialog>
#include <QtCore/QFutureWatcher>
#include <QtOpenGL/QtOpenGL>
#include <QtConcurrent/QtConcurrentRun>

#include <boost/filesystem.hpp>
#include <boost/random.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

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

class JetColormap {
public:
    static float Red(const float gray);
    static float Green(const float gray);
    static float Blue(const float gray);

private:
    static float Interpolate(const float val, const float y0, const float x0,
                             const float y1, const float x1);
    static float Base(const float val);
};

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
        Data() : x(0), y(0), z(0), r(0), g(0), b(0), a(0) {}
        Data(const float x_, const float y_, const float z_, const float r_,
             const float g_, const float b_, const float a_)
                : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_), a(a_) {}

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
        Data() {}
        Data(const PointPainter::Data& p1, const PointPainter::Data& p2)
                : point1(p1), point2(p2) {}

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
        Data() {}
        Data(const PointPainter::Data& p1, const PointPainter::Data& p2,
             const PointPainter::Data& p3)
                : point1(p1), point2(p2), point3(p3) {}

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


class MainWindow;

class NewProjectWidget : public QWidget {
public:
    NewProjectWidget(MainWindow* parent, OptionManager* options);

    bool IsValid();

    std::string ImagePath() const;
    void SetImagePath(const std::string& path);

private:
    void Create();
    void SelectImagePath();
    QString DefaultDirectory();

    MainWindow* main_window_;

    OptionManager* options_;

    bool prev_selected_;

    QLineEdit* image_path_text_;
};


class MatchesTab : public QWidget {
public:
    MatchesTab() {}
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


class ModelManagerWidget : public QComboBox {
public:
    const static size_t kNewestModelIdx;

    ModelManagerWidget(QWidget* parent);

    size_t ModelIdx() const;
    void SetModelIdx(const size_t idx);

    void UpdateModels(const std::vector<std::unique_ptr<Reconstruction>>& models);

private:
    std::vector<size_t> model_idxs_;
};



class MainWindow : public QMainWindow {
public:
    MainWindow(const std::string& binary_path);

    bool OverwriteReconstruction();

    std::unique_ptr<IncrementalMapperController> mapper_controller;

protected:
    void showEvent(QShowEvent* event);
    void moveEvent(QMoveEvent* event);
    void closeEvent(QCloseEvent* event);

    void afterShowEvent();

private:
    void CreateWidgets();
    void CreateActions();
    void CreateToolbar();
    void CreateStatusbar();
    void CreateControllers();
    void CreateFutures();
    void CreateProgressBar();

    void CenterProgressBar();

    void NewProject();
    void Import();
    void ImportFinished();
    void Export();
    void ExportFinished();

    void FeatureExtraction();
    void FeatureMatching();
    void DatabaseManagement();

    void ReconstructionStart();
    void ReconstructionPause();
    void ReconstructionReset();
    void ReconstructionFinish();

    void Render();
    void RenderNow();
    void RenderSelectedModel();
    void RenderClear();

    void SelectModelIdx(const size_t);
    size_t SelectedModelIdx();
    bool HasSelectedModel();
    bool IsSelectedModelValid();

    void DensifyModel();
    void SurfaceReconstructModel();

    void ShowInvalidProjectError();
    void UpdateTimer();

    void EnableBlockingActions();
    void DisableBlockingActions();

    void UpdateWindowTitle();

    OptionManager options_;

    OpenGLWindow* opengl_window_;

    Timer timer_;

    QTimer* after_show_event_timer_;

    NewProjectWidget* new_project_widget_;
    DatabaseManagementWidget* database_management_widget_;
    ModelManagerWidget* model_manager_widget_;

    QToolBar* file_toolbar_;
    QToolBar* preprocessing_toolbar_;
    QToolBar* reconstruction_toolbar_;
    QToolBar* render_toolbar_;

    QTimer* statusbar_timer_;
    QLabel* statusbar_timer_label_;

    QAction* action_new_project_;
    QAction* action_import_;
    QAction* action_export_;
    QAction* action_quit_;

    QAction* action_feature_extraction_;
    QAction* action_feature_matching_;
    QAction* action_database_management_;

    QAction* action_reconstruction_start_;
    QAction* action_reconstruction_pause_;
    QAction* action_reconstruction_reset_;
    QAction* action_reconstruction_finish_;

    QAction* action_render_;
    QAction* action_render_now_;
    QAction* action_render_reset_view_;

    QAction* action_densify_;
    QAction* action_surface_reconstruct_;

    QProgressDialog* progress_bar_;

    QFutureWatcher<void>* import_watcher_;
    QFutureWatcher<void>* export_watcher_;

    std::vector<QAction*> blocking_actions_;

    std::string binary_path_;
    std::string working_directory_;

    size_t render_counter_;

    bool window_closed_;
};

#endif //INC_3D_RECONSTRUCTION_USER_INTERFACE_H
