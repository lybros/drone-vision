#include "densify.h"

namespace {
    void WriteProjectionMatrix(const std::string& path, const Camera& camera,
                               const Image& image, const std::string& header) {
        std::ofstream file;
        file.open(path.c_str(), std::ios::trunc);

        Eigen::Matrix3d calib_matrix = Eigen::Matrix3d::Identity();
        calib_matrix(0, 0) = camera.FocalLengthX();
        calib_matrix(1, 1) = camera.FocalLengthY();
        calib_matrix(0, 2) = camera.PrincipalPointX();
        calib_matrix(1, 2) = camera.PrincipalPointY();

        const Eigen::Matrix3x4d proj_matrix = calib_matrix * image.ProjectionMatrix();

        if (!header.empty()) {
            file << header << std::endl;
        }

        file << proj_matrix(0, 0) << " ";
        file << proj_matrix(0, 1) << " ";
        file << proj_matrix(0, 2) << " ";
        file << proj_matrix(0, 3) << std::endl;

        file << proj_matrix(1, 0) << " ";
        file << proj_matrix(1, 1) << " ";
        file << proj_matrix(1, 2) << " ";
        file << proj_matrix(1, 3) << std::endl;

        file << proj_matrix(2, 0) << " ";
        file << proj_matrix(2, 1) << " ";
        file << proj_matrix(2, 2) << " ";
        file << proj_matrix(2, 3) << std::endl;

        file.close();
    }
}

ImageDensifier::ImageDensifier(const Reconstruction& reconstruction, const std::string& image_path,
                               const std::string& output_path, const std::string& binary_path)
        : stop_(false),
          image_path_(EnsureTrailingSlash(image_path)),
          output_path_(output_path),
          binary_path_(binary_path),
          results_(),
          successfull_(true),
          reconstruction_(reconstruction) {}

void ImageDensifier::Stop() {
    QMutexLocker locker(&mutex_);
    stop_ = true;
}

void ImageDensifier::run() {
    Timer total_timer;
    total_timer.Start();

    PrintHeading1("Image undistortion for CMVS/PMVS");

    namespace fs = boost::filesystem;

    const fs::path output_path(output_path_);
    const fs::path pmvs_path(output_path / fs::path("pmvs"));
    const fs::path txt_path(pmvs_path / fs::path("txt"));
    const fs::path visualize_path(pmvs_path / fs::path("visualize"));
    const fs::path models_path(pmvs_path / fs::path("models"));
    const fs::path bundle_path(pmvs_path / fs::path("bundle.rd.out"));
    const fs::path vis_path(pmvs_path / fs::path("vis.dat"));
    const fs::path option_path(pmvs_path / fs::path("option-all"));

    if (!fs::is_directory(output_path)) {
        fs::create_directory(output_path);
    }
    if (!fs::is_directory(pmvs_path)) {
        fs::create_directory(pmvs_path);
    }
    if (!fs::is_directory(txt_path)) {
        fs::create_directory(txt_path);
    }
    if (!fs::is_directory(visualize_path)) {
        fs::create_directory(visualize_path);
    }
    if (!fs::is_directory(models_path)) {
        fs::create_directory(models_path);
    }

    Reconstruction undistorted_reconstruction = reconstruction_;

    ThreadPool thread_pool;
    std::vector<std::future<size_t>> futures;

    const std::vector<image_t>& reg_image_ids = reconstruction_.RegImageIds();
    for (size_t i = 0; i < reg_image_ids.size(); ++i) {
        const image_t image_id = reg_image_ids[i];
        const Image& image = reconstruction_.Image(image_id);
        const Camera& camera = reconstruction_.Camera(image.CameraId());

        const std::string input_image_path = image_path_ + image.Name();
        const std::string output_image_path =
                (visualize_path / fs::path((boost::format("%08d.jpg") % i).str()))
                        .string();
        const std::string proj_matrix_path =
                (txt_path / fs::path((boost::format("%08d.txt") % i).str())).string();

        std::function<size_t(void)> UndistortFunc = [=]() {
            if (fs::exists(output_image_path) && fs::exists(proj_matrix_path)) {
                std::cout << boost::format("SKIP: Already undistorted [%d/%d]") %
                             (i + 1) % reg_image_ids.size()
                << std::endl;
                return i;
            }
            Bitmap distorted_bitmap;

            if (!distorted_bitmap.Read(input_image_path, true)) {
                std::cerr << "ERROR: Cannot read image at path " << input_image_path
                << std::endl;
                return i;
            }

            Bitmap undistorted_bitmap;
            Camera undistorted_camera;
            UndistortImage(distorted_bitmap, camera, &undistorted_bitmap,
                           &undistorted_camera);

            undistorted_bitmap.Write(output_image_path);
            WriteProjectionMatrix(proj_matrix_path, undistorted_camera, image,
                                  "CONTOUR");

            return i;
        };

        futures.push_back(thread_pool.AddTask(UndistortFunc));
    }

    for (auto& future : futures) {
        {
            QMutexLocker locker(&mutex_);
            if (stop_) {
                thread_pool.Stop();
                std::cout << "WARNING: Stopped the undistortion process. Image point "
                        "locations and camera parameters for not yet processed "
                        "images in the Bundler output file is probably wrong."
                << std::endl;
                break;
            }
        }

        const size_t i = future.get();

        const image_t image_id = reg_image_ids[i];
        const Image& image = reconstruction_.Image(image_id);
        const Camera& camera = reconstruction_.Camera(image.CameraId());
        Camera& undistorted_camera =
                undistorted_reconstruction.Camera(image.CameraId());
        undistorted_camera = UndistortCamera(camera);

        Image& undistorted_bitmap =
                undistorted_reconstruction.Image(image.ImageId());
        for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
             ++point2D_idx) {
            Point2D& point2D = undistorted_bitmap.Point2D(point2D_idx);
            const Eigen::Vector2d world_point = camera.ImageToWorld(point2D.XY());
            point2D.SetXY(undistorted_camera.WorldToImage(world_point));
        }

        std::cout << boost::format("Undistorting image [%d/%d]") % (i + 1) %
                     reg_image_ids.size()
        << std::endl;
    }

    std::cout << "Writing bundle file" << std::endl;
    try {
        undistorted_reconstruction.ExportBundler(
                bundle_path.string(), bundle_path.string() + ".list.txt");
    } catch (std::domain_error& error) {
        std::cerr << "WARNING: " << error.what() << std::endl;
    }

    std::cout << "Started densifying" << std::endl;

    QString program = QString::fromStdString(binary_path_);
    QStringList arguments;
    std::stringstream stream;
    stream << std::thread::hardware_concurrency();
    arguments << "cmvs" << EnsureTrailingSlash(pmvs_path.string()).c_str() << "70" << stream.str().c_str();
    QProcess* cmvs_process = new QProcess;
    std::cout << "Running " << program.toUtf8().constData() << " with mode cmvs" << std::endl;
    WaitForProcess(cmvs_process, program, arguments);

    {
        QMutexLocker locker(&mutex_);
        if (stop_ || !successfull_) {
            std::cout << "Stopping densifying" << std::endl;
            return;
        }
    }

    std::ifstream ifstr;
    ifstr.open((pmvs_path / fs::path("ske.dat")).string());

    std::string header;
    int inum, cnum;
    ifstr >> header >> inum >> cnum;

    for (int c = 0; c < cnum; ++c) {
        {
            QMutexLocker locker(&mutex_);
            if (stop_ || !successfull_) {
                std::cout << "Stopping densifying" << std::endl;
                return;
            }
        }

        std::ofstream ofstr;
        std::string option = (boost::format("option-%04d") % c).str();
        ofstr.open((pmvs_path/ fs::path(option)).string().c_str());
        results_.push_back(option);

        ofstr << "level 1" << std::endl;
        ofstr << "csize 2" << std::endl;
        ofstr << "threshold 0.7" << std::endl;
        ofstr << "wsize 7" << std::endl;
        ofstr << "minImageNum 3" << std::endl;
        ofstr << "CPU " << std::thread::hardware_concurrency() << std::endl;
        ofstr << "setEdge 0" << std::endl;
        ofstr << "useBound 0" << std::endl;
        ofstr << "useVisData 1" << std::endl;
        ofstr << "sequence -1" << std::endl;
        ofstr << "maxAngle 10" << std::endl;
        ofstr << "quad 2.0" << std::endl;

        int timagenum, oimagenum;
        ifstr >> timagenum >> oimagenum;

        std::vector<int> timages, oimages;
        timages.resize(timagenum);
        oimages.resize(oimagenum);
        for (int i = 0; i < timagenum; ++i)
            ifstr >> timages[i];
        for (int i = 0; i < oimagenum; ++i)
            ifstr >> oimages[i];

        ofstr << "timages " << timagenum << ' ';
        for (int i = 0; i < timagenum; ++i)
            ofstr << timages[i] << ' ';
        ofstr << std::endl;
        ofstr << "oimages " << oimagenum << ' ';
        for (int i = 0; i < oimagenum; ++i)
            ofstr << oimages[i] << ' ';
        ofstr << std::endl;
        ofstr.close();

        QString program = QString::fromStdString(binary_path_);
        QStringList arguments;
        arguments << "pmvs" << EnsureTrailingSlash(pmvs_path.string()).c_str() << (boost::format("option-%04d") % c).str().c_str();
        QProcess* pmvs_process = new QProcess;
        std::cout << "Running " << program.toUtf8().constData() << " with mode pmvs" << std::endl;
        WaitForProcess(pmvs_process, program, arguments);
    }

    total_timer.PrintMinutes();
}

bool ImageDensifier::IsSuccessfull() {
    QMutexLocker locker(&mutex_);
    return successfull_;
}

bool ImageDensifier::IsRunning() {
    QMutexLocker locker(&mutex_);
    return !stop_;
}

void ImageDensifier::WaitForProcess(QProcess *child, const QString &program, const QStringList &params) {
    child->setProcessChannelMode(QProcess::ForwardedChannels);
    child->start(program, params);
    while (true) {
        {
            QMutexLocker locker(&mutex_);
            if (stop_) {
                std::cout << "Stopping densifying" << std::endl;
                child->terminate();
                break;
            }
        }
        if (child->waitForFinished(5000)) {
            if (child->exitCode() == 0) {
                QMutexLocker locker(&mutex_);
            }
            else {
                successfull_ = false;
                std::cerr << "Process exited with exit code " << child->exitCode() << std::endl;
            }
            break;
        }
    }
}

std::vector<std::string> ImageDensifier::ResultFiles() const {
    return results_;
}


Camera UndistortCamera(const Camera& camera) {
    Camera undistorted_camera;
    undistorted_camera.SetModelId(PinholeCameraModel::model_id);
    undistorted_camera.Params().resize(PinholeCameraModel::num_params);
    undistorted_camera.SetWidth(camera.Width());
    undistorted_camera.SetHeight(camera.Height());

    const std::vector<size_t>& focal_length_idxs = camera.FocalLengthIdxs();
    if (focal_length_idxs.size() == 1) {
        undistorted_camera.SetFocalLengthX(camera.FocalLength());
        undistorted_camera.SetFocalLengthY(camera.FocalLength());
    } else if (focal_length_idxs.size() == 2) {
        undistorted_camera.SetFocalLengthX(camera.FocalLengthX());
        undistorted_camera.SetFocalLengthY(camera.FocalLengthY());
    }

    undistorted_camera.SetPrincipalPointX(camera.PrincipalPointX());
    undistorted_camera.SetPrincipalPointY(camera.PrincipalPointY());

    double left_min_x = std::numeric_limits<double>::max();
    double left_max_x = std::numeric_limits<double>::lowest();
    double right_min_x = std::numeric_limits<double>::max();
    double right_max_x = std::numeric_limits<double>::lowest();

    for (size_t y = 0; y < camera.Height(); ++y) {
        const Eigen::Vector2d world_point1 =
                camera.ImageToWorld(Eigen::Vector2d(0.5, y + 0.5));
        const Eigen::Vector2d undistorted_point1 =
                undistorted_camera.WorldToImage(world_point1);
        left_min_x = std::min(left_min_x, undistorted_point1(0));
        left_max_x = std::max(left_max_x, undistorted_point1(0));
        const Eigen::Vector2d world_point2 =
                camera.ImageToWorld(Eigen::Vector2d(camera.Width() - 0.5, y + 0.5));
        const Eigen::Vector2d undistorted_point2 =
                undistorted_camera.WorldToImage(world_point2);
        right_min_x = std::min(right_min_x, undistorted_point2(0));
        right_max_x = std::max(right_max_x, undistorted_point2(0));
    }

    double top_min_y = std::numeric_limits<double>::max();
    double top_max_y = std::numeric_limits<double>::lowest();
    double bottom_min_y = std::numeric_limits<double>::max();
    double bottom_max_y = std::numeric_limits<double>::lowest();

    for (size_t x = 0; x < camera.Width(); ++x) {
        const Eigen::Vector2d world_point1 =
                camera.ImageToWorld(Eigen::Vector2d(x + 0.5, 0.5));
        const Eigen::Vector2d undistorted_point1 =
                undistorted_camera.WorldToImage(world_point1);
        top_min_y = std::min(top_min_y, undistorted_point1(1));
        top_max_y = std::max(top_max_y, undistorted_point1(1));
        const Eigen::Vector2d world_point2 =
                camera.ImageToWorld(Eigen::Vector2d(x + 0.5, camera.Height() - 0.5));
        const Eigen::Vector2d undistorted_point2 =
                undistorted_camera.WorldToImage(world_point2);
        bottom_min_y = std::min(bottom_min_y, undistorted_point2(1));
        bottom_max_y = std::max(bottom_max_y, undistorted_point2(1));
    }

    const double cx = undistorted_camera.PrincipalPointX();
    const double cy = undistorted_camera.PrincipalPointY();

    const double min_scale_x = std::min(
            cx / (cx - left_min_x), (camera.Width() - 0.5 - cx) / (right_max_x - cx));
    const double min_scale_y =
            std::min(cy / (cy - top_min_y),
                     (camera.Height() - 0.5 - cy) / (bottom_max_y - cy));

    const double max_scale_x = std::max(
            cx / (cx - left_max_x), (camera.Width() - 0.5 - cx) / (right_min_x - cx));
    const double max_scale_y =
            std::max(cy / (cy - top_max_y),
                     (camera.Height() - 0.5 - cy) / (bottom_min_y - cy));

    double scale_x = 1.0 / max_scale_x;
    double scale_y = 1.0 / max_scale_y;

    scale_x = Clip(scale_x, 0.2, 2.);
    scale_y = Clip(scale_y, 0.2, 2.);

    undistorted_camera.SetWidth(
            static_cast<size_t>(std::max(1.0, scale_x * undistorted_camera.Width())));
    undistorted_camera.SetHeight(static_cast<size_t>(
                                         std::max(1.0, scale_y * undistorted_camera.Height())));

    undistorted_camera.SetPrincipalPointX(
            undistorted_camera.PrincipalPointX() *
            static_cast<double>(undistorted_camera.Width()) / camera.Width());
    undistorted_camera.SetPrincipalPointY(
            undistorted_camera.PrincipalPointY() *
            static_cast<double>(undistorted_camera.Height()) / camera.Height());

    return undistorted_camera;
}

void UndistortImage(const Bitmap& distorted_bitmap,
                    const Camera& distorted_camera, Bitmap* undistorted_bitmap,
                    Camera* undistorted_camera) {
    *undistorted_camera = UndistortCamera(distorted_camera);
    undistorted_bitmap->Allocate(static_cast<int>(undistorted_camera->Width()),
                                 static_cast<int>(undistorted_camera->Height()),
                                 distorted_bitmap.IsRGB());
    distorted_bitmap.CloneMetadata(undistorted_bitmap);
    undistorted_bitmap->Allocate(static_cast<int>(undistorted_camera->Width()),
                           static_cast<int>(undistorted_camera->Height()),
                           distorted_bitmap.IsRGB());
    Eigen::Vector2d image_point;
    for (int y = 0; y < undistorted_bitmap->Height(); ++y) {
        image_point.y() = y + 0.5;
        for (int x = 0; x < undistorted_bitmap->Width(); ++x) {
            image_point.x() = x + 0.5;
            const Eigen::Vector2d world_point =
                    undistorted_camera->ImageToWorld(image_point);
            const Eigen::Vector2d source_point =
                    distorted_camera.WorldToImage(world_point);

            Eigen::Vector3d color;
            if (!distorted_bitmap.InterpolateBilinear(source_point.x() - 0.5, source_point.y() - 0.5, &color)) {
                color.setZero();
            }

            color.unaryExpr(std::ptr_fun<double, double>(std::round));
            undistorted_bitmap->SetPixel(x, y, color.cast<uint8_t>());
        }
    }
}
