#include "ba_experiments.h"

// From CERES BA Examples.
// Allows to count residual for pinhole camera. Not very useful for us.
struct SnavelyReprojectionError {
    SnavelyReprojectionError(double observed_x, double observed_y)
            : observed_x(observed_x), observed_y(observed_y) {}

    template<typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];
        // Apply second and fourth order radial distortion.
        const T& l1 = camera[7];
        const T& l2 = camera[8];
        T r2 = xp * xp + yp * yp;
        T distortion = T(1.0) + r2 * (l1 + l2 * r2);
        // Compute final projected point position.
        const T& focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)));
    }

    double observed_x;
    double observed_y;
};

BAExperiments::BAExperiments(const OptionManager& options) : options_(options), database_cache_(new DatabaseCache()) {}

void BAExperiments::RunSimpleCeresImplementation() {

    DroneData data(options_);
    data.Read();
    return;

    ceres::Problem problem;
    BuildCeresProblem(problem);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

void BAExperiments::LoadDatabaseCache() {
    std::cout << "Loading database cache... ";
    Database database;
    database.Open(*options_.database_path);
    const size_t min_num_matches = static_cast<size_t>(options_.mapper_options->min_num_matches);
    database_cache_->Load(database, min_num_matches, options_.mapper_options->ignore_watermarks);
    std::cout << "finished successfully." << std::endl;
}

void BAExperiments::BuildCeresProblem(ceres::Problem& problem) {
    std::cout << "Building CERES problem..." << std::endl;
    LoadDatabaseCache();

    std::cout << "Retrieving data from database..." << std::endl;
    int num_images = (int) database_cache_->NumImages();
    std::cout << "Num images: " << num_images << std::endl;
    int num_cameras = (int) database_cache_->NumCameras();
    std::cout << "Num cameras: " << num_cameras << std::endl;

    auto images = database_cache_->Images();
    std::cout << "Real number of images retrieved: " << images.size() << std::endl;
    for (auto image_pair : images) {
        Image image = image_pair.second;
        std::cout << "Processing with image " << image.ImageId() << " :" << std::endl;
        std::cout << "   Image has camera: " << image.HasCamera() << std::endl;
        std::cout << "   Num points 3d: " << image.NumPoints3D() << std::endl;
        std::cout << "   Number of observations: " << image.NumPoints2D() << " "
                  << image.Points2D().size() << std::endl;

        Camera camera = database_cache_->Camera(image.CameraId());
        std::cout << "   Camera: " << camera.CameraId() << std::endl;
        if (camera.ModelName() != "RADIAL") {
            std::cout << "!! NOT RADIAL" << std::endl;
            continue;
        }
        // Processing with every image and every camera.
        auto observations = image.Points2D();
        for (auto obs : observations) {
            ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(obs.X(), obs.Y());
            // actually this function is for pinhole camera.
            // where (quoting CERES):
            // Templated pinhole camera model for used with Ceres.  The camera is
            // parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
            // focal length and 2 for radial distortion. The principal point is not modeled
            // (i.e. it is assumed be located at the image center).
            double camera_params[9] = {1., 1., 1., 1., 1., 1., 1., 1., 1.};
            // the camera is radial.
            // camera.Params() are "f, cx, cy, k1, k2".
            camera_params[6] = camera.Params()[0];


            double point3d_approx[3] = {0., 0., 0.};
            problem.AddResidualBlock(cost_function,
                                     NULL,
                                     camera_params,
                                     point3d_approx);
        }
    }
    std::cout << std::endl << "PROBLEM IS BUILT" << std::endl;
}

DroneData::DroneData(const OptionManager& options) : options_(options) {}

bool DroneData::Read() {
    std::string filename = EnsureTrailingSlash(*options_.image_path) + DroneData::DRONE_DATA_FILE;
    std::cout << "reading from " << filename << std::endl;
    std::ifstream file(filename);
    std::string camera_inner_params;

    std::getline(file, camera_inner_params);
    std::cout << "LINE READ " << camera_inner_params << std::endl;

    std::stringstream line_stream(camera_inner_params);

    std::string item;

    std::vector<std::string> items;
    while (!line_stream.eof()) {
        std::getline(line_stream, item, ' ');
        items.push_back(item);
    }
    std::cout << "camera inner read: " << items.size() << std::endl << "Camera inner: ";
    for (auto i: items) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::getline(file, item);
    std::cout << "num images: " << item << std::endl;

    for (int i = 0; i < 1; i++) {
        std::string image_line;
        std::getline(file, image_line);
        std::vector<std::string> params;
        std::stringstream image_stream(image_line);
        while (!image_stream.eof()) {
            std::getline(image_stream, item, ' ');
            params.push_back(item);
        }
        std::cout << "image num params: " << params.size() << std::endl << "  params themselves: ";
        for (auto p : params) {
            std::cout << p << " ";
        }
        std::cout << std::endl;

    }
    file.close();
    return true;
}