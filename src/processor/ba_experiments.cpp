#include "ba_experiments.h"

// From CERES BA Examples. Counts residual.
struct SnavelyReprojectionError {
    SnavelyReprojectionError(double observed_x, double observed_y)
            : observed_x(observed_x), observed_y(observed_y) {}

    // in my case T stands for double.
    template<typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3]; // vector <x y z>.
        ceres::AngleAxisRotatePoint(camera, point, p);
        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        // Compute the center of distortion.
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];
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

    drone_data_ = new DroneData(options_);
    drone_data_->Read();
    LoadDatabaseCache();
    int matched = drone_data_->MatchWithDatabase(database_cache_);
    std::cout << "matched: " << matched << std::endl;
    drone_data_->Print();

    ceres::Problem problem;
    BuildCeresProblem(problem);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    std::cout << std::endl << "3D approximations after resolving:" << std::endl;
    for (int i = 0; i < drone_data_->images.size(); i++) {
        std::cout << approx_3d[i][0] << " " << approx_3d[i][1] << " " << approx_3d[i][2] << std::endl;
    }
}

void BAExperiments::LoadDatabaseCache() {
    std::cout << "Loading database cache... " << std::endl;
    Database database;
    database.Open(*options_.database_path);
    const size_t min_num_matches = static_cast<size_t>(options_.mapper_options->min_num_matches);
    database_cache_->Load(database, min_num_matches, options_.mapper_options->ignore_watermarks);
}

void BAExperiments::BuildCeresProblem(ceres::Problem& problem) {
    std::cout << "Building CERES problem..." << std::endl;

    std::cout << "Retrieving data from database..." << std::endl;
    int num_images = (int) drone_data_->images.size();
    std::cout << "Num images: " << num_images << std::endl;
    int num_cameras = (int) database_cache_->NumCameras();
    std::cout << "Num physical cameras: 1" << std::endl;

    auto db_images = database_cache_->Images();
    auto drone_images = drone_data_->images;
    std::cout << "Real number of images retrieved: " << db_images.size() << std::endl;

    // Setting 3d points approximations.
    approx_3d = new double* [6];

    for (auto drone_image : drone_images) {
        Image db_image = database_cache_->Image(drone_image.image_db_id);
        std::cout << "Processing with image " << db_image.ImageId() << " :" << std::endl;
        std::cout << "   Image has camera: " << db_image.HasCamera() << std::endl;
        std::cout << "   Num points 3d: " << db_image.NumPoints3D() << std::endl;
        std::cout << "   Number of observations: " << db_image.NumPoints2D() << " "
                  << db_image.Points2D().size() << std::endl;

        Camera camera = database_cache_->Camera(db_image.CameraId());
        std::cout << "   Camera: " << camera.CameraId() << std::endl;
        if (camera.ModelName() != "RADIAL") {
            std::cout << "!! NOT RADIAL" << std::endl;
            continue;
        }
        // Processing with every image and every camera.
        auto observations = db_image.Points2D();
        // Templated pinhole camera model for used with Ceres.  The camera is
        // parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
        // focal length and 2 for radial distortion. The principal point is not modeled
        // (i.e. it is assumed be located at the image center).
        double camera_params[9];/* = {
                drone_image.rotation_params[0],
                drone_image.rotation_params[1],
                drone_image.rotation_params[2],
                drone_image.translation_params[0],
                drone_image.translation_params[1],
                drone_image.translation_params[2],
                drone_data_->camera_inner_params[0],
                drone_data_->camera_inner_params[1],
                drone_data_->camera_inner_params[2],
        }; */
        approx_3d[drone_image.image_db_id - 1] = new double[3]{
                0., 0., 0.,
                //drone_image.gps_data[0],
                //drone_image.gps_data[1],
                //drone_image.gps_data[2],
        };
        for (auto obs : observations) {
            ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(obs.X(), obs.Y());
            problem.AddResidualBlock(cost_function,
                                     NULL,
                                     camera_params,
                                     approx_3d[drone_image.image_db_id - 1]);
        }
    }
    std::cout << std::endl << "PROBLEM IS BUILT. 3D approximations based on GPS:" << std::endl;
    for (int i = 0; i < num_images; i++) {
        std::cout << approx_3d[i][0] << " " << approx_3d[i][1] << " " << approx_3d[i][2] << std::endl;
    }
}
