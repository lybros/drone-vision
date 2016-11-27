#include "ba_experiments.h"

// From CERES BA Examples.
// Allows to count residual for pinhole camera.
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
    approx_3d = new double*[6];

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
        double camera_params[9] = {
                drone_image.rotation_params[0],
                drone_image.rotation_params[1],
                drone_image.rotation_params[2],
                drone_image.translation_params[0],
                drone_image.translation_params[1],
                drone_image.translation_params[2],
                drone_data_->camera_inner_params[0],
                drone_data_->camera_inner_params[1],
                drone_data_->camera_inner_params[2],
        };
        approx_3d[drone_image.image_db_id - 1] = new double[3]{
                drone_image.gps_data[0],
                drone_image.gps_data[1],
                drone_image.gps_data[2],
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

DroneData::DroneData(const OptionManager& options) : options_(options) {}

bool DroneData::Read() {
    std::string filename = EnsureTrailingSlash(*options_.image_path) + DroneData::DRONE_DATA_FILE;
    std::ifstream file(filename);

    // Reading inner camera parameters.
    std::string camera_inner_params;
    std::getline(file, camera_inner_params);
    this->camera_inner_params = std::vector<double>(3);
    std::sscanf(camera_inner_params.c_str(), "%lf %lf %lf", &this->camera_inner_params[0],
                &this->camera_inner_params[1], &this->camera_inner_params[2]);

    // Reading number of images in a set.
    std::string image_number_line;
    std::getline(file, image_number_line);
    std::sscanf(image_number_line.c_str(), "%d", &num_images);
    images = std::vector<ImageData>(num_images);

    // Reading every image.
    for (int i = 0; i < num_images; i++) {
        std::string image_line;
        std::getline(file, image_line);
        images[i].rotation_params = new double[3];
        images[i].translation_params = new double[3];
        images[i].gps_data = new double[3];
        char name[100];
        std::sscanf(image_line.c_str(), "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                    name,
                    &images[i].rotation_params[0], &images[i].rotation_params[1], &images[i].rotation_params[2],
                    &images[i].translation_params[0], &images[i].translation_params[1],
                    &images[i].translation_params[2],
                    &images[i].gps_data[0], &images[i].gps_data[1], &images[i].gps_data[2]);
        images[i].image_name = std::string(name);
    }

    file.close();

    return true;
}

int DroneData::MatchWithDatabase(DatabaseCache* database_cache) {
    std::unordered_map<image_t, Image> db_images = database_cache->Images();
    int matched = 0;
    for (int i = 0; i < images.size(); i++) {
        ImageData& image = images[i];
        for (auto db_image : db_images) {
            if (db_image.second.Name() == image.image_name) {
                image.image_db_id = db_image.first;
                matched++;
                break;
            }
        }
    }
    return matched;
}

void DroneData::Print() {
    PrintHeading1("PRINTING DRONE DATA");

    std::cout << "Camera params: (" << camera_inner_params.size() << ")" << std::endl;
    for (auto param : camera_inner_params) {
        std::cout << param << " ";
    }
    std::cout << std::endl;

    std::cout << "Image params: (" << images.size() << ")" << std::endl;
    for (auto image : images) {
        std::cout << "  Image: " << image.image_name << std::endl;
        std::cout << "    in database as: " << image.image_db_id << std::endl;
        std::cout << "    Rotation params: " << image.rotation_params[0] << " "
                                           << image.rotation_params[1] << " "
                                           << image.rotation_params[2] << std::endl;
        std::cout << "    Translation params: " << image.translation_params[0] << " "
                                              << image.translation_params[1] << " "
                                              << image.translation_params[2] << std::endl;
        std::cout << "    GPS data: " << image.gps_data[0] << " "
                                    << image.gps_data[1] << " "
                                    << image.gps_data[2] << std::endl;
    }

    PrintHeading1("END OF PRINT");
}