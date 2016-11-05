#include "optimization.h"

BundleAdjustmentConfiguration::BundleAdjustmentConfiguration() {}

size_t BundleAdjustmentConfiguration::NumImages() const {
    return image_ids_.size();
}

size_t BundleAdjustmentConfiguration::NumPoints() const {
    return variable_point3D_ids_.size() + constant_point3D_ids_.size();
}

size_t BundleAdjustmentConfiguration::NumConstantCameras() const {
    return constant_camera_ids_.size();
}

size_t BundleAdjustmentConfiguration::NumConstantPoses() const {
    return constant_poses_.size();
}

size_t BundleAdjustmentConfiguration::NumConstantTvecs() const {
    return constant_tvecs_.size();
}

size_t BundleAdjustmentConfiguration::NumVariablePoints() const {
    return variable_point3D_ids_.size();
}

size_t BundleAdjustmentConfiguration::NumConstantPoints() const {
    return constant_point3D_ids_.size();
}

void BundleAdjustmentConfiguration::AddImage(const image_t image_id) {
    image_ids_.insert(image_id);
}

bool BundleAdjustmentConfiguration::HasImage(const image_t image_id) const {
    return image_ids_.count(image_id) > 0;
}

void BundleAdjustmentConfiguration::RemoveImage(const image_t image_id) {
    image_ids_.erase(image_id);
}

void BundleAdjustmentConfiguration::SetConstantCamera(
        const camera_t camera_id) {
    constant_camera_ids_.insert(camera_id);
}

void BundleAdjustmentConfiguration::SetVariableCamera(
        const camera_t camera_id) {
    constant_camera_ids_.erase(camera_id);
}

bool BundleAdjustmentConfiguration::IsConstantCamera(
        const camera_t camera_id) const {
    return constant_camera_ids_.count(camera_id) > 0;
}

void BundleAdjustmentConfiguration::SetConstantPose(const image_t image_id) {
    CHECK(HasImage(image_id));
    CHECK(!HasConstantTvec(image_id));
    constant_poses_.insert(image_id);
}

void BundleAdjustmentConfiguration::SetVariablePose(const image_t image_id) {
    constant_poses_.erase(image_id);
}

bool BundleAdjustmentConfiguration::HasConstantPose(
        const image_t image_id) const {
    return constant_poses_.count(image_id) > 0;
}

void BundleAdjustmentConfiguration::SetConstantTvec(
        const image_t image_id, const std::vector<int>& idxs) {
    CHECK_GT(idxs.size(), 0);
    CHECK_LE(idxs.size(), 3);
    CHECK(HasImage(image_id));
    CHECK(!HasConstantPose(image_id));
    std::vector<int> unique_idxs = idxs;
    CHECK(std::unique(unique_idxs.begin(), unique_idxs.end()) ==
          unique_idxs.end())
    << "Tvec indices must not contain duplicates";
    constant_tvecs_.emplace(image_id, idxs);
}

void BundleAdjustmentConfiguration::RemoveConstantTvec(const image_t image_id) {
    constant_tvecs_.erase(image_id);
}

bool BundleAdjustmentConfiguration::HasConstantTvec(
        const image_t image_id) const {
    return constant_tvecs_.count(image_id) > 0;
}

const std::unordered_set<image_t>& BundleAdjustmentConfiguration::Images()
const {
    return image_ids_;
}

const std::unordered_set<point3D_t>&
BundleAdjustmentConfiguration::VariablePoints() const {
    return variable_point3D_ids_;
}

const std::unordered_set<point3D_t>&
BundleAdjustmentConfiguration::ConstantPoints() const {
    return constant_point3D_ids_;
}

const std::vector<int>& BundleAdjustmentConfiguration::ConstantTvec(
        const image_t image_id) const {
    return constant_tvecs_.at(image_id);
}

void BundleAdjustmentConfiguration::AddVariablePoint(
        const point3D_t point3D_id) {
    CHECK(!HasConstantPoint(point3D_id));
    variable_point3D_ids_.insert(point3D_id);
}

void BundleAdjustmentConfiguration::AddConstantPoint(
        const point3D_t point3D_id) {
    CHECK(!HasVariablePoint(point3D_id));
    constant_point3D_ids_.insert(point3D_id);
}

bool BundleAdjustmentConfiguration::HasPoint(const point3D_t point3D_id) const {
    return HasVariablePoint(point3D_id) || HasConstantPoint(point3D_id);
}

bool BundleAdjustmentConfiguration::HasVariablePoint(
        const point3D_t point3D_id) const {
    return variable_point3D_ids_.count(point3D_id) > 0;
}

bool BundleAdjustmentConfiguration::HasConstantPoint(
        const point3D_t point3D_id) const {
    return constant_point3D_ids_.count(point3D_id) > 0;
}

void BundleAdjustmentConfiguration::RemoveVariablePoint(
        const point3D_t point3D_id) {
    variable_point3D_ids_.erase(point3D_id);
}

void BundleAdjustmentConfiguration::RemoveConstantPoint(
        const point3D_t point3D_id) {
    constant_point3D_ids_.erase(point3D_id);
}

ceres::LossFunction* BundleAdjuster::Options::CreateLossFunction() const {
    ceres::LossFunction* loss_function = nullptr;
    switch (loss_function_type) {
        case LossFunctionType::TRIVIAL:
            loss_function = new ceres::TrivialLoss();
            break;
        case LossFunctionType::CAUCHY:
            loss_function = new ceres::CauchyLoss(loss_function_scale);
            break;
    }
    return loss_function;
}


void BundleAdjuster::Options::Check() const {
}

BundleAdjuster::BundleAdjuster(const Options& options,
                               const BundleAdjustmentConfiguration& config)
        : options_(options), config_(config) {
    options_.Check();
}

bool BundleAdjuster::Solve(Reconstruction* reconstruction) {
    point3D_num_images_.clear();

    problem_.reset(new ceres::Problem());

    ceres::LossFunction* loss_function = options_.CreateLossFunction();
    SetUp(reconstruction, loss_function);

    ParameterizeCameras(reconstruction);
    ParameterizePoints(reconstruction);

    if (problem_->NumResiduals() == 0) {
        return false;
    }

    ceres::Solver::Options solver_options = options_.solver_options;

    const size_t kMaxNumImagesDirectDenseSolver = 50;
    const size_t kMaxNumImagesDirectSparseSolver = 1000;
    const size_t num_images = config_.NumImages();
    if (num_images <= kMaxNumImagesDirectDenseSolver) {
        solver_options.linear_solver_type = ceres::DENSE_SCHUR;
    } else if (num_images <= kMaxNumImagesDirectSparseSolver) {
        solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
    } else {
        solver_options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        solver_options.preconditioner_type = ceres::SCHUR_JACOBI;
    }

#ifdef OPENMP_ENABLED
    if (solver_options.num_threads <= 0) {
solver_options.num_threads = omp_get_max_threads();
}
if (solver_options.num_linear_solver_threads <= 0) {
solver_options.num_linear_solver_threads = omp_get_max_threads();
}
#else
    solver_options.num_threads = 1;
    solver_options.num_linear_solver_threads = 1;
#endif

    std::string error;

    ceres::Solve(solver_options, problem_.get(), &summary_);

    if (solver_options.minimizer_progress_to_stdout) {
        std::cout << std::endl;
    }

    if (options_.print_summary) {
        PrintHeading2("Bundle Adjustment Report");
        PrintSolverSummary(summary_);
    }

    TearDown(reconstruction);

    return true;
}

ceres::Solver::Summary BundleAdjuster::Summary() const { return summary_; }

void BundleAdjuster::SetUp(Reconstruction* reconstruction,
                           ceres::LossFunction* loss_function) {
    for (const image_t image_id : config_.Images()) {
        AddImageToProblem(image_id, reconstruction, loss_function);
    }

    FillPoints(config_.VariablePoints(), reconstruction, loss_function);
    FillPoints(config_.ConstantPoints(), reconstruction, loss_function);
}

void BundleAdjuster::TearDown(Reconstruction*) {
}

void BundleAdjuster::AddImageToProblem(const image_t image_id,
                                       Reconstruction* reconstruction,
                                       ceres::LossFunction* loss_function) {
    Image& image = reconstruction->Image(image_id);

    if (image.NumPoints3D() <
        static_cast<size_t>(options_.min_observations_per_image)) {
        return;
    }

    Camera& camera = reconstruction->Camera(image.CameraId());

    image.NormalizeQvec();

    double* qvec_data = image.Qvec().data();
    double* tvec_data = image.Tvec().data();
    double* camera_params_data = camera.ParamsData();

    camera_ids_.insert(image.CameraId());

    const bool constant_pose = config_.HasConstantPose(image_id);

    for (const Point2D& point2D : image.Points2D()) {
        if (!point2D.HasPoint3D()) {
            continue;
        }

        point3D_num_images_[point2D.Point3DId()] += 1;

        Point_3D& point3D = reconstruction->Point3D(point2D.Point3DId());

        ceres::CostFunction* cost_function = nullptr;

        if (constant_pose) {
            cost_function = BundleAdjustmentConstantPoseCostFunction<RadialCameraModel>::Create(
                    image.Qvec(), image.Tvec(), point2D.XY());
            problem_->AddResidualBlock(cost_function, loss_function, point3D.XYZ().data(), camera_params_data);
        } else {
            cost_function = BundleAdjustmentCostFunction<RadialCameraModel>::Create(point2D.XY());
            problem_->AddResidualBlock(cost_function, loss_function, qvec_data, tvec_data, point3D.XYZ().data(),
                                       camera_params_data);
        }
    }

    if (!constant_pose) {
        ceres::LocalParameterization* quaternion_parameterization =
                new ceres::QuaternionParameterization;
        problem_->SetParameterization(qvec_data, quaternion_parameterization);
        if (config_.HasConstantTvec(image_id)) {
            const std::vector<int>& constant_tvec_idxs =
                    config_.ConstantTvec(image_id);
            ceres::SubsetParameterization* tvec_parameterization =
                    new ceres::SubsetParameterization(3, constant_tvec_idxs);
            problem_->SetParameterization(tvec_data, tvec_parameterization);
        }
    }
}

void BundleAdjuster::FillPoints(
        const std::unordered_set<point3D_t>& point3D_ids,
        Reconstruction* reconstruction, ceres::LossFunction* loss_function) {
    for (const point3D_t point3D_id : point3D_ids) {
        Point_3D& point3D = reconstruction->Point3D(point3D_id);

        if (point3D_num_images_[point3D_id] == point3D.Track().Length()) {
            continue;
        }

        for (const auto& track_el : point3D.Track().Elements()) {
            if (config_.HasImage(track_el.image_id)) {
                continue;
            }

            point3D_num_images_[point3D_id] += 1;

            Image& image = reconstruction->Image(track_el.image_id);
            Camera& camera = reconstruction->Camera(image.CameraId());
            const Point2D& point2D = image.Point2D(track_el.point2D_idx);

            if (camera_ids_.count(image.CameraId()) == 0) {
                camera_ids_.insert(image.CameraId());
                config_.SetConstantCamera(image.CameraId());
            }

            ceres::CostFunction* cost_function = nullptr;

            cost_function = BundleAdjustmentConstantPoseCostFunction<RadialCameraModel>::Create(
                    image.Qvec(), image.Tvec(), point2D.XY());
            problem_->AddResidualBlock(cost_function, loss_function, point3D.XYZ().data(), camera.ParamsData());
        }
    }
}

void BundleAdjuster::ParameterizeCameras(Reconstruction* reconstruction) {
    const bool constant_camera = !options_.refine_focal_length &&
                                 !options_.refine_principal_point &&
                                 !options_.refine_extra_params;
    for (const camera_t camera_id : camera_ids_) {
        Camera& camera = reconstruction->Camera(camera_id);

        if (constant_camera || config_.IsConstantCamera(camera_id)) {
            problem_->SetParameterBlockConstant(camera.ParamsData());
            continue;
        } else {
            std::vector<int> const_camera_params;

            if (!options_.refine_focal_length) {
                const std::vector<size_t>& params_idxs = camera.FocalLengthIdxs();
                const_camera_params.insert(const_camera_params.end(),
                                           params_idxs.begin(), params_idxs.end());
            }
            if (!options_.refine_principal_point) {
                const std::vector<size_t>& params_idxs = camera.PrincipalPointIdxs();
                const_camera_params.insert(const_camera_params.end(),
                                           params_idxs.begin(), params_idxs.end());
            }
            if (!options_.refine_extra_params) {
                const std::vector<size_t>& params_idxs = camera.ExtraParamsIdxs();
                const_camera_params.insert(const_camera_params.end(),
                                           params_idxs.begin(), params_idxs.end());
            }

            if (const_camera_params.size() > 0) {
                ceres::SubsetParameterization* camera_params_parameterization =
                        new ceres::SubsetParameterization(
                                static_cast<int>(camera.NumParams()), const_camera_params);
                problem_->SetParameterization(camera.ParamsData(),
                                              camera_params_parameterization);
            }
        }
    }
}

void BundleAdjuster::ParameterizePoints(Reconstruction* reconstruction) {
    for (const auto num_images : point3D_num_images_) {
        if (!config_.HasVariablePoint(num_images.first) &&
            !config_.HasConstantPoint(num_images.first)) {
            Point_3D& point3D = reconstruction->Point3D(num_images.first);
            if (point3D.Track().Length() > point3D_num_images_[num_images.first]) {
                problem_->SetParameterBlockConstant(point3D.XYZ().data());
            }
        }
    }

    for (const point3D_t point3D_id : config_.ConstantPoints()) {
        Point_3D& point3D = reconstruction->Point3D(point3D_id);
        problem_->SetParameterBlockConstant(point3D.XYZ().data());
    }
}


void PrintSolverSummary(const ceres::Solver::Summary& summary) {
    std::cout << std::right << std::setw(16) << "Residuals : ";
    std::cout << std::left << summary.num_residuals_reduced << std::endl;

    std::cout << std::right << std::setw(16) << "Parameters : ";
    std::cout << std::left << summary.num_effective_parameters_reduced
    << std::endl;

    std::cout << std::right << std::setw(16) << "Iterations : ";
    std::cout << std::left
    << summary.num_successful_steps + summary.num_unsuccessful_steps
    << std::endl;

    std::cout << std::right << std::setw(16) << "Time : ";
    std::cout << std::left << summary.total_time_in_seconds << " [s]"
    << std::endl;

    std::cout << std::right << std::setw(16) << "Initial cost : ";
    std::cout << std::right << std::setprecision(6)
    << std::sqrt(summary.initial_cost / summary.num_residuals_reduced)
    << " [px]" << std::endl;

    std::cout << std::right << std::setw(16) << "Final cost : ";
    std::cout << std::right << std::setprecision(6)
    << std::sqrt(summary.final_cost / summary.num_residuals_reduced)
    << " [px]" << std::endl;

    std::cout << std::right << std::setw(16) << "Termination : ";

    std::string termination = "";

    switch (summary.termination_type) {
        case ceres::CONVERGENCE:
            termination = "Convergence";
            break;
        case ceres::NO_CONVERGENCE:
            termination = "No convergence";
            break;
        case ceres::FAILURE:
            termination = "Failure";
            break;
        case ceres::USER_SUCCESS:
            termination = "User success";
            break;
        case ceres::USER_FAILURE:
            termination = "User failure";
            break;
        default:
            termination = "Unknown";
            break;
    }

    std::cout << std::right << termination << std::endl;
    std::cout << std::endl;
}
