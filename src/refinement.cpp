#include "refinement.h"

namespace {

    typedef LORANSAC<P3PEstimator, EPnPEstimator> AbsolutePoseRANSAC_t;

    AbsolutePoseRANSAC_t::Report EstimateAbsolutePoseKernel(
            const Camera& camera, const double focal_length_factor,
            const std::vector<Eigen::Vector2d>& points2D,
            const std::vector<Eigen::Vector3d>& points3D, RANSACOptions options) {
        Camera scaled_camera = camera;
        const std::vector<size_t>& focal_length_idxs = camera.FocalLengthIdxs();
        for (const size_t idx : focal_length_idxs) {
            scaled_camera.Params(idx) *= focal_length_factor;
        }

        std::vector<Eigen::Vector2d> points2D_N(points2D.size());
        for (size_t i = 0; i < points2D.size(); ++i) {
            points2D_N[i] = scaled_camera.ImageToWorld(points2D[i]);
        }

        options.max_error = scaled_camera.ImageToWorldThreshold(options.max_error);
        AbsolutePoseRANSAC_t ransac(options);
        const auto report = ransac.Estimate(points2D_N, points3D);

        return report;
    }

}


bool EstimateAbsolutePose(const AbsolutePoseEstimationOptions& options,
                          const std::vector<Eigen::Vector2d>& points2D,
                          const std::vector<Eigen::Vector3d>& points3D,
                          Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                          Camera* camera, size_t* num_inliers,
                          std::vector<bool>* inlier_mask) {
    options.Check();

    std::vector<double> focal_length_factors;
    if (options.estimate_focal_length) {
        focal_length_factors.reserve(options.num_focal_length_samples + 1);
        const double fstep = 1.0 / options.num_focal_length_samples;
        const double fscale =
                options.max_focal_length_ratio - options.min_focal_length_ratio;
        for (double f = 0; f <= 1.0; f += fstep) {
            focal_length_factors.push_back(options.min_focal_length_ratio +
                                           fscale * f * f);
        }
    } else {
        focal_length_factors.reserve(1);
        focal_length_factors.push_back(1);
    }

    std::vector<std::future<typename AbsolutePoseRANSAC_t::Report>> futures;
    futures.reserve(focal_length_factors.size());

    ThreadPool thread_pool(std::min(
            options.num_threads, static_cast<int>(focal_length_factors.size())));

    for (const double focal_length_factor : focal_length_factors) {
        futures.push_back(thread_pool.AddTask(EstimateAbsolutePoseKernel, *camera,
                                              focal_length_factor, points2D,
                                              points3D, options.ransac_options));
    }

    double focal_length_factor = 0;
    Eigen::Matrix3x4d proj_matrix;
    *num_inliers = 0;
    inlier_mask->clear();

    for (size_t i = 0; i < focal_length_factors.size(); ++i) {
        const auto report = futures[i].get();
        if (report.success && report.support.num_inliers > *num_inliers) {
            *num_inliers = report.support.num_inliers;
            proj_matrix = report.model;
            *inlier_mask = report.inlier_mask;
            focal_length_factor = focal_length_factors[i];
        }
    }

    if (*num_inliers == 0) {
        return false;
    }

    if (options.estimate_focal_length && *num_inliers > 0) {
        const std::vector<size_t>& focal_length_idxs = camera->FocalLengthIdxs();
        for (const size_t idx : focal_length_idxs) {
            camera->Params(idx) *= focal_length_factor;
        }
    }

    *qvec = RotationMatrixToQuaternion(proj_matrix.leftCols<3>());
    *tvec = proj_matrix.rightCols<1>();

    if (IsNaN(*qvec) || IsNaN(*tvec)) {
        return false;
    }

    return true;
}

size_t EstimateRelativePose(const RANSACOptions& ransac_options,
                            const std::vector<Eigen::Vector2d>& points1,
                            const std::vector<Eigen::Vector2d>& points2,
                            Eigen::Vector4d* qvec, Eigen::Vector3d* tvec) {
    RANSAC<EssentialMatrixFivePointEstimator> ransac(ransac_options);
    const auto report = ransac.Estimate(points1, points2);

    if (!report.success) {
        return 0;
    }

    std::vector<Eigen::Vector2d> inliers1(report.support.num_inliers);
    std::vector<Eigen::Vector2d> inliers2(report.support.num_inliers);

    size_t j = 0;
    for (size_t i = 0; i < points1.size(); ++i) {
        if (report.inlier_mask[i]) {
            inliers1[j] = points1[i];
            inliers2[j] = points2[i];
            j += 1;
        }
    }

    Eigen::Matrix3d R;

    std::vector<Eigen::Vector3d> points3D;
    PoseFromEssentialMatrix(report.model, inliers1, inliers2, &R, tvec,
                            &points3D);

    *qvec = RotationMatrixToQuaternion(R);

    if (IsNaN(*qvec) || IsNaN(*tvec)) {
        return 0;
    }

    return points3D.size();
}

bool RefineAbsolutePose(const AbsolutePoseRefinementOptions& options,
                        const std::vector<bool>& inlier_mask,
                        const std::vector<Eigen::Vector2d>& points2D,
                        const std::vector<Eigen::Vector3d>& points3D,
                        Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                        Camera* camera) {
    options.Check();

    ceres::LossFunction* loss_function =
            new ceres::CauchyLoss(options.loss_function_scale);

    double* camera_params_data = camera->ParamsData();
    double* qvec_data = qvec->data();
    double* tvec_data = tvec->data();

    std::vector<Eigen::Vector3d> points3D_copy = points3D;

    ceres::Problem problem;

    for (size_t i = 0; i < points2D.size(); ++i) {
        if (!inlier_mask[i]) {
            continue;
        }

        ceres::CostFunction* cost_function = nullptr;

        cost_function = BundleAdjustmentCostFunction<RadialCameraModel>::Create(points2D[i]);
        problem.AddResidualBlock(cost_function, loss_function, qvec_data,
                                 tvec_data, points3D_copy[i].data(),
                                 camera_params_data);

        problem.SetParameterBlockConstant(points3D_copy[i].data());
    }

    if (problem.NumResiduals() > 0) {
        *qvec = NormalizeQuaternion(*qvec);
        ceres::LocalParameterization* quaternion_parameterization =
                new ceres::QuaternionParameterization;
        problem.SetParameterization(qvec_data, quaternion_parameterization);

        if (!options.refine_focal_length && !options.refine_extra_params) {
            problem.SetParameterBlockConstant(camera->ParamsData());
        } else {
            std::vector<int> camera_params_const;
            const std::vector<size_t>& principal_point_idxs =
                    camera->PrincipalPointIdxs();
            camera_params_const.insert(camera_params_const.end(),
                                       principal_point_idxs.begin(),
                                       principal_point_idxs.end());

            if (!options.refine_focal_length) {
                const std::vector<size_t>& focal_length_idxs =
                        camera->FocalLengthIdxs();
                camera_params_const.insert(camera_params_const.end(),
                                           focal_length_idxs.begin(),
                                           focal_length_idxs.end());
            }

            if (!options.refine_extra_params) {
                const std::vector<size_t>& extra_params_idxs =
                        camera->ExtraParamsIdxs();
                camera_params_const.insert(camera_params_const.end(),
                                           extra_params_idxs.begin(),
                                           extra_params_idxs.end());
            }

            ceres::SubsetParameterization* camera_params_parameterization =
                    new ceres::SubsetParameterization(
                            static_cast<int>(camera->NumParams()), camera_params_const);
            problem.SetParameterization(camera->ParamsData(),
                                        camera_params_parameterization);
        }
    }

    ceres::Solver::Options solver_options;
    solver_options.gradient_tolerance = options.gradient_tolerance;
    solver_options.max_num_iterations = options.max_num_iterations;
    solver_options.linear_solver_type = ceres::DENSE_QR;

#ifdef OPENMP_ENABLED
    solver_options.num_threads = omp_get_max_threads();
solver_options.num_linear_solver_threads = omp_get_max_threads();
#endif

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);

    if (solver_options.minimizer_progress_to_stdout) {
        std::cout << std::endl;
    }

    if (options.print_summary) {
        PrintHeading2("Pose Refinement Report");
        PrintSolverSummary(summary);
    }

    return summary.IsSolutionUsable();
}

bool RefineRelativePose(const ceres::Solver::Options& options,
                        const std::vector<Eigen::Vector2d>& points1,
                        const std::vector<Eigen::Vector2d>& points2,
                        Eigen::Vector4d* qvec, Eigen::Vector3d* tvec) {
    CHECK_EQ(points1.size(), points2.size());

    *qvec = NormalizeQuaternion(*qvec);

    ceres::LossFunction* loss_function = new ceres::CauchyLoss(1);

    ceres::Problem problem;

    for (size_t i = 0; i < points1.size(); ++i) {
        ceres::CostFunction* cost_function =
                RelativePoseCostFunction::Create(points1[i], points2[i]);
        problem.AddResidualBlock(cost_function, loss_function, qvec->data(),
                                 tvec->data());
    }

    ceres::LocalParameterization* quaternion_parameterization =
            new ceres::QuaternionParameterization;
    problem.SetParameterization(qvec->data(), quaternion_parameterization);

    ceres::LocalParameterization* local_parameterization =
            new ceres::AutoDiffLocalParameterization<UnitTranslationPlus, 3, 2>;
    problem.SetParameterization(tvec->data(), local_parameterization);

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return summary.IsSolutionUsable();
}


bool RefineEssentialMatrix(const ceres::Solver::Options& options,
                           const std::vector<Eigen::Vector2d>& points1,
                           const std::vector<Eigen::Vector2d>& points2,
                           const std::vector<bool>& inlier_mask,
                           Eigen::Matrix3d* E) {
    size_t num_inliers = 0;
    for (const auto inlier : inlier_mask) {
        if (inlier) {
            num_inliers += 1;
        }
    }

    std::vector<Eigen::Vector2d> inlier_points1(num_inliers);
    std::vector<Eigen::Vector2d> inlier_points2(num_inliers);
    size_t j = 0;
    for (size_t i = 0; i < inlier_mask.size(); ++i) {
        if (inlier_mask[i]) {
            inlier_points1[j] = points1[i];
            inlier_points2[j] = points2[i];
            j += 1;
        }
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d tvec;
    std::vector<Eigen::Vector3d> points3D;
    PoseFromEssentialMatrix(*E, inlier_points1, inlier_points2, &R, &tvec,
                            &points3D);

    Eigen::Vector4d qvec = RotationMatrixToQuaternion(R);

    if (points3D.size() == 0) {
        return false;
    }

    const bool refinement_success =
            RefineRelativePose(options, inlier_points1, inlier_points2, &qvec, &tvec);

    if (!refinement_success) {
        return false;
    }

    const Eigen::Matrix3d rot_mat = QuaternionToRotationMatrix(qvec);
    *E = EssentialMatrixFromPose(rot_mat, tvec);

    return true;
}
