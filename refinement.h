#ifndef INC_3D_RECONSTRUCTION_REFINEMENT_H
#define INC_3D_RECONSTRUCTION_REFINEMENT_H

#include "optimization.h"

struct AbsolutePoseEstimationOptions {
    bool estimate_focal_length = false;

    size_t num_focal_length_samples = 30;

    double min_focal_length_ratio = 0.2;

    double max_focal_length_ratio = 5;

    int num_threads = ThreadPool::kMaxNumThreads;

    RANSACOptions ransac_options;

    void Check() const {
        ransac_options.Check();
    }
};

struct AbsolutePoseRefinementOptions {
    double gradient_tolerance = 1.0;

    int max_num_iterations = 100;

    double loss_function_scale = 1.0;

    bool refine_focal_length = true;

    bool refine_extra_params = true;

    bool print_summary = true;

    void Check() const {
    }
};

bool EstimateAbsolutePose(const AbsolutePoseEstimationOptions& options,
                          const std::vector<Eigen::Vector2d>& points2D,
                          const std::vector<Eigen::Vector3d>& points3D,
                          Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                          Camera* camera, size_t* num_inliers,
                          std::vector<bool>* inlier_mask);

size_t EstimateRelativePose(const RANSACOptions& ransac_options,
                            const std::vector<Eigen::Vector2d>& points1,
                            const std::vector<Eigen::Vector2d>& points2,
                            Eigen::Vector4d* qvec, Eigen::Vector3d* tvec);

bool RefineAbsolutePose(const AbsolutePoseRefinementOptions& options,
                        const std::vector<bool>& inlier_mask,
                        const std::vector<Eigen::Vector2d>& points2D,
                        const std::vector<Eigen::Vector3d>& points3D,
                        Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                        Camera* camera);

bool RefineRelativePose(const ceres::Solver::Options& options,
                        const std::vector<Eigen::Vector2d>& points1,
                        const std::vector<Eigen::Vector2d>& points2,
                        Eigen::Vector4d* qvec, Eigen::Vector3d* tvec);

bool RefineEssentialMatrix(const ceres::Solver::Options& options,
                           const std::vector<Eigen::Vector2d>& points1,
                           const std::vector<Eigen::Vector2d>& points2,
                           const std::vector<bool>& inlier_mask,
                           Eigen::Matrix3d* E);


#endif //INC_3D_RECONSTRUCTION_REFINEMENT_H
