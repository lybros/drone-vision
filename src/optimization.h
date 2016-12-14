#ifndef INC_3D_RECONSTRUCTION_OPTIMIZATION_H
#define INC_3D_RECONSTRUCTION_OPTIMIZATION_H

#include "utils.h"
#include "reconstruction.h"

#include <unordered_set>
#include <unordered_map>

#include <Eigen/Core>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

template<typename CameraModel>
class BundleAdjustmentCostFunction {
public:
    BundleAdjustmentCostFunction(const Eigen::Vector2d& point2D)
            : point2D_(point2D) { }

    static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
        return (new ceres::AutoDiffCostFunction<
                BundleAdjustmentCostFunction<CameraModel>, 2, 4, 3, 3,
                CameraModel::num_params>(
                new BundleAdjustmentCostFunction(point2D)));
    }

    template<typename T>
    bool operator()(const T* const qvec, const T* const tvec,
                    const T* const point3D, const T* const camera_params,
                    T* residuals) const {
        T point3D_local[3];
        ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_local);
        point3D_local[0] += tvec[0];
        point3D_local[1] += tvec[1];
        point3D_local[2] += tvec[2];

        point3D_local[0] /= point3D_local[2];
        point3D_local[1] /= point3D_local[2];

        T x, y;
        CameraModel::WorldToImage(camera_params, point3D_local[0], point3D_local[1],
                                  &x, &y);

        residuals[0] = x - T(point2D_(0));
        residuals[1] = y - T(point2D_(1));

        return true;
    }

private:
    const Eigen::Vector2d point2D_;
};

template<typename CameraModel>
class BundleAdjustmentConstantPoseCostFunction {
public:
    BundleAdjustmentConstantPoseCostFunction(const Eigen::Vector4d& qvec,
                                             const Eigen::Vector3d& tvec,
                                             const Eigen::Vector2d& point2D)
            : qvec_(qvec), tvec_(tvec), point2D_(point2D) { }

    static ceres::CostFunction* Create(const Eigen::Vector4d& qvec,
                                       const Eigen::Vector3d& tvec,
                                       const Eigen::Vector2d& point2D) {
        return (new ceres::AutoDiffCostFunction<
                BundleAdjustmentConstantPoseCostFunction<CameraModel>, 2, 3,
                CameraModel::num_params>(
                new BundleAdjustmentConstantPoseCostFunction(qvec, tvec, point2D)));
    }

    template<typename T>
    bool operator()(const T* const point3D, const T* const camera_params,
                    T* residuals) const {
        T qvec[4] = {T(qvec_(0)), T(qvec_(1)), T(qvec_(2)), T(qvec_(3))};

        T point3D_local[3];
        ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_local);
        point3D_local[0] += T(tvec_(0));
        point3D_local[1] += T(tvec_(1));
        point3D_local[2] += T(tvec_(2));

        point3D_local[0] /= point3D_local[2];
        point3D_local[1] /= point3D_local[2];

        T x, y;
        CameraModel::WorldToImage(camera_params, point3D_local[0], point3D_local[1],
                                  &x, &y);

        residuals[0] = x - T(point2D_(0));
        residuals[1] = y - T(point2D_(1));

        return true;
    }

private:
    const Eigen::Vector4d qvec_;
    const Eigen::Vector3d tvec_;
    const Eigen::Vector2d point2D_;
};

class RelativePoseCostFunction {
public:
    RelativePoseCostFunction(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2)
            : x1_(x1), x2_(x2) { }

    static ceres::CostFunction* Create(const Eigen::Vector2d& x1,
                                       const Eigen::Vector2d& x2) {
        return (new ceres::AutoDiffCostFunction<RelativePoseCostFunction, 1, 4, 3>(
                new RelativePoseCostFunction(x1, x2)));
    }

    template<typename T>
    bool operator()(const T* const qvec, const T* const tvec,
                    T* residuals) const {
        Eigen::Matrix<T, 3, 3, Eigen::RowMajor> R;
        ceres::QuaternionToRotation(qvec, R.data());

        Eigen::Matrix<T, 3, 3> t_x;
        t_x << T(0), -tvec[2], tvec[1], tvec[2], T(0), -tvec[0], -tvec[1], tvec[0],
                T(0);

        const Eigen::Matrix<T, 3, 3> E = t_x * R;

        const Eigen::Matrix<T, 3, 1> x1_h(T(x1_(0)), T(x1_(1)), T(1));
        const Eigen::Matrix<T, 3, 1> x2_h(T(x2_(0)), T(x2_(1)), T(1));

        const Eigen::Matrix<T, 3, 1> Ex1 = E * x1_h;
        const Eigen::Matrix<T, 3, 1> Etx2 = E.transpose() * x2_h;
        const T x2tEx1 = x2_h.transpose() * Ex1;
        residuals[0] = x2tEx1 * x2tEx1 / (Ex1(0) * Ex1(0) + Ex1(1) * Ex1(1) +
                                          Etx2(0) * Etx2(0) + Etx2(1) * Etx2(1));

        return true;
    }

private:
    const Eigen::Vector2d x1_;
    const Eigen::Vector2d x2_;
};

struct UnitTranslationPlus {
    template<typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
        x_plus_delta[0] = x[0] + delta[0];
        x_plus_delta[1] = x[1] + delta[1];
        x_plus_delta[2] = x[2] + delta[2];

        const T squared_norm = x_plus_delta[0] * x_plus_delta[0] +
                               x_plus_delta[1] * x_plus_delta[1] +
                               x_plus_delta[2] * x_plus_delta[2];

        if (squared_norm > T(0)) {
            const T norm = T(1.0) / ceres::sqrt(squared_norm);
            x_plus_delta[0] *= norm;
            x_plus_delta[1] *= norm;
            x_plus_delta[2] *= norm;
        }

        return true;
    }
};

class BundleAdjustmentConfiguration {
public:
    BundleAdjustmentConfiguration();

    size_t NumImages() const;

    size_t NumPoints() const;

    size_t NumConstantCameras() const;

    size_t NumConstantPoses() const;

    size_t NumConstantTvecs() const;

    size_t NumVariablePoints() const;

    size_t NumConstantPoints() const;

    void AddImage(const image_t image_id);

    bool HasImage(const image_t image_id) const;

    void RemoveImage(const image_t image_id);

    void SetConstantCamera(const camera_t camera_id);

    void SetVariableCamera(const camera_t camera_id);

    bool IsConstantCamera(const camera_t camera_id) const;

    void SetConstantPose(const image_t image_id);

    void SetVariablePose(const image_t image_id);

    bool HasConstantPose(const image_t image_id) const;

    void SetConstantTvec(const image_t image_id, const std::vector<int>& idxs);

    void RemoveConstantTvec(const image_t image_id);

    bool HasConstantTvec(const image_t image_id) const;

    void AddVariablePoint(const point3D_t point3D_id);

    void AddConstantPoint(const point3D_t point3D_id);

    bool HasPoint(const point3D_t point3D_id) const;

    bool HasVariablePoint(const point3D_t point3D_id) const;

    bool HasConstantPoint(const point3D_t point3D_id) const;

    void RemoveVariablePoint(const point3D_t point3D_id);

    void RemoveConstantPoint(const point3D_t point3D_id);

    const std::unordered_set<image_t>& Images() const;

    const std::unordered_set<point3D_t>& VariablePoints() const;

    const std::unordered_set<point3D_t>& ConstantPoints() const;

    const std::vector<int>& ConstantTvec(const image_t image_id) const;

private:
    std::unordered_set<camera_t> constant_camera_ids_;
    std::unordered_set<image_t> image_ids_;
    std::unordered_set<point3D_t> variable_point3D_ids_;
    std::unordered_set<point3D_t> constant_point3D_ids_;
    std::unordered_set<image_t> constant_poses_;
    std::unordered_map<image_t, std::vector<int>> constant_tvecs_;
};

class BundleAdjuster {
public:
    struct Options {
        int min_observations_per_image = 10;

        enum class LossFunctionType { TRIVIAL, CAUCHY };
        LossFunctionType loss_function_type = LossFunctionType::TRIVIAL;

        double loss_function_scale = 1.0;

        bool refine_focal_length = true;

        bool refine_principal_point = false;

        bool refine_extra_params = true;

        bool print_summary = true;

        // Initialized in MapperOptions::GlobalBundleAdjustmentOptions() &
        //                MapperOptions::LocalBundleAdjustmentOptions()
        bool use_drone_data;
        bool use_qvec_tvec_estimations;

        ceres::Solver::Options solver_options;

        Options() {
            solver_options.function_tolerance = 0.0;
            solver_options.gradient_tolerance = 0.0;
            solver_options.parameter_tolerance = 0.0;
            solver_options.minimizer_progress_to_stdout = false;
            solver_options.max_num_iterations = 50;
            solver_options.num_threads = -1;
            solver_options.num_linear_solver_threads = -1;
        }

        ceres::LossFunction* CreateLossFunction() const;

        void Check() const;
    };

    explicit BundleAdjuster(const Options& options,
                            const BundleAdjustmentConfiguration& config);

    bool Solve(Reconstruction* reconstruction);

    ceres::Solver::Summary Summary() const;

private:
    void SetUp(Reconstruction* reconstruction,
               ceres::LossFunction* loss_function);

    void TearDown(Reconstruction* reconstruction);

    void AddImageToProblem(const image_t image_id, Reconstruction* reconstruction,
                           ceres::LossFunction* loss_function);

    void FillPoints(const std::unordered_set<point3D_t>& point3D_ids,
                    Reconstruction* reconstruction,
                    ceres::LossFunction* loss_function);

    void ParameterizeCameras(Reconstruction* reconstruction);

    void ParameterizePoints(Reconstruction* reconstruction);

    const Options options_;
    BundleAdjustmentConfiguration config_;
    std::unique_ptr<ceres::Problem> problem_;
    ceres::Solver::Summary summary_;
    std::unordered_set<camera_t> camera_ids_;
    std::unordered_map<point3D_t, size_t> point3D_num_images_;
};

void PrintSolverSummary(const ceres::Solver::Summary& summary);

#endif //INC_3D_RECONSTRUCTION_OPTIMIZATION_H
