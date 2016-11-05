#ifndef INC_3D_RECONSTRUCTION_ESTIMATORS_H
#define INC_3D_RECONSTRUCTION_ESTIMATORS_H

#include "utils.h"
#include "entities.h"
#include "projection.h"

#include <future>

#include <ceres/solver.h>
#include <ceres/loss_function.h>
#include <ceres/cost_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>

#include <Eigen/Geometry>

void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d>& points,
                                   std::vector<Eigen::Vector2d>* normed_points,
                                   Eigen::Matrix3d* matrix);

void ComputeSquaredSampsonError(const std::vector<Eigen::Vector2d>& points1,
                                const std::vector<Eigen::Vector2d>& points2,
                                const Eigen::Matrix3d& E,
                                std::vector<double>* residuals);

void ComputeSquaredReprojectionError(
        const std::vector<Eigen::Vector2d>& points2D,
        const std::vector<Eigen::Vector3d>& points3D,
        const Eigen::Matrix3x4d& proj_matrix, std::vector<double>* residuals);

struct InlierSupportMeasurer {
    struct Support {
        size_t num_inliers = 0;

        double residual_sum = std::numeric_limits<double>::max();
    };

    Support Evaluate(const std::vector<double>& residuals,
                     const double max_residual);

    bool Compare(const Support& support1, const Support& support2);
};

struct MEstimatorSupportMeasurer {
    struct Support {
        size_t num_inliers = 0;

        double score = std::numeric_limits<double>::max();
    };

    Support Evaluate(const std::vector<double>& residuals,
                     const double max_residual);

    bool Compare(const Support& support1, const Support& support2);
};

struct RANSACOptions {
    double max_error = 0.0;

    double min_inlier_ratio = 0.1;

    double confidence = 0.99;

    size_t min_num_trials = 0;
    size_t max_num_trials = std::numeric_limits<size_t>::max();

    void Check() const {
    }
};


class Sampler {
public:
    Sampler(){};
    Sampler(const size_t num_samples);

    virtual void Initialize(const size_t total_num_samples) = 0;

    virtual size_t MaxNumSamples() = 0;

    virtual std::vector<size_t> Sample() = 0;

    template <typename X_t>
    void SampleX(const X_t& X, X_t* X_rand);

    template <typename X_t, typename Y_t>
    void SampleXY(const X_t& X, const Y_t& Y, X_t* X_rand, Y_t* Y_rand);
};

template <typename X_t>
void Sampler::SampleX(const X_t& X, X_t* X_rand) {
    const auto sample_idxs = Sample();
    for (size_t i = 0; i < X_rand->size(); ++i) {
        (*X_rand)[i] = X[sample_idxs[i]];
    }
}

template <typename X_t, typename Y_t>
void Sampler::SampleXY(const X_t& X, const Y_t& Y, X_t* X_rand, Y_t* Y_rand) {
    const auto sample_idxs = Sample();
    for (size_t i = 0; i < X_rand->size(); ++i) {
        (*X_rand)[i] = X[sample_idxs[i]];
        (*Y_rand)[i] = Y[sample_idxs[i]];
    }
}


class RandomSampler : public Sampler {
public:
    RandomSampler(const size_t num_samples);

    void Initialize(const size_t total_num_samples) override;

    size_t MaxNumSamples() override;

    std::vector<size_t> Sample() override;

private:
    const size_t num_samples_;
    std::vector<size_t> sample_idxs_;
};


class CombinationSampler : public Sampler {
public:
    CombinationSampler(const size_t num_samples);

    void Initialize(const size_t total_num_samples) override;

    size_t MaxNumSamples() override;

    std::vector<size_t> Sample() override;

private:
    const size_t num_samples_;
    std::vector<size_t> total_sample_idxs_;
};


template <typename Estimator, typename SupportMeasurer = InlierSupportMeasurer,
        typename Sampler = RandomSampler>
class RANSAC {
public:
    struct Report {
        bool success = false;

        size_t num_trials = 0;

        typename SupportMeasurer::Support support;

        std::vector<bool> inlier_mask;

        typename Estimator::M_t model;
    };

    RANSAC(const RANSACOptions& options);

    static size_t ComputeNumTrials(const size_t num_inliers,
                                   const size_t num_samples,
                                   const double confidence);

    Report Estimate(const std::vector<typename Estimator::X_t>& X,
                    const std::vector<typename Estimator::Y_t>& Y);

    Estimator estimator;
    Sampler sampler;
    SupportMeasurer support_measurer;

protected:
    RANSACOptions options_;
};

template <typename Estimator, typename SupportMeasurer, typename Sampler>
RANSAC<Estimator, SupportMeasurer, Sampler>::RANSAC(
        const RANSACOptions& options)
        : sampler(Sampler(Estimator::MinNumSamples())), options_(options) {
    options.Check();

    const size_t kNumSamples = 100000;
    const size_t dyn_max_num_trials = ComputeNumTrials(
            static_cast<size_t>(options_.min_inlier_ratio * kNumSamples), kNumSamples,
            options_.confidence);
    options_.max_num_trials =
            std::min<size_t>(options_.max_num_trials, dyn_max_num_trials);
}

template <typename Estimator, typename SupportMeasurer, typename Sampler>
size_t RANSAC<Estimator, SupportMeasurer, Sampler>::ComputeNumTrials(
        const size_t num_inliers, const size_t num_samples,
        const double confidence) {
    const double inlier_ratio = num_inliers / static_cast<double>(num_samples);

    const double nom = 1 - confidence;
    if (nom <= 0) {
        return std::numeric_limits<size_t>::max();
    }

    const double denom = 1 - std::pow(inlier_ratio, Estimator::MinNumSamples());
    if (denom <= 0) {
        return 1;
    }

    return static_cast<size_t>(std::ceil(std::log(nom) / std::log(denom)));
}

template <typename Estimator, typename SupportMeasurer, typename Sampler>
typename RANSAC<Estimator, SupportMeasurer, Sampler>::Report
RANSAC<Estimator, SupportMeasurer, Sampler>::Estimate(
        const std::vector<typename Estimator::X_t>& X,
        const std::vector<typename Estimator::Y_t>& Y) {
    const size_t num_samples = X.size();

    Report report;
    report.success = false;
    report.num_trials = 0;

    if (num_samples < Estimator::MinNumSamples()) {
        return report;
    }

    typename SupportMeasurer::Support best_support;
    typename Estimator::M_t best_model;

    bool abort = false;

    const double max_residual = options_.max_error * options_.max_error;

    std::vector<double> residuals(num_samples);

    std::vector<typename Estimator::X_t> X_rand(Estimator::MinNumSamples());
    std::vector<typename Estimator::Y_t> Y_rand(Estimator::MinNumSamples());

    sampler.Initialize(num_samples);

    size_t max_num_trials = options_.max_num_trials;
    max_num_trials = std::min<size_t>(max_num_trials, sampler.MaxNumSamples());
    size_t dyn_max_num_trials = max_num_trials;

    for (report.num_trials = 0; report.num_trials < max_num_trials;
         ++report.num_trials) {
        if (abort) {
            report.num_trials += 1;
            break;
        }

        sampler.SampleXY(X, Y, &X_rand, &Y_rand);

        const std::vector<typename Estimator::M_t> sample_models =
                estimator.Estimate(X_rand, Y_rand);

        for (const auto& sample_model : sample_models) {
            estimator.Residuals(X, Y, sample_model, &residuals);
            const auto support = support_measurer.Evaluate(residuals, max_residual);

            if (support_measurer.Compare(support, best_support)) {
                best_support = support;
                best_model = sample_model;

                if (report.num_trials >= options_.min_num_trials) {
                    dyn_max_num_trials = ComputeNumTrials(
                            best_support.num_inliers, num_samples, options_.confidence);
                }
            }

            if (report.num_trials >= dyn_max_num_trials) {
                abort = true;
                break;
            }
        }
    }

    report.support = best_support;
    report.model = best_model;

    if (report.support.num_inliers < estimator.MinNumSamples()) {
        return report;
    }

    report.success = true;

    estimator.Residuals(X, Y, report.model, &residuals);

    report.inlier_mask.resize(num_samples);
    for (size_t i = 0; i < residuals.size(); ++i) {
        if (residuals[i] <= max_residual) {
            report.inlier_mask[i] = true;
        } else {
            report.inlier_mask[i] = false;
        }
    }

    return report;
}


template <typename Estimator, typename LocalEstimator,
        typename SupportMeasurer = InlierSupportMeasurer,
        typename Sampler = RandomSampler>
class LORANSAC : public RANSAC<Estimator, SupportMeasurer, Sampler> {
public:
    using typename RANSAC<Estimator, SupportMeasurer, Sampler>::Report;

    LORANSAC(const RANSACOptions& options);

    Report Estimate(const std::vector<typename Estimator::X_t>& X,
                    const std::vector<typename Estimator::Y_t>& Y);

    using RANSAC<Estimator, SupportMeasurer, Sampler>::estimator;
    LocalEstimator local_estimator;
    using RANSAC<Estimator, SupportMeasurer, Sampler>::sampler;
    using RANSAC<Estimator, SupportMeasurer, Sampler>::support_measurer;

private:
    using RANSAC<Estimator, SupportMeasurer, Sampler>::options_;
};

template <typename Estimator, typename LocalEstimator, typename SupportMeasurer,
        typename Sampler>
LORANSAC<Estimator, LocalEstimator, SupportMeasurer, Sampler>::LORANSAC(
        const RANSACOptions& options)
        : RANSAC<Estimator, SupportMeasurer, Sampler>(options) {}

template <typename Estimator, typename LocalEstimator, typename SupportMeasurer,
        typename Sampler>
typename LORANSAC<Estimator, LocalEstimator, SupportMeasurer, Sampler>::Report
LORANSAC<Estimator, LocalEstimator, SupportMeasurer, Sampler>::Estimate(
        const std::vector<typename Estimator::X_t>& X,
        const std::vector<typename Estimator::Y_t>& Y) {
    const size_t num_samples = X.size();

    typename RANSAC<Estimator, SupportMeasurer, Sampler>::Report report;
    report.success = false;
    report.num_trials = 0;

    if (num_samples < Estimator::MinNumSamples()) {
        return report;
    }

    typename SupportMeasurer::Support best_support;
    typename Estimator::M_t best_model;
    bool best_model_is_local = false;

    bool abort = false;

    const double max_residual = options_.max_error * options_.max_error;

    std::vector<double> residuals(num_samples);
    std::vector<typename LocalEstimator::X_t> X_inlier;
    std::vector<typename LocalEstimator::Y_t> Y_inlier;

    std::vector<typename Estimator::X_t> X_rand(Estimator::MinNumSamples());
    std::vector<typename Estimator::Y_t> Y_rand(Estimator::MinNumSamples());

    sampler.Initialize(num_samples);

    size_t max_num_trials = options_.max_num_trials;
    max_num_trials = std::min<size_t>(max_num_trials, sampler.MaxNumSamples());
    size_t dyn_max_num_trials = max_num_trials;

    for (report.num_trials = 0; report.num_trials < max_num_trials;
         ++report.num_trials) {
        if (abort) {
            report.num_trials += 1;
            break;
        }

        sampler.SampleXY(X, Y, &X_rand, &Y_rand);

        const std::vector<typename Estimator::M_t> sample_models =
                estimator.Estimate(X_rand, Y_rand);

        for (const auto& sample_model : sample_models) {
            estimator.Residuals(X, Y, sample_model, &residuals);
            const auto support = support_measurer.Evaluate(residuals, max_residual);

            if (support_measurer.Compare(support, best_support)) {
                best_support = support;
                best_model = sample_model;
                best_model_is_local = false;

                if (support.num_inliers >= LocalEstimator::MinNumSamples()) {
                    X_inlier.clear();
                    Y_inlier.clear();
                    X_inlier.reserve(support.num_inliers);
                    Y_inlier.reserve(support.num_inliers);
                    for (size_t i = 0; i < residuals.size(); ++i) {
                        if (residuals[i] <= max_residual) {
                            X_inlier.push_back(X[i]);
                            Y_inlier.push_back(Y[i]);
                        }
                    }

                    const std::vector<typename LocalEstimator::M_t> local_models =
                            local_estimator.Estimate(X_inlier, Y_inlier);

                    for (const auto& local_model : local_models) {
                        local_estimator.Residuals(X, Y, local_model, &residuals);
                        const auto local_support =
                                support_measurer.Evaluate(residuals, max_residual);

                        if (support_measurer.Compare(local_support, support)) {
                            best_support = local_support;
                            best_model = local_model;
                            best_model_is_local = true;
                        }
                    }
                }

                if (report.num_trials >= options_.min_num_trials) {
                    dyn_max_num_trials =
                            RANSAC<Estimator, SupportMeasurer, Sampler>::ComputeNumTrials(
                                    best_support.num_inliers, num_samples, options_.confidence);
                }
            }

            if (report.num_trials >= dyn_max_num_trials) {
                abort = true;
                break;
            }
        }
    }

    report.support = best_support;
    report.model = best_model;

    if (report.support.num_inliers < estimator.MinNumSamples()) {
        return report;
    }

    report.success = true;

    if (best_model_is_local) {
        local_estimator.Residuals(X, Y, report.model, &residuals);
    } else {
        estimator.Residuals(X, Y, report.model, &residuals);
    }

    report.inlier_mask.resize(num_samples, false);

    for (size_t i = 0; i < residuals.size(); ++i) {
        if (residuals[i] <= max_residual) {
            report.inlier_mask[i] = true;
        }
    }

    return report;
}



class P3PEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector3d Y_t;
    typedef Eigen::Matrix3x4d M_t;

    static size_t MinNumSamples() { return 3; }

    static std::vector<M_t> Estimate(const std::vector<X_t>& points2D,
                                     const std::vector<Y_t>& points3D);

    static void Residuals(const std::vector<X_t>& points2D,
                          const std::vector<Y_t>& points3D,
                          const M_t& proj_matrix, std::vector<double>* residuals);
};


class EPnPEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector3d Y_t;
    typedef Eigen::Matrix3x4d M_t;

    static size_t MinNumSamples() { return 4; }

    static std::vector<M_t> Estimate(const std::vector<X_t>& points2D,
                                     const std::vector<Y_t>& points3D);

    static void Residuals(const std::vector<X_t>& points2D,
                          const std::vector<Y_t>& points3D,
                          const M_t& proj_matrix, std::vector<double>* residuals);

private:
    bool ComputePose(const std::vector<Eigen::Vector2d>& points2D,
                     const std::vector<Eigen::Vector3d>& points3D,
                     Eigen::Matrix3x4d* proj_matrix);

    void ChooseControlPoints();
    bool ComputeBarycentricCoordinates();

    Eigen::Matrix<double, Eigen::Dynamic, 12> ComputeM();
    Eigen::Matrix<double, 6, 10> ComputeL6x10(
            const Eigen::Matrix<double, 12, 12>& Ut);
    Eigen::Matrix<double, 6, 1> ComputeRho();

    void FindBetasApprox1(const Eigen::Matrix<double, 6, 10>& L_6x10,
                          const Eigen::Matrix<double, 6, 1>& rho,
                          Eigen::Vector4d* betas);
    void FindBetasApprox2(const Eigen::Matrix<double, 6, 10>& L_6x10,
                          const Eigen::Matrix<double, 6, 1>& rho,
                          Eigen::Vector4d* betas);
    void FindBetasApprox3(const Eigen::Matrix<double, 6, 10>& L_6x10,
                          const Eigen::Matrix<double, 6, 1>& rho,
                          Eigen::Vector4d* betas);

    void RunGaussNewton(const Eigen::Matrix<double, 6, 10>& L_6x10,
                        const Eigen::Matrix<double, 6, 1>& rho,
                        Eigen::Vector4d* betas);

    double ComputeRT(const Eigen::Matrix<double, 12, 12>& Ut,
                     const Eigen::Vector4d& betas, Eigen::Matrix3d* R,
                     Eigen::Vector3d* t);

    void ComputeCcs(const Eigen::Vector4d& betas,
                    const Eigen::Matrix<double, 12, 12>& Ut);
    void ComputePcs();

    void SolveForSign();

    void EstimateRT(Eigen::Matrix3d* R, Eigen::Vector3d* t);

    double ComputeTotalReprojectionError(const Eigen::Matrix3d& R,
                                         const Eigen::Vector3d& t);

    std::vector<Eigen::Vector2d> points2D_;
    std::vector<Eigen::Vector3d> points3D_;
    std::vector<Eigen::Vector3d> pcs_;
    std::vector<Eigen::Vector4d> alphas_;
    std::array<Eigen::Vector3d, 4> cws_;
    std::array<Eigen::Vector3d, 4> ccs_;
};



class EssentialMatrixFivePointEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    static size_t MinNumSamples() { return 5; }

    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                     const std::vector<Y_t>& points2);

    static void Residuals(const std::vector<X_t>& points1,
                          const std::vector<Y_t>& points2, const M_t& E,
                          std::vector<double>* residuals);
};

class EssentialMatrixEightPointEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    static size_t MinNumSamples() { return 8; }

    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                     const std::vector<Y_t>& points2);

    static void Residuals(const std::vector<X_t>& points1,
                          const std::vector<Y_t>& points2, const M_t& E,
                          std::vector<double>* residuals);
};


class FundamentalMatrixSevenPointEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    static size_t MinNumSamples() { return 7; }

    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                     const std::vector<Y_t>& points2);

    static void Residuals(const std::vector<X_t>& points1,
                          const std::vector<Y_t>& points2, const M_t& F,
                          std::vector<double>* residuals);
};

class FundamentalMatrixEightPointEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    static size_t MinNumSamples() { return 8; }

    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                     const std::vector<Y_t>& points2);

    static void Residuals(const std::vector<X_t>& points1,
                          const std::vector<Y_t>& points2, const M_t& F,
                          std::vector<double>* residuals);
};


class HomographyMatrixEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    static size_t MinNumSamples() { return 4; }

    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                     const std::vector<Y_t>& points2);

    static void Residuals(const std::vector<X_t>& points1,
                          const std::vector<Y_t>& points2, const M_t& H,
                          std::vector<double>* residuals);
};


template <int kDim>
class TranslationTransformEstimator {
public:
    typedef Eigen::Matrix<double, kDim, 1> X_t;
    typedef Eigen::Matrix<double, kDim, 1> Y_t;
    typedef Eigen::Matrix<double, kDim, 1> M_t;

    static size_t MinNumSamples() { return 1; }

    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                     const std::vector<Y_t>& points2);

    static void Residuals(const std::vector<X_t>& points1,
                          const std::vector<Y_t>& points2, const M_t& translation,
                          std::vector<double>* residuals);
};

template <int kDim>
std::vector<typename TranslationTransformEstimator<kDim>::M_t>
TranslationTransformEstimator<kDim>::Estimate(const std::vector<X_t>& points1,
                                              const std::vector<Y_t>& points2) {
    X_t mean_src = X_t::Zero();
    Y_t mean_dst = Y_t::Zero();

    for (size_t i = 0; i < points1.size(); ++i) {
        mean_src += points1[i];
        mean_dst += points2[i];
    }

    mean_src /= points1.size();
    mean_dst /= points2.size();

    std::vector<M_t> models(1);
    models[0] = mean_dst - mean_src;

    return models;
}

template <int kDim>
void TranslationTransformEstimator<kDim>::Residuals(
        const std::vector<X_t>& points1, const std::vector<Y_t>& points2,
        const M_t& translation, std::vector<double>* residuals) {
    residuals->resize(points1.size());

    for (size_t i = 0; i < points1.size(); ++i) {
        const M_t diff = points2[i] - points1[i] - translation;
        (*residuals)[i] = diff.squaredNorm();
    }
}

class TriangulationEstimator {
public:
    enum class ResidualType {
        ANGULAR_ERROR,
        REPROJECTION_ERROR,
    };

    struct PointData {
        PointData() {}
        PointData(const Eigen::Vector2d& point_, const Eigen::Vector2d& point_N_)
                : point(point_), point_normalized(point_N_) {}
        Eigen::Vector2d point;
        Eigen::Vector2d point_normalized;
    };

    struct PoseData {
        PoseData() : camera(nullptr) {}
        PoseData(const Eigen::Matrix3x4d& proj_matrix_,
                 const Eigen::Vector3d& pose_, const Camera* camera_)
                : proj_matrix(proj_matrix_), proj_center(pose_), camera(camera_) {}
        Eigen::Matrix3x4d proj_matrix;
        Eigen::Vector3d proj_center;
        const Camera* camera;
    };

    typedef PointData X_t;
    typedef PoseData Y_t;
    typedef Eigen::Vector3d M_t;

    void SetMinTriAngle(const double min_tri_angle);
    void SetResidualType(const ResidualType residual_type);

    static size_t MinNumSamples() { return 2; }

    std::vector<M_t> Estimate(const std::vector<X_t>& point_data,
                              const std::vector<Y_t>& pose_data) const;

    void Residuals(const std::vector<X_t>& point_data,
                   const std::vector<Y_t>& pose_data, const M_t& xyz,
                   std::vector<double>* residuals) const;

private:
    ResidualType residual_type_ = ResidualType::REPROJECTION_ERROR;
    double min_tri_angle_ = 0.0;
};

struct EstimateTriangulationOptions {
    double min_tri_angle = 0.0;

    TriangulationEstimator::ResidualType residual_type =
            TriangulationEstimator::ResidualType::ANGULAR_ERROR;

    RANSACOptions ransac_options;

    void Check() const {
        ransac_options.Check();
    }
};


bool EstimateTriangulation(
        const EstimateTriangulationOptions& options,
        const std::vector<TriangulationEstimator::PointData>& point_data,
        const std::vector<TriangulationEstimator::PoseData>& pose_data,
        std::vector<bool>* inlier_mask, Eigen::Vector3d* xyz);

#endif //INC_3D_RECONSTRUCTION_ESTIMATORS_H
