#ifndef INC_3D_RECONSTRUCTION_GEOMETRY_H
#define INC_3D_RECONSTRUCTION_GEOMETRY_H

#include "estimators.h"

struct TwoViewGeometry {
    enum ConfigurationType {
        UNDEFINED = 0,
                DEGENERATE = 1,
                CALIBRATED = 2,
                UNCALIBRATED = 3,
                PLANAR = 4,
                PANORAMIC = 5,
                PLANAR_OR_PANORAMIC = 6,
                WATERMARK = 7,
                MULTIPLE = 8,
    };

    struct Options {
        size_t min_num_inliers = 15;

        double min_E_F_inlier_ratio = 0.95;

        double max_H_inlier_ratio = 0.8;

        double watermark_min_inlier_ratio = 0.7;

        double watermark_border_size = 0.1;

        bool detect_watermark = true;

        bool multiple_ignore_watermark = true;

        RANSACOptions ransac_options;

        void Check() const {
            ransac_options.Check();
        }
    };

    TwoViewGeometry()
            : config(ConfigurationType::UNDEFINED),
              E(Eigen::Matrix3d::Zero()),
              F(Eigen::Matrix3d::Zero()),
              H(Eigen::Matrix3d::Zero()),
              qvec(Eigen::Vector4d::Zero()),
              tvec(Eigen::Vector3d::Zero()),
              tri_angle(0),
              E_num_inliers(0),
              F_num_inliers(0),
              H_num_inliers(0) {}

    void Estimate(const Camera& camera1,
                  const std::vector<Eigen::Vector2d>& points1,
                  const Camera& camera2,
                  const std::vector<Eigen::Vector2d>& points2,
                  const FeatureMatches& matches, const Options& options);

    void EstimateMultiple(const Camera& camera1,
                          const std::vector<Eigen::Vector2d>& points1,
                          const Camera& camera2,
                          const std::vector<Eigen::Vector2d>& points2,
                          const FeatureMatches& matches, const Options& options);

    void EstimateWithRelativePose(const Camera& camera1,
                                  const std::vector<Eigen::Vector2d>& points1,
                                  const Camera& camera2,
                                  const std::vector<Eigen::Vector2d>& points2,
                                  const FeatureMatches& matches,
                                  const Options& options);

    void EstimateCalibrated(const Camera& camera1,
                            const std::vector<Eigen::Vector2d>& points1,
                            const Camera& camera2,
                            const std::vector<Eigen::Vector2d>& points2,
                            const FeatureMatches& matches,
                            const Options& options);

    void EstimateUncalibrated(const Camera& camera1,
                              const std::vector<Eigen::Vector2d>& points1,
                              const Camera& camera2,
                              const std::vector<Eigen::Vector2d>& points2,
                              const FeatureMatches& matches,
                              const Options& options);

    static bool DetectWatermark(const Camera& camera1,
                                const std::vector<Eigen::Vector2d>& points1,
                                const Camera& camera2,
                                const std::vector<Eigen::Vector2d>& points2,
                                const size_t num_inliers,
                                const std::vector<bool>& inlier_mask,
                                const Options& options);

    int config;

    Eigen::Matrix3d E;
    Eigen::Matrix3d F;
    Eigen::Matrix3d H;

    Eigen::Vector4d qvec;
    Eigen::Vector3d tvec;

    FeatureMatches inlier_matches;
    std::vector<bool> inlier_mask;

    double tri_angle;

    size_t E_num_inliers;
    size_t F_num_inliers;
    size_t H_num_inliers;
};


#endif //INC_3D_RECONSTRUCTION_GEOMETRY_H
