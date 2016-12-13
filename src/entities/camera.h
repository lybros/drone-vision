#ifndef INC_3D_RECONSTRUCTION_CAMERA_H
#define INC_3D_RECONSTRUCTION_CAMERA_H

#include "../utils.h"
#include "radial_camera_model.h"
#include "pinhole_camera_model.h"

static const int kInvalidCameraModelId = -1;

int CameraModelNameToId(const std::string& name);

std::string CameraModelIdToName(const int model_id);

void CameraModelInitializeParams(
        const int model_id, const double focal_length,
        const size_t width,
        const size_t height,
        std::vector<double>* params
);

std::string CameraModelParamsInfo(const int model_id);

std::vector<size_t> CameraModelFocalLengthIdxs(const int model_id);

std::vector<size_t> CameraModelPrincipalPointIdxs(const int model_id);

std::vector<size_t> CameraModelExtraParamsIdxs(const int model_id);

bool CameraModelVerifyParams(
        const int model_id,
        const std::vector<double>& params
);

bool CameraModelHasBogusParams(
        const int model_id,
        const std::vector<double>& params,
        const size_t width,
        const size_t height,
        const double min_focal_length_ratio,
        const double max_focal_length_ratio,
        const double max_extra_param
);

void CameraModelWorldToImage(
        const int model_id,
        const std::vector<double>& params,
        const double u,
        const double v,
        double* x,
        double* y
);

void CameraModelImageToWorld(
        const int model_id,
        const std::vector<double>& params,
        const double x,
        const double y,
        double* u,
        double* v
);

double CameraModelImageToWorldThreshold(
        const int model_id,
        const std::vector<double>& params,
        const double threshold
);

class Camera {
public:
    Camera();

    camera_t CameraId() const;

    void SetCameraId(const camera_t camera_id);

    int ModelId() const;

    std::string ModelName() const;

    void SetModelId(const int model_id);

    void SetModelIdFromName(const std::string& name);

    size_t Width() const;

    size_t Height() const;

    void SetWidth(const size_t width);

    void SetHeight(const size_t height);

    double MeanFocalLength() const;

    double FocalLength() const;

    double FocalLengthX() const;

    double FocalLengthY() const;

    void SetFocalLength(const double focal_length);

    void SetFocalLengthX(const double focal_length_x);

    void SetFocalLengthY(const double focal_length_y);

    bool HasPriorFocalLength() const;

    void SetPriorFocalLength(const bool prior);

    double PrincipalPointX() const;

    double PrincipalPointY() const;

    void SetPrincipalPointX(const double ppx);

    void SetPrincipalPointY(const double ppy);

    std::vector<size_t> FocalLengthIdxs() const;

    std::vector<size_t> PrincipalPointIdxs() const;

    std::vector<size_t> ExtraParamsIdxs() const;

    Eigen::Matrix3d CalibrationMatrix() const;

    std::string ParamsInfo() const;

    size_t NumParams() const;

    const std::vector<double>& Params() const;

    std::vector<double>& Params();

    double Params(const size_t idx) const;

    double& Params(const size_t idx);

    const double* ParamsData() const;

    double* ParamsData();

    void SetParams(const std::vector<double>& params);

    std::string ParamsToString() const;

    bool SetParamsFromString(const std::string& string);

    bool VerifyParams() const;

    bool HasBogusParams(
            const double min_focal_length_ratio,
            const double max_focal_length_ratio,
            const double max_extra_param
    ) const;

    // Calling this method after reading EXIF data from an image.
    void InitializeWithId(
            const int model_id,
            const double focal_length,
            const size_t width,
            const size_t height
    );

    void InitializeWithName(
            const std::string& model_name,
            const double focal_length,
            const size_t width,
            const size_t height
    );

    Eigen::Vector2d ImageToWorld(const Eigen::Vector2d& image_point) const;

    double ImageToWorldThreshold(const double threshold) const;

    Eigen::Vector2d WorldToImage(const Eigen::Vector2d& world_point) const;

    void Rescale(const double scale);

private:
    camera_t camera_id_;

    int model_id_;

    size_t width_;
    size_t height_;

    std::vector<double> params_;

    bool prior_focal_length_;
};


struct FeatureKeypoint {
    float x = 0.0f;
    float y = 0.0f;

    float scale_or_size = 0.0f;
    float orientation_or_angle = 0.0f;
};

struct FeatureMatch {
    point2D_t point2D_idx1 = kInvalidPoint2DIdx;

    point2D_t point2D_idx2 = kInvalidPoint2DIdx;
};

typedef std::vector<FeatureKeypoint> FeatureKeypoints;
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> FeatureDescriptors;
typedef std::vector<FeatureMatch> FeatureMatches;

std::vector<Eigen::Vector2d> FeatureKeypointsToPointsVector(const FeatureKeypoints& keypoints);

Eigen::MatrixXf L2NormalizeFeatureDescriptors(const Eigen::MatrixXf& descriptors);

Eigen::MatrixXf L1RootNormalizeFeatureDescriptors(const Eigen::MatrixXf& descriptors);

FeatureDescriptors FeatureDescriptorsToUnsignedByte(const Eigen::MatrixXf& descriptors);

#endif //INC_3D_RECONSTRUCTION_CAMERA_H
