#include "camera.h"

int CameraModelNameToId(const std::string& name) {
    std::string uppercast_name = name;
    boost::to_upper(uppercast_name);
    if (uppercast_name == "RADIAL") {
        return RadialCameraModel::model_id;
    }
    else if (uppercast_name == "PINHOLE") {
        return PinholeCameraModel::model_id;
    }
    return kInvalidCameraModelId;
}

std::string CameraModelIdToName(const int model_id) {
    if (model_id == RadialCameraModel::model_id) {
        return "RADIAL";
    }
    else if (model_id == PinholeCameraModel::model_id) {
        return "PINHOLE";
    }
    return "INVALID_CAMERA_MODEL";
}

void CameraModelInitializeParams(
        const int model_id,
        const double focal_length,
        const size_t width,
        const size_t height,
        std::vector<double>* params
) {
    if (model_id == RadialCameraModel::model_id) {
        params->resize(RadialCameraModel::num_params);
        for (const int idx : RadialCameraModel::focal_length_idxs) {
            (*params)[idx] = focal_length;
        }
        (*params)[RadialCameraModel::principal_point_idxs[0]] = width / 2.0;
        (*params)[RadialCameraModel::principal_point_idxs[1]] = height / 2.0;
        for (const int idx : RadialCameraModel::extra_params_idxs) {
            (*params)[idx] = 0;
        }
    }
    else if (model_id == PinholeCameraModel::model_id) {
        params->resize(PinholeCameraModel::num_params);
        for (const int idx : PinholeCameraModel::focal_length_idxs) {
            (*params)[idx] = focal_length;
        }
        (*params)[PinholeCameraModel::principal_point_idxs[0]] = width / 2.0;
        (*params)[PinholeCameraModel::principal_point_idxs[1]] = height / 2.0;
        for (const int idx : PinholeCameraModel::extra_params_idxs) {
            (*params)[idx] = 0;
        }
    }
    else {
        throw std::domain_error("Camera model does not exist");
    }
}

void CameraModelWorldToImage(const int model_id,
                             const std::vector<double>& params, const double u,
                             const double v, double* x, double* y) {
    if (model_id == 1)
        RadialCameraModel::WorldToImage(params.data(), u, v, x, y);
    else if (model_id == 2)
        PinholeCameraModel::WorldToImage(params.data(), u, v, x, y);
    else
        throw std::domain_error("Camera model does not exist");
}

void CameraModelImageToWorld(const int model_id,
                             const std::vector<double>& params, const double x,
                             const double y, double* u, double* v) {
    if (model_id == 1)
        RadialCameraModel::ImageToWorld(params.data(), x, y, u, v);
    else if (model_id == 2)
        PinholeCameraModel::ImageToWorld(params.data(), x, y, u, v);
    else
        throw std::domain_error("Camera model does not exist");
}

double CameraModelImageToWorldThreshold(const int model_id,
                                        const std::vector<double>& params,
                                        const double threshold) {
    if (model_id == 1)
        return RadialCameraModel::ImageToWorldThreshold(params.data(), threshold);
    else if (model_id == 2)
        return PinholeCameraModel::ImageToWorldThreshold(params.data(), threshold);
    else
        throw std::domain_error("Camera model does not exist");
}

std::string CameraModelParamsInfo(const int model_id) {
    if (model_id == RadialCameraModel::model_id) {
        return RadialCameraModel::params_info;
    }
    else if (model_id == PinholeCameraModel::model_id) {
        return PinholeCameraModel::params_info;
    }
    return "Camera model does not exist";
}

std::vector<size_t> CameraModelFocalLengthIdxs(const int model_id) {
    if (model_id == RadialCameraModel::model_id) {
        return RadialCameraModel::focal_length_idxs;
    }
    else if (model_id == PinholeCameraModel::model_id) {
        return PinholeCameraModel::focal_length_idxs;
    }
    return std::vector<size_t>{};
}

std::vector<size_t> CameraModelPrincipalPointIdxs(const int model_id) {
    if (model_id == RadialCameraModel::model_id) {
        return RadialCameraModel::principal_point_idxs;
    }
    else if (model_id == PinholeCameraModel::model_id) {
        return PinholeCameraModel::principal_point_idxs;
    }
    return std::vector<size_t>{};
}

std::vector<size_t> CameraModelExtraParamsIdxs(const int model_id) {
    if (model_id == RadialCameraModel::model_id) {
        return RadialCameraModel::extra_params_idxs;
    }
    else if (model_id == PinholeCameraModel::model_id) {
        return PinholeCameraModel::extra_params_idxs;
    }
    return std::vector<size_t>{};
}

bool CameraModelVerifyParams(const int model_id, const std::vector<double>& params) {
    if (model_id == RadialCameraModel::model_id && params.size() == RadialCameraModel::num_params) {
        return true;
    }
    else if (model_id == PinholeCameraModel::model_id && params.size() == PinholeCameraModel::num_params) {
        return true;
    }
    return false;
}

bool CameraModelHasBogusParams(
        const int model_id,
        const std::vector<double>& params,
        const size_t width, const size_t height,
        const double min_focal_length_ratio,
        const double max_focal_length_ratio,
        const double max_extra_param
) {
    if (model_id == RadialCameraModel::model_id) {
        return RadialCameraModel::HasBogusParams(params,
                                                 width,
                                                 height,
                                                 min_focal_length_ratio,
                                                 max_focal_length_ratio,
                                                 max_extra_param);
    }
    if (model_id == PinholeCameraModel::model_id) {
        return PinholeCameraModel::HasBogusParams(params,
                                                  width,
                                                  height,
                                                  min_focal_length_ratio,
                                                  max_focal_length_ratio,
                                                  max_extra_param);
    }
    return false;
}

std::vector<Eigen::Vector2d> FeatureKeypointsToPointsVector(
        const FeatureKeypoints& keypoints) {
    std::vector<Eigen::Vector2d> points(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i) {
        points[i] = Eigen::Vector2d(keypoints[i].x, keypoints[i].y);
    }
    return points;
}

Eigen::MatrixXf L2NormalizeFeatureDescriptors(
        const Eigen::MatrixXf& descriptors) {
    return descriptors.rowwise().normalized();
}

Eigen::MatrixXf L1RootNormalizeFeatureDescriptors(
        const Eigen::MatrixXf& descriptors) {
    Eigen::MatrixXf descriptors_normalized(descriptors.rows(),
                                           descriptors.cols());
    for (Eigen::MatrixXf::Index r = 0; r < descriptors.rows(); ++r) {
        const float norm = descriptors.row(r).lpNorm<1>();
        descriptors_normalized.row(r) = descriptors.row(r) / norm;
        descriptors_normalized.row(r) =
                descriptors_normalized.row(r).array().sqrt();
    }
    return descriptors_normalized;
}

FeatureDescriptors FeatureDescriptorsToUnsignedByte(
        const Eigen::MatrixXf& descriptors) {
    FeatureDescriptors descriptors_unsigned_byte(descriptors.rows(),
                                                 descriptors.cols());
    for (Eigen::MatrixXf::Index r = 0; r < descriptors.rows(); ++r) {
        for (Eigen::MatrixXf::Index c = 0; c < descriptors.cols(); ++c) {
            const float scaled_value = std::round(512.0f * descriptors(r, c));
            descriptors_unsigned_byte(r, c) =
                    static_cast<uint8_t>(std::min(255.0f, scaled_value));
        }
    }
    return descriptors_unsigned_byte;
}

Camera::Camera()
        : camera_id_(kInvalidCameraId),
          model_id_(kInvalidCameraModelId),
          width_(0),
          height_(0),
          prior_focal_length_(false) { }

std::string Camera::ModelName() const { return CameraModelIdToName(model_id_); }

void Camera::SetModelIdFromName(const std::string& name) {
    model_id_ = CameraModelNameToId(name);
}

std::vector<size_t> Camera::FocalLengthIdxs() const {
    return CameraModelFocalLengthIdxs(model_id_);
}

std::vector<size_t> Camera::PrincipalPointIdxs() const {
    return CameraModelPrincipalPointIdxs(model_id_);
}

std::vector<size_t> Camera::ExtraParamsIdxs() const {
    return CameraModelExtraParamsIdxs(model_id_);
}

Eigen::Matrix3d Camera::CalibrationMatrix() const {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();

    const std::vector<size_t>& idxs = FocalLengthIdxs();
    if (idxs.size() == 1) {
        K(0, 0) = params_[idxs[0]];
        K(1, 1) = params_[idxs[0]];
    } else if (idxs.size() == 2) {
        K(0, 0) = params_[idxs[0]];
        K(1, 1) = params_[idxs[1]];
    } else {
        std::cerr << "Camera model must either have 1 or 2 focal length parameters.";
    }

    K(0, 2) = PrincipalPointX();
    K(1, 2) = PrincipalPointY();

    return K;
}

std::string Camera::ParamsInfo() const {
    return CameraModelParamsInfo(model_id_);
}

double Camera::MeanFocalLength() const {
    const auto& focal_length_idxs = FocalLengthIdxs();
    double focal_length = 0;
    for (const auto idx : focal_length_idxs) {
        focal_length += params_[idx];
    }
    return focal_length / focal_length_idxs.size();
}

double Camera::FocalLength() const {
    const std::vector<size_t>& idxs = FocalLengthIdxs();
    return params_[idxs[0]];
}

double Camera::FocalLengthX() const {
    const std::vector<size_t>& idxs = FocalLengthIdxs();
    return params_[idxs[0]];
}

double Camera::FocalLengthY() const {
    const std::vector<size_t>& idxs = FocalLengthIdxs();
    return params_[idxs[1]];
}

void Camera::SetFocalLength(const double focal_length) {
    const std::vector<size_t>& idxs = FocalLengthIdxs();
    params_[idxs[0]] = focal_length;
}

void Camera::SetFocalLengthX(const double focal_length_x) {
    const std::vector<size_t>& idxs = FocalLengthIdxs();
    params_[idxs[0]] = focal_length_x;
}

void Camera::SetFocalLengthY(const double focal_length_y) {
    const std::vector<size_t>& idxs = FocalLengthIdxs();
    params_[idxs[1]] = focal_length_y;
}

double Camera::PrincipalPointX() const {
    const std::vector<size_t>& idxs = PrincipalPointIdxs();
    return params_[idxs[0]];
}

double Camera::PrincipalPointY() const {
    const std::vector<size_t>& idxs = PrincipalPointIdxs();
    return params_[idxs[1]];
}

void Camera::SetPrincipalPointX(const double ppx) {
    const std::vector<size_t>& idxs = PrincipalPointIdxs();
    params_[idxs[0]] = ppx;
}

void Camera::SetPrincipalPointY(const double ppy) {
    const std::vector<size_t>& idxs = PrincipalPointIdxs();
    params_[idxs[1]] = ppy;
}

std::string Camera::ParamsToString() const { return VectorToCSV(params_); }

bool Camera::SetParamsFromString(const std::string& string) {
    params_ = CSVToVector<double>(string);
    return VerifyParams();
}

bool Camera::VerifyParams() const {
    return CameraModelVerifyParams(model_id_, params_);
}

bool Camera::HasBogusParams(
        const double min_focal_length_ratio,
        const double max_focal_length_ratio,
        const double max_extra_param) const {

    return CameraModelHasBogusParams(model_id_, params_, width_, height_,
                                     min_focal_length_ratio,
                                     max_focal_length_ratio, max_extra_param);
}

void Camera::InitializeWithId(const int model_id,
                              const double focal_length,
                              const size_t width,
                              const size_t height) {

    this->model_id_ = model_id;
    this->width_ = width;
    this->height_ = height;
    CameraModelInitializeParams(model_id, focal_length, width, height, &params_);
}

void Camera::InitializeWithName(const std::string& model_name,
                                const double focal_length, const size_t width,
                                const size_t height) {

    InitializeWithId(CameraModelNameToId(model_name), focal_length, width, height);
}

Eigen::Vector2d Camera::ImageToWorld(const Eigen::Vector2d& image_point) const {
    Eigen::Vector2d world_point;

    CameraModelImageToWorld(model_id_, params_,
                            image_point(0), image_point(1),
                            &world_point(0), &world_point(1));
    return world_point;
}

double Camera::ImageToWorldThreshold(const double threshold) const {
    return CameraModelImageToWorldThreshold(model_id_, params_, threshold);
}

Eigen::Vector2d Camera::WorldToImage(const Eigen::Vector2d& world_point) const {
    Eigen::Vector2d image_point;

    CameraModelWorldToImage(model_id_, params_,
                            world_point(0), world_point(1),
                            &image_point(0), &image_point(1));
    return image_point;
}

void Camera::Rescale(const double scale) {
    const double scale_x = std::round(scale * width_) / static_cast<double>(width_);
    const double scale_y = std::round(scale * height_) / static_cast<double>(height_);
    width_ = static_cast<size_t>(std::round(scale * width_));
    height_ = static_cast<size_t>(std::round(scale * height_));
    SetPrincipalPointX(scale_x * PrincipalPointX());
    SetPrincipalPointY(scale_y * PrincipalPointY());
    if (FocalLengthIdxs().size() == 1) {
        SetFocalLength((scale_x + scale_y) / 2.0 * FocalLength());
    } else if (FocalLengthIdxs().size() == 2) {
        SetFocalLengthX(scale_x * FocalLengthX());
        SetFocalLengthY(scale_y * FocalLengthY());
    } else {
        std::cerr << "Camera model must either have 1 or 2 focal length parameters.";
    }
}


camera_t Camera::CameraId() const { return camera_id_; }

void Camera::SetCameraId(const camera_t camera_id) { camera_id_ = camera_id; }

int Camera::ModelId() const { return model_id_; }

void Camera::SetModelId(const int model_id) { model_id_ = model_id; }

size_t Camera::Width() const { return width_; }

size_t Camera::Height() const { return height_; }

void Camera::SetWidth(const size_t width) { width_ = width; }

void Camera::SetHeight(const size_t height) { height_ = height; }

bool Camera::HasPriorFocalLength() const { return prior_focal_length_; }

void Camera::SetPriorFocalLength(const bool prior) {
    prior_focal_length_ = prior;
}

size_t Camera::NumParams() const { return params_.size(); }

const std::vector<double>& Camera::Params() const { return params_; }

std::vector<double>& Camera::Params() { return params_; }

double Camera::Params(const size_t idx) const { return params_[idx]; }

double& Camera::Params(const size_t idx) { return params_[idx]; }

const double* Camera::ParamsData() const { return params_.data(); }

double* Camera::ParamsData() { return params_.data(); }

void Camera::SetParams(const std::vector<double>& params) { params_ = params; }