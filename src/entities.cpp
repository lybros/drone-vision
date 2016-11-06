#include "entities.h"

const std::string RadialCameraModel::params_info = RadialCameraModel::InitializeParamsInfo();
const std::vector<size_t> RadialCameraModel::focal_length_idxs = RadialCameraModel::InitializeFocalLengthIdxs();
const std::vector<size_t> RadialCameraModel::principal_point_idxs = RadialCameraModel::InitializePrincipalPointIdxs();
const std::vector<size_t> RadialCameraModel::extra_params_idxs = RadialCameraModel::InitializeExtraParamsIdxs();

const std::string PinholeCameraModel::params_info = PinholeCameraModel::InitializeParamsInfo();
const std::vector<size_t> PinholeCameraModel::focal_length_idxs = PinholeCameraModel::InitializeFocalLengthIdxs();
const std::vector<size_t> PinholeCameraModel::principal_point_idxs = PinholeCameraModel::InitializePrincipalPointIdxs();
const std::vector<size_t> PinholeCameraModel::extra_params_idxs = PinholeCameraModel::InitializeExtraParamsIdxs();

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

void CameraModelInitializeParams(const int model_id, const double focal_length,
                                 const size_t width, const size_t height,
                                 std::vector<double>* params) {
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

bool CameraModelVerifyParams(const int model_id,
                             const std::vector<double>& params) {
    if (model_id == RadialCameraModel::model_id && params.size() == RadialCameraModel::num_params) {
        return true;
    }
    else if (model_id == PinholeCameraModel::model_id && params.size() == PinholeCameraModel::num_params) {
        return true;
    }
    return false;
}

bool CameraModelHasBogusParams(const int model_id,
                               const std::vector<double>& params,
                               const size_t width, const size_t height,
                               const double min_focal_length_ratio,
                               const double max_focal_length_ratio,
                               const double max_extra_param) {
    if (model_id == RadialCameraModel::model_id) {
        return RadialCameraModel::HasBogusParams(params, width, height, min_focal_length_ratio, max_focal_length_ratio,
                                                 max_extra_param);
    }
    if (model_id == PinholeCameraModel::model_id) {
        return PinholeCameraModel::HasBogusParams(params, width, height, min_focal_length_ratio, max_focal_length_ratio,
                                                  max_extra_param);
    }
    return false;
}

std::string RadialCameraModel::InitializeParamsInfo() {
    return "f, cx, cy, k1, k2";
}

std::vector<size_t> RadialCameraModel::InitializeFocalLengthIdxs() {
    std::vector<size_t> idxs(1);
    idxs[0] = 0;
    return idxs;
}

std::vector<size_t> RadialCameraModel::InitializePrincipalPointIdxs() {
    std::vector<size_t> idxs(2);
    idxs[0] = 1;
    idxs[1] = 2;
    return idxs;
}

std::vector<size_t> RadialCameraModel::InitializeExtraParamsIdxs() {
    std::vector<size_t> idxs(2);
    idxs[0] = 3;
    idxs[1] = 4;
    return idxs;
}


std::string PinholeCameraModel::InitializeParamsInfo() {
    return "fx, fy, cx, cy";
}

std::vector<size_t> PinholeCameraModel::InitializeFocalLengthIdxs() {
    std::vector<size_t> idxs(2);
    idxs[0] = 0;
    idxs[1] = 1;
    return idxs;
}

std::vector<size_t> PinholeCameraModel::InitializePrincipalPointIdxs() {
    std::vector<size_t> idxs(2);
    idxs[0] = 2;
    idxs[1] = 3;
    return idxs;
}

std::vector<size_t> PinholeCameraModel::InitializeExtraParamsIdxs() {
    std::vector<size_t> idxs;
    return idxs;
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


Track::Track() { }

TrackElement::TrackElement()
        : image_id(kInvalidImageId), point2D_idx(kInvalidPoint2DIdx) { }

TrackElement::TrackElement(const image_t image_id, const point2D_t point2D_idx)
        : image_id(image_id), point2D_idx(point2D_idx) { }

void Track::DeleteElement(const image_t image_id, const point2D_t point2D_idx) {
    elements_.erase(
            std::remove_if(elements_.begin(), elements_.end(),
                           [image_id, point2D_idx](const TrackElement& element) {
                               return element.image_id == image_id &&
                                      element.point2D_idx == point2D_idx;
                           }),
            elements_.end());
}

size_t Track::Length() const { return elements_.size(); }

const std::vector<TrackElement>& Track::Elements() const { return elements_; }

void Track::SetElements(const std::vector<TrackElement>& elements) {
    elements_ = elements;
}

const TrackElement& Track::Element(const size_t idx) const {
    return elements_.at(idx);
}

TrackElement& Track::Element(const size_t idx) { return elements_.at(idx); }

void Track::SetElement(const size_t idx, const TrackElement& element) {
    elements_.at(idx) = element;
}

void Track::AddElement(const TrackElement& element) {
    elements_.push_back(element);
}

void Track::AddElement(const image_t image_id, const point2D_t point2D_idx) {
    elements_.emplace_back(image_id, point2D_idx);
}

void Track::AddElements(const std::vector<TrackElement>& elements) {
    elements_.insert(elements_.end(), elements.begin(), elements.end());
}

void Track::DeleteElement(const size_t idx) {
    elements_.erase(elements_.begin() + idx);
}

void Track::Reserve(const size_t num_elements) {
    elements_.reserve(num_elements);
}

void Track::Compress() { elements_.shrink_to_fit(); }


Point_3D::Point_3D() : xyz_(0.0, 0.0, 0.0), color_(0, 0, 0), error_(-1.0) { }

const Eigen::Vector3d& Point_3D::XYZ() const { return xyz_; }

Eigen::Vector3d& Point_3D::XYZ() { return xyz_; }

double Point_3D::XYZ(const size_t idx) const { return xyz_(idx); }

double& Point_3D::XYZ(const size_t idx) { return xyz_(idx); }

double Point_3D::X() const { return xyz_.x(); }

double Point_3D::Y() const { return xyz_.y(); }

double Point_3D::Z() const { return xyz_.z(); }

void Point_3D::SetXYZ(const Eigen::Vector3d& xyz) { xyz_ = xyz; }

const Eigen::Vector3ub& Point_3D::Color() const { return color_; }

Eigen::Vector3ub& Point_3D::Color() { return color_; }

uint8_t Point_3D::Color(const size_t idx) const { return color_(idx); }

uint8_t& Point_3D::Color(const size_t idx) { return color_(idx); }

void Point_3D::SetColor(const Eigen::Vector3ub& color) { color_ = color; }

double Point_3D::Error() const { return error_; }

bool Point_3D::HasError() const { return error_ != -1.0; }

void Point_3D::SetError(const double error) { error_ = error; }

const class Track& Point_3D::Track() const { return track_; }

class Track& Point_3D::Track() {
    return track_;
}

void Point_3D::SetTrack(const class Track& track) { track_ = track; }


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

bool Camera::HasBogusParams(const double min_focal_length_ratio,
                            const double max_focal_length_ratio,
                            const double max_extra_param) const {
    return CameraModelHasBogusParams(model_id_, params_, width_, height_,
                                     min_focal_length_ratio,
                                     max_focal_length_ratio, max_extra_param);
}

void Camera::InitializeWithId(const int model_id, const double focal_length,
                              const size_t width, const size_t height) {
    this->model_id_ = model_id;
    this->width_ = width;
    this->height_ = height;
    CameraModelInitializeParams(model_id, focal_length, width, height, &params_);
}

void Camera::InitializeWithName(const std::string& model_name,
                                const double focal_length, const size_t width,
                                const size_t height) {
    InitializeWithId(CameraModelNameToId(model_name), focal_length, width,
                     height);
}

Eigen::Vector2d Camera::ImageToWorld(const Eigen::Vector2d& image_point) const {
    Eigen::Vector2d world_point;
    CameraModelImageToWorld(model_id_, params_, image_point(0), image_point(1),
                            &world_point(0), &world_point(1));
    return world_point;
}

double Camera::ImageToWorldThreshold(const double threshold) const {
    return CameraModelImageToWorldThreshold(model_id_, params_, threshold);
}

Eigen::Vector2d Camera::WorldToImage(const Eigen::Vector2d& world_point) const {
    Eigen::Vector2d image_point;
    CameraModelWorldToImage(model_id_, params_, world_point(0), world_point(1),
                            &image_point(0), &image_point(1));
    return image_point;
}

void Camera::Rescale(const double scale) {
    const double scale_x =
            std::round(scale * width_) / static_cast<double>(width_);
    const double scale_y =
            std::round(scale * height_) / static_cast<double>(height_);
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


namespace {
    static const double kNaN = std::numeric_limits<double>::quiet_NaN();
}


VisibilityPyramid::VisibilityPyramid() : VisibilityPyramid(0, 0, 0) { }

VisibilityPyramid::VisibilityPyramid(const size_t num_levels,
                                     const size_t width, const size_t height)
        : width_(width), height_(height), score_(0), max_score_(0) {
    pyramid_.resize(num_levels);
    for (size_t level = 0; level < num_levels; ++level) {
        const size_t level_plus_one = level + 1;
        const int dim = 1 << level_plus_one;
        pyramid_[level].setZero(dim, dim);
        max_score_ += dim * dim * dim * dim;
    }
}

void VisibilityPyramid::SetPoint(const double x, const double y) {
    size_t cx = 0;
    size_t cy = 0;
    CellForPoint(x, y, &cx, &cy);

    for (int i = static_cast<int>(pyramid_.size() - 1); i >= 0; --i) {
        auto& level = pyramid_[i];

        level(cy, cx) += 1;
        if (level(cy, cx) == 1) {
            score_ += level.size();
        }

        cx = cx >> 1;
        cy = cy >> 1;
    }
}

void VisibilityPyramid::ResetPoint(const double x, const double y) {
    size_t cx = 0;
    size_t cy = 0;
    CellForPoint(x, y, &cx, &cy);

    for (int i = static_cast<int>(pyramid_.size() - 1); i >= 0; --i) {
        auto& level = pyramid_[i];

        level(cy, cx) -= 1;
        if (level(cy, cx) == 0) {
            score_ -= level.size();
        }

        cx = cx >> 1;
        cy = cy >> 1;
    }
}

void VisibilityPyramid::CellForPoint(const double x, const double y, size_t* cx,
                                     size_t* cy) const {
    const int max_dim = 1 << pyramid_.size();
    *cx = Clip<size_t>(static_cast<size_t>(max_dim * x / width_), 0,
                       static_cast<size_t>(max_dim - 1));
    *cy = Clip<size_t>(static_cast<size_t>(max_dim * y / height_), 0,
                       static_cast<size_t>(max_dim - 1));
}


size_t VisibilityPyramid::NumLevels() const { return pyramid_.size(); }

size_t VisibilityPyramid::Width() const { return width_; }

size_t VisibilityPyramid::Height() const { return height_; }

size_t VisibilityPyramid::Score() const { return score_; }

size_t VisibilityPyramid::MaxScore() const { return max_score_; }


Point2D::Point2D()
        : xy_(Eigen::Vector2d::Zero()), point3D_id_(kInvalidPoint3DId) { }

const Eigen::Vector2d& Point2D::XY() const { return xy_; }

Eigen::Vector2d& Point2D::XY() { return xy_; }

double Point2D::X() const { return xy_.x(); }

double Point2D::Y() const { return xy_.y(); }

void Point2D::SetXY(const Eigen::Vector2d& xy) { xy_ = xy; }

point3D_t Point2D::Point3DId() const { return point3D_id_; }

bool Point2D::HasPoint3D() const { return point3D_id_ != kInvalidPoint3DId; }

void Point2D::SetPoint3DId(const point3D_t point3D_id) {
    point3D_id_ = point3D_id;
}

Image::Image()
        : image_id_(kInvalidImageId),
          name_(""),
          camera_id_(kInvalidCameraId),
          registered_(false),
          num_points3D_(0),
          num_observations_(0),
          num_correspondences_(0),
          num_visible_points3D_(0),
          qvec_(1.0, 0.0, 0.0, 0.0),
          tvec_(0.0, 0.0, 0.0),
          qvec_prior_(kNaN, kNaN, kNaN, kNaN),
          tvec_prior_(kNaN, kNaN, kNaN) { }

void Image::SetUp(const class Camera& camera) {
    point3D_visibility_pyramid_ = VisibilityPyramid(
            kNumPoint3DVisibilityPyramidLevels, camera.Width(), camera.Height());
}

void Image::TearDown() {
    point3D_visibility_pyramid_ = VisibilityPyramid(0, 0, 0);
}

void Image::SetPoints2D(const std::vector<Eigen::Vector2d>& points) {
    points2D_.resize(points.size());
    num_correspondences_have_point3D_.resize(points.size(), 0);
    for (point2D_t point2D_idx = 0; point2D_idx < points.size(); ++point2D_idx) {
        points2D_[point2D_idx].SetXY(points[point2D_idx]);
    }
}

void Image::SetPoint3DForPoint2D(const point2D_t point2D_idx,
                                 const point3D_t point3D_id) {
    class Point2D& point2D = points2D_.at(point2D_idx);
    if (!point2D.HasPoint3D()) {
        num_points3D_ += 1;
    }
    point2D.SetPoint3DId(point3D_id);
}

void Image::ResetPoint3DForPoint2D(const point2D_t point2D_idx) {
    class Point2D& point2D = points2D_.at(point2D_idx);
    if (point2D.HasPoint3D()) {
        point2D.SetPoint3DId(kInvalidPoint3DId);
        num_points3D_ -= 1;
    }
}

bool Image::HasPoint3D(const point3D_t point3D_id) const {
    return std::find_if(points2D_.begin(), points2D_.end(),
                        [point3D_id](const class Point2D& point2D) {
                            return point2D.Point3DId() == point3D_id;
                        }) != points2D_.end();
}

void Image::IncrementCorrespondenceHasPoint3D(const point2D_t point2D_idx) {
    const class Point2D& point2D = points2D_.at(point2D_idx);

    num_correspondences_have_point3D_[point2D_idx] += 1;
    if (num_correspondences_have_point3D_[point2D_idx] == 1) {
        num_visible_points3D_ += 1;
    }

    point3D_visibility_pyramid_.SetPoint(point2D.X(), point2D.Y());

    assert(num_visible_points3D_ <= num_observations_);
}

void Image::DecrementCorrespondenceHasPoint3D(const point2D_t point2D_idx) {
    const class Point2D& point2D = points2D_.at(point2D_idx);

    num_correspondences_have_point3D_[point2D_idx] -= 1;
    if (num_correspondences_have_point3D_[point2D_idx] == 0) {
        num_visible_points3D_ -= 1;
    }

    point3D_visibility_pyramid_.ResetPoint(point2D.X(), point2D.Y());

    assert(num_visible_points3D_ <= num_observations_);
}

void Image::NormalizeQvec() { qvec_ = NormalizeQuaternion(qvec_); }

Eigen::Matrix3x4d Image::ProjectionMatrix() const {
    return ComposeProjectionMatrix(qvec_, tvec_);
}

Eigen::Matrix3x4d Image::InverseProjectionMatrix() const {
    return InvertProjectionMatrix(ComposeProjectionMatrix(qvec_, tvec_));
}

Eigen::Matrix3d Image::RotationMatrix() const {
    return QuaternionToRotationMatrix(qvec_);
}

Eigen::Vector3d Image::ProjectionCenter() const {
    return ProjectionCenterFromParameters(qvec_, tvec_);
}


image_t Image::ImageId() const { return image_id_; }

void Image::SetImageId(const image_t image_id) { image_id_ = image_id; }

const std::string& Image::Name() const { return name_; }

std::string& Image::Name() { return name_; }

void Image::SetName(const std::string& name) { name_ = name; }

camera_t Image::CameraId() const { return camera_id_; }

void Image::SetCameraId(const camera_t camera_id) {
    camera_id_ = camera_id;
}

bool Image::HasCamera() const { return camera_id_ != kInvalidCameraId; }

bool Image::IsRegistered() const { return registered_; }

void Image::SetRegistered(const bool registered) { registered_ = registered; }

point2D_t Image::NumPoints2D() const {
    return static_cast<point2D_t>(points2D_.size());
}

point2D_t Image::NumPoints3D() const { return num_points3D_; }

point2D_t Image::NumObservations() const { return num_observations_; }

void Image::SetNumObservations(const point2D_t num_observations) {
    num_observations_ = num_observations;
}

point2D_t Image::NumCorrespondences() const { return num_correspondences_; }

void Image::SetNumCorrespondences(const point2D_t num_correspondences) {
    num_correspondences_ = num_correspondences;
}

point2D_t Image::NumVisiblePoints3D() const { return num_visible_points3D_; }

size_t Image::Point3DVisibilityScore() const {
    return point3D_visibility_pyramid_.Score();
}

const Eigen::Vector4d& Image::Qvec() const { return qvec_; }

Eigen::Vector4d& Image::Qvec() { return qvec_; }

double Image::Qvec(const size_t idx) const { return qvec_(idx); }

double& Image::Qvec(const size_t idx) { return qvec_(idx); }

void Image::SetQvec(const Eigen::Vector4d& qvec) { qvec_ = qvec; }

const Eigen::Vector4d& Image::QvecPrior() const { return qvec_prior_; }

Eigen::Vector4d& Image::QvecPrior() { return qvec_prior_; }

double Image::QvecPrior(const size_t idx) const {
    return qvec_prior_(idx);
}

double& Image::QvecPrior(const size_t idx) { return qvec_prior_(idx); }

bool Image::HasQvecPrior() const { return !IsNaN(qvec_prior_.sum()); }

void Image::SetQvecPrior(const Eigen::Vector4d& qvec) { qvec_prior_ = qvec; }

const Eigen::Vector3d& Image::Tvec() const { return tvec_; }

Eigen::Vector3d& Image::Tvec() { return tvec_; }

double Image::Tvec(const size_t idx) const { return tvec_(idx); }

double& Image::Tvec(const size_t idx) { return tvec_(idx); }

void Image::SetTvec(const Eigen::Vector3d& tvec) { tvec_ = tvec; }

const Eigen::Vector3d& Image::TvecPrior() const { return tvec_prior_; }

Eigen::Vector3d& Image::TvecPrior() { return tvec_prior_; }

double Image::TvecPrior(const size_t idx) const {
    return tvec_prior_(idx);
}

double& Image::TvecPrior(const size_t idx) { return tvec_prior_(idx); }

bool Image::HasTvecPrior() const { return !IsNaN(tvec_prior_.sum()); }

void Image::SetTvecPrior(const Eigen::Vector3d& tvec) { tvec_prior_ = tvec; }

const class Point2D& Image::Point2D(const point2D_t point2D_idx) const {
    return points2D_.at(point2D_idx);
}

class Point2D& Image::Point2D(const point2D_t point2D_idx) {
    return points2D_.at(point2D_idx);
}

const std::vector<class Point2D>& Image::Points2D() const { return points2D_; }

bool Image::IsPoint3DVisible(const point2D_t point2D_idx) const {
    return num_correspondences_have_point3D_.at(point2D_idx) > 0;
}
