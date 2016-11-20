#include "image.h"

namespace { static const double kNaN = std::numeric_limits<double>::quiet_NaN(); }

Image::Image() : image_id_(kInvalidImageId),
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
            kNumPoint3DVisibilityPyramidLevels,
            camera.Width(),
            camera.Height()
    );
}

void Image::TearDown() { point3D_visibility_pyramid_ = VisibilityPyramid(0, 0, 0); }

void Image::SetPoints2D(const std::vector<Eigen::Vector2d>& points) {
    points2D_.resize(points.size());
    num_correspondences_have_point3D_.resize(points.size(), 0);

    for (point2D_t point2D_idx = 0; point2D_idx < points.size(); ++point2D_idx) {
        points2D_[point2D_idx].SetXY(points[point2D_idx]);
    }
}

const FeatureDescriptors& Image::Descriptors() const {
    return Image::descriptors_;
}

void Image::SetDescriptors(const FeatureDescriptors& descriptors) {
    Image::descriptors_ = descriptors;
}

void Image::SetPoint3DForPoint2D(const point2D_t point2D_idx, const point3D_t point3D_id) {
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
    return std::find_if(
            points2D_.begin(),
            points2D_.end(),
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

Eigen::Matrix3x4d Image::ProjectionMatrix() const { return ComposeProjectionMatrix(qvec_, tvec_); }

Eigen::Matrix3x4d Image::InverseProjectionMatrix() const {
    return InvertProjectionMatrix(ComposeProjectionMatrix(qvec_, tvec_));
}

Eigen::Matrix3d Image::RotationMatrix() const { return QuaternionToRotationMatrix(qvec_); }

Eigen::Vector3d Image::ProjectionCenter() const { return ProjectionCenterFromParameters(qvec_, tvec_); }

image_t Image::ImageId() const { return image_id_; }

void Image::SetImageId(const image_t image_id) { image_id_ = image_id; }

const std::string& Image::Name() const { return name_; }

std::string& Image::Name() { return name_; }

void Image::SetName(const std::string& name) { name_ = name; }

camera_t Image::CameraId() const { return camera_id_; }

void Image::SetCameraId(const camera_t camera_id) { camera_id_ = camera_id; }

bool Image::HasCamera() const { return camera_id_ != kInvalidCameraId; }

bool Image::IsRegistered() const { return registered_; }

void Image::SetRegistered(const bool registered) { registered_ = registered; }

point2D_t Image::NumPoints2D() const { return static_cast<point2D_t>(points2D_.size()); }

point2D_t Image::NumPoints3D() const { return num_points3D_; }

point2D_t Image::NumObservations() const { return num_observations_; }

void Image::SetNumObservations(const point2D_t num_observations) { num_observations_ = num_observations; }

point2D_t Image::NumCorrespondences() const { return num_correspondences_; }

void Image::SetNumCorrespondences(const point2D_t num_correspondences) {
    num_correspondences_ = num_correspondences;
}

point2D_t Image::NumVisiblePoints3D() const { return num_visible_points3D_; }

size_t Image::Point3DVisibilityScore() const { return point3D_visibility_pyramid_.Score(); }

const Eigen::Vector4d& Image::Qvec() const { return qvec_; }

Eigen::Vector4d& Image::Qvec() { return qvec_; }

double Image::Qvec(const size_t idx) const { return qvec_(idx); }

double& Image::Qvec(const size_t idx) { return qvec_(idx); }

void Image::SetQvec(const Eigen::Vector4d& qvec) { qvec_ = qvec; }

const Eigen::Vector4d& Image::QvecPrior() const { return qvec_prior_; }

Eigen::Vector4d& Image::QvecPrior() { return qvec_prior_; }

double Image::QvecPrior(const size_t idx) const { return qvec_prior_(idx); }

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

double Image::TvecPrior(const size_t idx) const { return tvec_prior_(idx); }

double& Image::TvecPrior(const size_t idx) { return tvec_prior_(idx); }

bool Image::HasTvecPrior() const { return !IsNaN(tvec_prior_.sum()); }

void Image::SetTvecPrior(const Eigen::Vector3d& tvec) { tvec_prior_ = tvec; }

const class Point2D& Image::Point2D(const point2D_t point2D_idx) const { return points2D_.at(point2D_idx); }

class Point2D& Image::Point2D(const point2D_t point2D_idx) { return points2D_.at(point2D_idx); }

const std::vector<class Point2D>& Image::Points2D() const { return points2D_; }

bool Image::IsPoint3DVisible(const point2D_t point2D_idx) const {
    return num_correspondences_have_point3D_.at(point2D_idx) > 0;
}