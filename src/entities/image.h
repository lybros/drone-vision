#ifndef INC_3D_RECONSTRUCTION_IMAGE_H
#define INC_3D_RECONSTRUCTION_IMAGE_H

#include "../utils.h"
#include "camera.h"
#include "point_2d.h"
#include "visibility_pyramid.h"

class Image {
public:
    static const int kNumPoint3DVisibilityPyramidLevels = 6;

    Image();

    void SetUp(const Camera& camera);

    void TearDown();

    image_t ImageId() const;

    void SetImageId(const image_t image_id);

    const std::string& Name() const;

    std::string& Name();

    void SetName(const std::string& name);

    camera_t CameraId() const;

    void SetCameraId(const camera_t camera_id);

    bool HasCamera() const;

    bool IsRegistered() const;

    void SetRegistered(const bool registered);

    point2D_t NumPoints2D() const;

    point2D_t NumPoints3D() const;

    point2D_t NumObservations() const;

    void SetNumObservations(const point2D_t num_observations);

    point2D_t NumCorrespondences() const;

    void SetNumCorrespondences(const point2D_t num_observations);

    point2D_t NumVisiblePoints3D() const;

    size_t Point3DVisibilityScore() const;

    const Eigen::Vector4d& Qvec() const;

    Eigen::Vector4d& Qvec();

    double Qvec(const size_t idx) const;

    double& Qvec(const size_t idx);

    void SetQvec(const Eigen::Vector4d& qvec);

    const Eigen::Vector4d& QvecPrior() const;

    Eigen::Vector4d& QvecPrior();

    double QvecPrior(const size_t idx) const;

    double& QvecPrior(const size_t idx);

    bool HasQvecPrior() const;

    void SetQvecPrior(const Eigen::Vector4d& qvec);

    const Eigen::Vector3d& Tvec() const;

    Eigen::Vector3d& Tvec();

    double Tvec(const size_t idx) const;

    double& Tvec(const size_t idx);

    void SetTvec(const Eigen::Vector3d& tvec);

    const Eigen::Vector3d& TvecPrior() const;

    Eigen::Vector3d& TvecPrior();

    double TvecPrior(const size_t idx) const;

    double& TvecPrior(const size_t idx);

    bool HasTvecPrior() const;

    void SetTvecPrior(const Eigen::Vector3d& tvec);

    const class Point2D& Point2D(const point2D_t point2D_idx) const;

    class Point2D& Point2D(const point2D_t point2D_idx);

    const std::vector<class Point2D>& Points2D() const;

    void SetPoints2D(const std::vector <Eigen::Vector2d>& points);

    void SetPoint3DForPoint2D(const point2D_t point2D_idx,
                              const point3D_t point3D_id);

    void ResetPoint3DForPoint2D(const point2D_t point2D_idx);

    bool IsPoint3DVisible(const point2D_t point2D_idx) const;

    bool HasPoint3D(const point3D_t point3D_id) const;

    void IncrementCorrespondenceHasPoint3D(const point2D_t point2D_idx);

    void DecrementCorrespondenceHasPoint3D(const point2D_t point2D_idx);

    void NormalizeQvec();

    Eigen::Matrix3x4d ProjectionMatrix() const;

    Eigen::Matrix3x4d InverseProjectionMatrix() const;

    Eigen::Matrix3d RotationMatrix() const;

    Eigen::Vector3d ProjectionCenter() const;

private:
    image_t image_id_;

    std::string name_;

    camera_t camera_id_;

    bool registered_;

    point2D_t num_points3D_;

    point2D_t num_observations_;

    point2D_t num_correspondences_;

    point2D_t num_visible_points3D_;

    Eigen::Vector4d qvec_;
    Eigen::Vector3d tvec_;

    Eigen::Vector4d qvec_prior_;
    Eigen::Vector3d tvec_prior_;

    std::vector<class Point2D> points2D_;

    std::vector <image_t> num_correspondences_have_point3D_;

    VisibilityPyramid point3D_visibility_pyramid_;
};

#endif //INC_3D_RECONSTRUCTION_IMAGE_H
