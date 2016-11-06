#ifndef INC_3D_RECONSTRUCTION_ENTITIES_H
#define INC_3D_RECONSTRUCTION_ENTITIES_H

#include "utils.h"

#include <Eigen/Core>

#include <boost/algorithm/string.hpp>

static const int kInvalidCameraModelId = -1;

template<typename CameraModel>
struct BaseCameraModel {
    template<typename T>
    static bool HasBogusParams(const std::vector<T>& params,
                               const size_t width, const size_t height,
                               const T min_focal_length_ratio,
                               const T max_focal_length_ratio,
                               const T max_extra_param);

    template<typename T>
    static bool HasBogusFocalLength(const std::vector<T>& params,
                                    const size_t width,
                                    const size_t height,
                                    const T min_focal_length_ratio,
                                    const T max_focal_length_ratio);

    template<typename T>
    static bool HasBogusPrincipalPoint(const std::vector<T>& params,
                                       const size_t width,
                                       const size_t height);

    template<typename T>
    static bool HasBogusExtraParams(const std::vector<T>& params,
                                    const T max_extra_param);

    template<typename T>
    static T ImageToWorldThreshold(const T* params, const T threshold);

    template<typename T>
    static void IterativeUndistortion(const T* params, T* u, T* v);
};

struct RadialCameraModel : public BaseCameraModel<RadialCameraModel> {
    static const int model_id = 1;
    static const int num_params = 5;
    static const std::string params_info;
    static const std::vector<size_t> focal_length_idxs;
    static const std::vector<size_t> principal_point_idxs;
    static const std::vector<size_t> extra_params_idxs;

    static std::string InitializeParamsInfo();

    static std::vector<size_t> InitializeFocalLengthIdxs();

    static std::vector<size_t> InitializePrincipalPointIdxs();

    static std::vector<size_t> InitializeExtraParamsIdxs();

    template<typename T>
    static void WorldToImage(const T* params, const T u, const T v, T* x, T* y);

    template<typename T>
    static void ImageToWorld(const T* params, const T x, const T y, T* u, T* v);

    template<typename T>
    static void Distortion(const T* extra_params, const T u, const T v, T* du,
                           T* dv);
};

struct PinholeCameraModel : public BaseCameraModel<PinholeCameraModel> {
    static const int model_id = 2;
    static const int num_params = 4;
    static const std::string params_info;
    static const std::vector<size_t> focal_length_idxs;
    static const std::vector<size_t> principal_point_idxs;
    static const std::vector<size_t> extra_params_idxs;

    static std::string InitializeParamsInfo();

    static std::vector<size_t> InitializeFocalLengthIdxs();

    static std::vector<size_t> InitializePrincipalPointIdxs();

    static std::vector<size_t> InitializeExtraParamsIdxs();

    template<typename T>
    static void WorldToImage(const T* params, const T u, const T v, T* x, T* y);

    template<typename T>
    static void ImageToWorld(const T* params, const T x, const T y, T* u, T* v);

    template<typename T>
    static void Distortion(const T* extra_params, const T u, const T v, T* du,
                           T* dv);
};

int CameraModelNameToId(const std::string& name);

std::string CameraModelIdToName(const int model_id);

void CameraModelInitializeParams(const int model_id, const double focal_length,
                                 const size_t width, const size_t height,
                                 std::vector<double>* params);

std::string CameraModelParamsInfo(const int model_id);

std::vector<size_t> CameraModelFocalLengthIdxs(const int model_id);

std::vector<size_t> CameraModelPrincipalPointIdxs(const int model_id);

std::vector<size_t> CameraModelExtraParamsIdxs(const int model_id);

bool CameraModelVerifyParams(const int model_id,
                             const std::vector<double>& params);

bool CameraModelHasBogusParams(const int model_id,
                               const std::vector<double>& params,
                               const size_t width, const size_t height,
                               const double min_focal_length_ratio,
                               const double max_focal_length_ratio,
                               const double max_extra_param);

void CameraModelWorldToImage(const int model_id,
                             const std::vector<double>& params,
                             const double u, const double v, double* x,
                             double* y);

void CameraModelImageToWorld(const int model_id,
                             const std::vector<double>& params,
                             const double x, const double y, double* u,
                             double* v);

double CameraModelImageToWorldThreshold(
        const int model_id, const std::vector<double>& params,
        const double threshold);

template<typename CameraModel>
template<typename T>
bool BaseCameraModel<CameraModel>::HasBogusParams(
        const std::vector<T>& params, const size_t width, const size_t height,
        const T min_focal_length_ratio, const T max_focal_length_ratio,
        const T max_extra_param) {
    if (HasBogusPrincipalPoint(params, width, height)) {
        return true;
    }

    if (HasBogusFocalLength(params, width, height, min_focal_length_ratio,
                            max_focal_length_ratio)) {
        return true;
    }

    if (HasBogusExtraParams(params, max_extra_param)) {
        return true;
    }

    return false;
}

template<typename CameraModel>
template<typename T>
bool BaseCameraModel<CameraModel>::HasBogusFocalLength(
        const std::vector<T>& params, const size_t width, const size_t height,
        const T min_focal_length_ratio, const T max_focal_length_ratio) {
    const size_t max_size = std::max(width, height);

    for (const auto& idx : CameraModel::focal_length_idxs) {
        const T focal_length_ratio = params[idx] / max_size;
        if (focal_length_ratio < min_focal_length_ratio ||
            focal_length_ratio > max_focal_length_ratio) {
            return true;
        }
    }

    return false;
}

template<typename CameraModel>
template<typename T>
bool BaseCameraModel<CameraModel>::HasBogusPrincipalPoint(
        const std::vector<T>& params, const size_t width, const size_t height) {
    const T cx = params[CameraModel::principal_point_idxs[0]];
    const T cy = params[CameraModel::principal_point_idxs[1]];
    return cx < 0 || cx > width || cy < 0 || cy > height;
}

template<typename CameraModel>
template<typename T>
bool BaseCameraModel<CameraModel>::HasBogusExtraParams(
        const std::vector<T>& params, const T max_extra_param) {
    for (const auto& idx : CameraModel::extra_params_idxs) {
        if (std::abs(params[idx]) > max_extra_param) {
            return true;
        }
    }

    return false;
}

template<typename CameraModel>
template<typename T>
T BaseCameraModel<CameraModel>::ImageToWorldThreshold(const T* params,
                                                      const T threshold) {
    T mean_focal_length = 0;
    for (const auto& idx : CameraModel::focal_length_idxs) {
        mean_focal_length += params[idx];
    }
    mean_focal_length /= CameraModel::focal_length_idxs.size();
    return threshold / mean_focal_length;
}

template<typename CameraModel>
template<typename T>
void BaseCameraModel<CameraModel>::IterativeUndistortion(const T* params, T* u,
                                                         T* v) {
    const size_t kNumUndistortionIterations = 100;
    const double kUndistortionEpsilon = 1e-10;

    T uu = *u;
    T vv = *v;
    T du;
    T dv;

    for (size_t i = 0; i < kNumUndistortionIterations; ++i) {
        CameraModel::Distortion(params, uu, vv, &du, &dv);
        const T uu_prev = uu;
        const T vv_prev = vv;
        uu = *u - du;
        vv = *v - dv;
        if (std::abs(uu_prev - uu) < kUndistortionEpsilon &&
            std::abs(vv_prev - vv) < kUndistortionEpsilon) {
            break;
        }
    }

    *u = uu;
    *v = vv;
}

template<typename T>
void RadialCameraModel::WorldToImage(const T* params, const T u, const T v,
                                     T* x, T* y) {
    const T f = params[0];
    const T c1 = params[1];
    const T c2 = params[2];

    T du, dv;
    Distortion(&params[3], u, v, &du, &dv);
    *x = u + du;
    *y = v + dv;

    *x = f * *x + c1;
    *y = f * *y + c2;
}

template<typename T>
void RadialCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                     T* u, T* v) {
    const T f = params[0];
    const T c1 = params[1];
    const T c2 = params[2];

    *u = (x - c1) / f;
    *v = (y - c2) / f;

    IterativeUndistortion(&params[3], u, v);
}

template<typename T>
void RadialCameraModel::Distortion(const T* extra_params, const T u, const T v,
                                   T* du, T* dv) {
    const T k1 = extra_params[0];
    const T k2 = extra_params[1];

    const T u2 = u * u;
    const T v2 = v * v;
    const T r2 = u2 + v2;
    const T radial = k1 * r2 + k2 * r2 * r2;
    *du = u * radial;
    *dv = v * radial;
}


template<typename T>
void PinholeCameraModel::WorldToImage(const T* params, const T u, const T v,
                                      T* x, T* y) {
    const T f1 = params[0];
    const T f2 = params[1];
    const T c1 = params[2];
    const T c2 = params[3];

    *x = f1 * u + c1;
    *y = f2 * v + c2;
}

template<typename T>
void PinholeCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                      T* u, T* v) {
    const T f1 = params[0];
    const T f2 = params[1];
    const T c1 = params[2];
    const T c2 = params[3];

    *u = (x - c1) / f1;
    *v = (y - c2) / f2;
}


struct TrackElement {
    TrackElement();

    TrackElement(const image_t image_id, const point2D_t point2D_idx);

    image_t image_id;
    point2D_t point2D_idx;
};

class Track {
public:
    Track();

    size_t Length() const;

    const std::vector<TrackElement>& Elements() const;

    void SetElements(const std::vector<TrackElement>& elements);

    const TrackElement& Element(const size_t idx) const;

    TrackElement& Element(const size_t idx);

    void SetElement(const size_t idx, const TrackElement& element);

    void AddElement(const TrackElement& element);

    void AddElement(const image_t image_id, const point2D_t point2D_idx);

    void AddElements(const std::vector<TrackElement>& elements);

    void DeleteElement(const size_t idx);

    void DeleteElement(const image_t image_id, const point2D_t point2D_idx);

    void Reserve(const size_t num_elements);

    void Compress();

private:
    std::vector<TrackElement> elements_;
};


class Point_3D {
public:
    Point_3D();

    const Eigen::Vector3d& XYZ() const;

    Eigen::Vector3d& XYZ();

    double XYZ(const size_t idx) const;

    double& XYZ(const size_t idx);

    double X() const;

    double Y() const;

    double Z() const;

    void SetXYZ(const Eigen::Vector3d& xyz);

    const Eigen::Vector3ub& Color() const;

    Eigen::Vector3ub& Color();

    uint8_t Color(const size_t idx) const;

    uint8_t& Color(const size_t idx);

    void SetColor(const Eigen::Vector3ub& color);

    double Error() const;

    bool HasError() const;

    void SetError(const double error);

    const class Track& Track() const;

    class Track& Track();

    void SetTrack(const class Track& track);

private:
    Eigen::Vector3d xyz_;

    Eigen::Vector3ub color_;

    double error_;

    class Track track_;
};


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

    bool HasBogusParams(const double min_focal_length_ratio,
                        const double max_focal_length_ratio,
                        const double max_extra_param) const;

    void InitializeWithId(const int model_id, const double focal_length,
                          const size_t width, const size_t height);

    void InitializeWithName(const std::string& model_name,
                            const double focal_length, const size_t width,
                            const size_t height);

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

    float scale = 0.0f;
    float orientation = 0.0f;
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


class VisibilityPyramid {
public:
    VisibilityPyramid();

    VisibilityPyramid(const size_t num_levels, const size_t width,
                      const size_t height);

    void SetPoint(const double x, const double y);

    void ResetPoint(const double x, const double y);

    size_t NumLevels() const;

    size_t Width() const;

    size_t Height() const;

    size_t Score() const;

    size_t MaxScore() const;

private:
    void CellForPoint(const double x, const double y, size_t* cx,
                      size_t* cy) const;

    size_t width_;
    size_t height_;

    size_t score_;

    size_t max_score_;

    std::vector<Eigen::MatrixXi> pyramid_;
};


class Point2D {
public:
    Point2D();

    const Eigen::Vector2d& XY() const;

    Eigen::Vector2d& XY();

    double X() const;

    double Y() const;

    void SetXY(const Eigen::Vector2d& xy);

    point3D_t Point3DId() const;

    bool HasPoint3D() const;

    void SetPoint3DId(const point3D_t point3D_id);

private:
    Eigen::Vector2d xy_;

    point3D_t point3D_id_;
};


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

    void SetPoints2D(const std::vector<Eigen::Vector2d>& points);

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

    std::vector<image_t> num_correspondences_have_point3D_;

    VisibilityPyramid point3D_visibility_pyramid_;
};


#endif //INC_3D_RECONSTRUCTION_ENTITIES_H
