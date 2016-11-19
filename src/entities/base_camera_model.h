#ifndef INC_3D_RECONSTRUCTION_BASE_CAMERA_MODEL_H
#define INC_3D_RECONSTRUCTION_BASE_CAMERA_MODEL_H

#include "../utils.h"

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

template<typename CameraModel>
template<typename T>
bool BaseCameraModel<CameraModel>::HasBogusParams(
        const std::vector<T>& params,
        const size_t width,
        const size_t height,
        const T min_focal_length_ratio,
        const T max_focal_length_ratio,
        const T max_extra_param
) {
    if (HasBogusPrincipalPoint(params, width, height)) {
        return true;
    }

    if (HasBogusFocalLength(params, width, height, min_focal_length_ratio, max_focal_length_ratio)) {
        return true;
    }

    return HasBogusExtraParams(params, max_extra_param);
}

template<typename CameraModel>
template<typename T>
bool BaseCameraModel<CameraModel>::HasBogusFocalLength(
        const std::vector<T>& params,
        const size_t width,
        const size_t height,
        const T min_focal_length_ratio,
        const T max_focal_length_ratio
) {
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
        const std::vector<T>& params,
        const size_t width,
        const size_t height
) {
    const T cx = params[CameraModel::principal_point_idxs[0]];
    const T cy = params[CameraModel::principal_point_idxs[1]];
    return cx < 0 || cx > width || cy < 0 || cy > height;
}

template<typename CameraModel>
template<typename T>
bool BaseCameraModel<CameraModel>::HasBogusExtraParams(const std::vector<T>& params, const T max_extra_param) {
    for (const auto& idx : CameraModel::extra_params_idxs) {
        if (std::abs(params[idx]) > max_extra_param) {
            return true;
        }
    }

    return false;
}

template<typename CameraModel>
template<typename T>
T BaseCameraModel<CameraModel>::ImageToWorldThreshold(const T* params, const T threshold) {
    T mean_focal_length = 0;
    for (const auto& idx : CameraModel::focal_length_idxs) {
        mean_focal_length += params[idx];
    }
    mean_focal_length /= CameraModel::focal_length_idxs.size();
    return threshold / mean_focal_length;
}

template<typename CameraModel>
template<typename T>
void BaseCameraModel<CameraModel>::IterativeUndistortion(const T* params, T* u, T* v) {
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
#endif //INC_3D_RECONSTRUCTION_BASE_CAMERA_MODEL_H
