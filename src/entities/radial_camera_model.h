#ifndef INC_3D_RECONSTRUCTION_RADIAL_CAMERA_MODEL_H
#define INC_3D_RECONSTRUCTION_RADIAL_CAMERA_MODEL_H

#include "base_camera_model.h"

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
    static void Distortion(const T* extra_params, const T u, const T v, T* du, T* dv);
};

template<typename T>
void RadialCameraModel::WorldToImage(const T* params, const T u, const T v, T* x, T* y) {
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
void RadialCameraModel::ImageToWorld(const T* params, const T x, const T y, T* u, T* v) {
    const T f = params[0];
    const T c1 = params[1];
    const T c2 = params[2];

    *u = (x - c1) / f;
    *v = (y - c2) / f;

    IterativeUndistortion(&params[3], u, v);
}

template<typename T>
void RadialCameraModel::Distortion(const T* extra_params, const T u, const T v, T* du, T* dv) {
    const T k1 = extra_params[0];
    const T k2 = extra_params[1];

    const T u2 = u * u;
    const T v2 = v * v;
    const T r2 = u2 + v2;
    const T radial = k1 * r2 + k2 * r2 * r2;
    *du = u * radial;
    *dv = v * radial;
}

#endif //INC_3D_RECONSTRUCTION_RADIAL_CAMERA_MODEL_H
