#ifndef INC_3D_RECONSTRUCTION_PINHOLE_CAMERA_MODEL_H
#define INC_3D_RECONSTRUCTION_PINHOLE_CAMERA_MODEL_H

#include "base_camera_model.h"

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
    static void Distortion(const T* extra_params, const T u, const T v, T* du, T* dv);
};

template<typename T>
void PinholeCameraModel::WorldToImage(const T* params, const T u, const T v, T* x, T* y) {
    const T f1 = params[0];
    const T f2 = params[1];
    const T c1 = params[2];
    const T c2 = params[3];

    *x = f1 * u + c1;
    *y = f2 * v + c2;
}

template<typename T>
void PinholeCameraModel::ImageToWorld(const T* params, const T x, const T y, T* u, T* v) {
    const T f1 = params[0];
    const T f2 = params[1];
    const T c1 = params[2];
    const T c2 = params[3];

    *u = (x - c1) / f1;
    *v = (y - c2) / f2;
}

#endif //INC_3D_RECONSTRUCTION_PINHOLE_CAMERA_MODEL_H
