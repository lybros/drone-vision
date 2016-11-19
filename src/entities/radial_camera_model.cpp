#include "radial_camera_model.h"

const std::string RadialCameraModel::params_info = RadialCameraModel::InitializeParamsInfo();
const std::vector<size_t> RadialCameraModel::focal_length_idxs = RadialCameraModel::InitializeFocalLengthIdxs();
const std::vector<size_t> RadialCameraModel::principal_point_idxs = RadialCameraModel::InitializePrincipalPointIdxs();
const std::vector<size_t> RadialCameraModel::extra_params_idxs = RadialCameraModel::InitializeExtraParamsIdxs();

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