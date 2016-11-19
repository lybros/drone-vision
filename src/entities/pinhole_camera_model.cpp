#include "pinhole_camera_model.h"

const std::string PinholeCameraModel::params_info = PinholeCameraModel::InitializeParamsInfo();
const std::vector<size_t> PinholeCameraModel::focal_length_idxs = PinholeCameraModel::InitializeFocalLengthIdxs();
const std::vector<size_t> PinholeCameraModel::principal_point_idxs = PinholeCameraModel::InitializePrincipalPointIdxs();
const std::vector<size_t> PinholeCameraModel::extra_params_idxs = PinholeCameraModel::InitializeExtraParamsIdxs();

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