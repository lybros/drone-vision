#include "point_3d.h"

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

class Track& Point_3D::Track() { return track_; }

void Point_3D::SetTrack(const class Track& track) { track_ = track; }