#include "point_2d.h"

Point2D::Point2D() : xy_(Eigen::Vector2d::Zero()), point3D_id_(kInvalidPoint3DId) { }

const Eigen::Vector2d& Point2D::XY() const { return xy_; }

Eigen::Vector2d& Point2D::XY() { return xy_; }

double Point2D::X() const { return xy_.x(); }

double Point2D::Y() const { return xy_.y(); }

void Point2D::SetXY(const Eigen::Vector2d& xy) { xy_ = xy; }

point3D_t Point2D::Point3DId() const { return point3D_id_; }

bool Point2D::HasPoint3D() const { return point3D_id_ != kInvalidPoint3DId; }

void Point2D::SetPoint3DId(const point3D_t point3D_id) { point3D_id_ = point3D_id; }