#ifndef INC_3D_RECONSTRUCTION_POINT_2D_H
#define INC_3D_RECONSTRUCTION_POINT_2D_H

#include "../utils.h"

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

#endif //INC_3D_RECONSTRUCTION_POINT_2D_H
