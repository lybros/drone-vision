#ifndef INC_3D_RECONSTRUCTION_POINT_3D_H
#define INC_3D_RECONSTRUCTION_POINT_3D_H

#include "../utils.h"
#include "track.h"
#include <Eigen/Core>


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

#endif //INC_3D_RECONSTRUCTION_POINT_3D_H
