#ifndef INC_3D_RECONSTRUCTION_VISIBILITY_PYRAMID_H
#define INC_3D_RECONSTRUCTION_VISIBILITY_PYRAMID_H

#include "../utils.h"

class VisibilityPyramid {
public:
    VisibilityPyramid();

    VisibilityPyramid(const size_t num_levels, const size_t width, const size_t height);

    void SetPoint(const double x, const double y);

    void ResetPoint(const double x, const double y);

    size_t NumLevels() const;

    size_t Width() const;

    size_t Height() const;

    size_t Score() const;

    size_t MaxScore() const;

private:
    void CellForPoint(const double x, const double y, size_t* cx, size_t* cy) const;

    size_t width_;
    size_t height_;

    size_t score_;

    size_t max_score_;

    std::vector <Eigen::MatrixXi> pyramid_;
};

#endif //INC_3D_RECONSTRUCTION_VISIBILITY_PYRAMID_H
