#include "visibility_pyramid.h"

VisibilityPyramid::VisibilityPyramid() : VisibilityPyramid(0, 0, 0) { }

VisibilityPyramid::VisibilityPyramid(
        const size_t num_levels,
        const size_t width,
        const size_t height
) : width_(width), height_(height), score_(0), max_score_(0) {

    pyramid_.resize(num_levels);
    for (size_t level = 0; level < num_levels; ++level) {
        const size_t level_plus_one = level + 1;
        const int dim = 1 << level_plus_one;
        pyramid_[level].setZero(dim, dim);
        max_score_ += dim * dim * dim * dim;
    }
}

void VisibilityPyramid::SetPoint(const double x, const double y) {
    size_t cx = 0;
    size_t cy = 0;
    CellForPoint(x, y, &cx, &cy);

    for (int i = static_cast<int>(pyramid_.size() - 1); i >= 0; --i) {
        auto& level = pyramid_[i];

        level(cy, cx) += 1;
        if (level(cy, cx) == 1) {
            score_ += level.size();
        }

        cx = cx >> 1;
        cy = cy >> 1;
    }
}

void VisibilityPyramid::ResetPoint(const double x, const double y) {
    size_t cx = 0;
    size_t cy = 0;
    CellForPoint(x, y, &cx, &cy);

    for (int i = static_cast<int>(pyramid_.size() - 1); i >= 0; --i) {
        auto& level = pyramid_[i];

        level(cy, cx) -= 1;
        if (level(cy, cx) == 0) {
            score_ -= level.size();
        }

        cx = cx >> 1;
        cy = cy >> 1;
    }
}

void VisibilityPyramid::CellForPoint(const double x, const double y, size_t* cx, size_t* cy) const {
    const int max_dim = 1 << pyramid_.size();
    *cx = Clip<size_t>(static_cast<size_t>(max_dim * x / width_), 0,
                       static_cast<size_t>(max_dim - 1));
    *cy = Clip<size_t>(static_cast<size_t>(max_dim * y / height_), 0,
                       static_cast<size_t>(max_dim - 1));
}


size_t VisibilityPyramid::NumLevels() const { return pyramid_.size(); }

size_t VisibilityPyramid::Width() const { return width_; }

size_t VisibilityPyramid::Height() const { return height_; }

size_t VisibilityPyramid::Score() const { return score_; }

size_t VisibilityPyramid::MaxScore() const { return max_score_; }
