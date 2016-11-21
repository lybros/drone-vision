#ifndef INC_3D_RECONSTRUCTION_TRACK_H
#define INC_3D_RECONSTRUCTION_TRACK_H

#include "../utils.h"

struct TrackElement {
    TrackElement();

    TrackElement(const image_t image_id, const point2D_t point2D_idx);

    image_t image_id;
    point2D_t point2D_idx;
};

// Due to definition from diploma: Track is a set of points of 2d points which refer to the same 3d point.
class Track {
public:
    Track();

    size_t Length() const;

    const std::vector<TrackElement>& Elements() const;

    void SetElements(const std::vector<TrackElement>& elements);

    const TrackElement& Element(const size_t idx) const;

    TrackElement& Element(const size_t idx);

    void SetElement(const size_t idx, const TrackElement& element);

    void AddElement(const TrackElement& element);

    void AddElement(const image_t image_id, const point2D_t point2D_idx);

    void AddElements(const std::vector<TrackElement>& elements);

    void DeleteElement(const size_t idx);

    void DeleteElement(const image_t image_id, const point2D_t point2D_idx);

    void Reserve(const size_t num_elements);

    void Compress();

private:
    std::vector<TrackElement> elements_;
};

#endif //INC_3D_RECONSTRUCTION_TRACK_H
