#include "track.h"

Track::Track() { }

TrackElement::TrackElement() : image_id(kInvalidImageId), point2D_idx(kInvalidPoint2DIdx) { }

TrackElement::TrackElement(
        const image_t image_id,
        const point2D_t point2D_idx
) : image_id(image_id), point2D_idx(point2D_idx) { }

void Track::DeleteElement(const image_t image_id, const point2D_t point2D_idx) {
    elements_.erase(std::remove_if(
            elements_.begin(),
            elements_.end(),
            [image_id, point2D_idx](const TrackElement& element) {
                return element.image_id == image_id && element.point2D_idx == point2D_idx;
            }), elements_.end());
}

size_t Track::Length() const { return elements_.size(); }

const std::vector<TrackElement>& Track::Elements() const { return elements_; }

void Track::SetElements(const std::vector<TrackElement>& elements) { elements_ = elements; }

const TrackElement& Track::Element(const size_t idx) const {
    return elements_.at(idx);
}

TrackElement& Track::Element(const size_t idx) { return elements_.at(idx); }

void Track::SetElement(const size_t idx, const TrackElement& element) {
    elements_.at(idx) = element;
}

void Track::AddElement(const TrackElement& element) {
    elements_.push_back(element);
}

void Track::AddElement(const image_t image_id, const point2D_t point2D_idx) {
    elements_.emplace_back(image_id, point2D_idx);
}

void Track::AddElements(const std::vector<TrackElement>& elements) {
    elements_.insert(elements_.end(), elements.begin(), elements.end());
}

void Track::DeleteElement(const size_t idx) {
    elements_.erase(elements_.begin() + idx);
}

void Track::Reserve(const size_t num_elements) {
    elements_.reserve(num_elements);
}

void Track::Compress() { elements_.shrink_to_fit(); }