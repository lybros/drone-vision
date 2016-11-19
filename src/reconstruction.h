#ifndef INC_3D_RECONSTRUCTION_MODEL_H
#define INC_3D_RECONSTRUCTION_MODEL_H

#include "utils.h"
#include "storage.h"

#include <unordered_map>
#include <unordered_set>
#include <fstream>

class Reconstruction {
public:
    Reconstruction();

    size_t NumCameras() const;

    size_t NumImages() const;

    size_t NumRegImages() const;

    size_t NumPoints3D() const;

    size_t NumImagePairs() const;

    const class Camera& Camera(const camera_t camera_id) const;

    const class Image& Image(const image_t image_id) const;

    const class Point_3D& Point3D(const point3D_t point3D_id) const;

    const std::pair<size_t, size_t>& ImagePair(
            const image_pair_t pair_id) const;

    std::pair<size_t, size_t>& ImagePair(const image_t image_id1,
                                         const image_t image_id2);

    class Camera& Camera(const camera_t camera_id);

    class Image& Image(const image_t image_id);

    class Point_3D& Point3D(const point3D_t point3D_id);

    std::pair<size_t, size_t>& ImagePair(const image_pair_t pair_id);

    const std::pair<size_t, size_t>& ImagePair(
            const image_t image_id1, const image_t image_id2) const;

    const std::unordered_map<camera_t, class Camera>& Cameras() const;

    const std::unordered_map<image_t, class Image>& Images() const;

    const std::vector<image_t>& RegImageIds() const;

    const std::unordered_map<point3D_t, class Point_3D>& Points3D() const;

    const std::unordered_map<image_pair_t, std::pair<size_t, size_t>>&
            ImagePairs() const;

    std::unordered_set<point3D_t> Point3DIds() const;

    bool ExistsCamera(const camera_t camera_id) const;

    bool ExistsImage(const image_t image_id) const;

    bool ExistsPoint3D(const point3D_t point3D_id) const;

    bool ExistsImagePair(const image_pair_t pair_id) const;

    void Load(const DatabaseCache& database_cache);

    void SetUp(const SceneGraph* scene_graph);

    void TearDown();

    void AddCamera(const class Camera& camera);

    void AddImage(const class Image& image);

    point3D_t AddPoint3D(const Eigen::Vector3d& xyz, const Track& track);

    void AddObservation(const point3D_t point3D_id, const TrackElement& track_el);

    point3D_t MergePoints3D(const point3D_t point3D_id1,
                            const point3D_t point3D_id2);

    void DeletePoint3D(const point3D_t point3D_id);

    void DeleteObservation(const image_t image_id, const point2D_t point2D_idx);

    void RegisterImage(const image_t image_id);

    void DeRegisterImage(const image_t image_id);

    bool IsImageRegistered(const image_t image_id) const;

    void Normalize(const double extent = 10.0, const double p0 = 0.1,
                   const double p1 = 0.9, const bool use_images = true);

    const class Image* FindImageWithName(const std::string& name) const;

    size_t FilterPoints3D(const double max_reproj_error,
                          const double min_tri_angle,
                          const std::unordered_set<point3D_t>& point3D_ids);

    size_t FilterPoints3DInImages(const double max_reproj_error,
                                  const double min_tri_angle,
                                  const std::unordered_set<image_t>& image_ids);

    size_t FilterAllPoints3D(const double max_reproj_error,
                             const double min_tri_angle);

    size_t FilterObservationsWithNegativeDepth();

    std::vector<image_t> FilterImages(const double min_focal_length_ratio,
                                      const double max_focal_length_ratio,
                                      const double max_extra_param);

    size_t ComputeNumObservations() const;

    double ComputeMeanTrackLength() const;

    double ComputeMeanObservationsPerRegImage() const;

    double ComputeMeanReprojectionError() const;

    void ImportPLY(const std::string& path, bool append_to_existing = false);

    void ExportPLY(const std::string& path) const;

    void ExportBundler(const std::string& path,
                       const std::string& list_path) const;

    bool ExtractColors(const image_t image_id, const std::string& path);

private:
    size_t FilterPoints3DWithSmallTriangulationAngle(
            const double min_tri_angle,
            const std::unordered_set<point3D_t>& point3D_ids);

    size_t FilterPoints3DWithLargeReprojectionError(
            const double max_reproj_error,
            const std::unordered_set<point3D_t>& point3D_ids);

    void SetObservationAsTriangulated(const image_t image_id,
                                      const point2D_t point2D_idx,
                                      const bool is_continued_point3D);

    void ResetTriObservations(const image_t image_id, const point2D_t point2D_idx,
                              const bool is_deleted_point3D);

    const SceneGraph* scene_graph_;

    std::unordered_map<camera_t, class Camera> cameras_;
    std::unordered_map<image_t, class Image> images_;
    std::unordered_map<point3D_t, class Point_3D> points3D_;
    std::unordered_map<image_pair_t, std::pair<size_t, size_t>> image_pairs_;

    std::vector<image_t> reg_image_ids_;

    point3D_t num_added_points3D_;
};

#endif //INC_3D_RECONSTRUCTION_MODEL_H
