#ifndef INC_3D_RECONSTRUCTION_PROCESSOR_H
#define INC_3D_RECONSTRUCTION_PROCESSOR_H

#include "storage.h"
#include "model.h"
#include "optimization.h"
#include "refinement.h"

class IncrementalTriangulator {
public:
    struct Options {
        int max_transitivity = 1;

        double create_max_angle_error = 2.0;

        double continue_max_angle_error = 2.0;

        double merge_max_reproj_error = 4.0;

        double complete_max_reproj_error = 4.0;

        int complete_max_transitivity = 5;

        double re_max_angle_error = 5.0;

        double re_min_ratio = 0.2;

        int re_max_trials = 1;

        double min_angle = 1.5;

        bool ignore_two_view_tracks = true;

        double min_focal_length_ratio = 0.1;
        double max_focal_length_ratio = 10.0;
        double max_extra_param = 1.0;

        void Check() const;
    };

    IncrementalTriangulator(const SceneGraph* scene_graph,
                            Reconstruction* reconstruction);

    size_t TriangulateImage(const Options& options, const image_t image_id);

    size_t CompleteImage(const Options& options, const image_t image_id);

    size_t CompleteTracks(const Options& options,
                          const std::unordered_set<point3D_t>& point3D_ids);

    size_t CompleteAllTracks(const Options& options);

    size_t MergeTracks(const Options& options,
                       const std::unordered_set<point3D_t>& point3D_ids);

    size_t MergeAllTracks(const Options& options);

    size_t Retriangulate(const Options& options);

    std::unordered_set<point3D_t> ChangedPoints3D() const;

    void ClearChangedPoints3D();

private:
    struct CorrData {
        image_t image_id;
        point2D_t point2D_idx;
        const Image* image;
        const Camera* camera;
        const Point2D* point2D;
        Eigen::Matrix3x4d proj_matrix;
    };

    void ClearCaches();

    size_t Find(const Options& options, const image_t image_id,
                const point2D_t point2D_idx, const size_t transitivity,
                std::vector<CorrData>* corrs_data);

    size_t Create(const Options& options,
                  const std::vector<CorrData>& corrs_data);

    size_t Continue(const Options& options, const CorrData& ref_corr_data,
                    const std::vector<CorrData>& corrs_data);

    size_t Merge(const Options& options, const point3D_t point3D_id);

    size_t Complete(const Options& options, const point3D_t point3D_id);

    bool HasCameraBogusParams(const Options& options, const Camera& camera);

    const SceneGraph* scene_graph_;

    Reconstruction* reconstruction_;

    std::unordered_map<camera_t, bool> camera_has_bogus_params_;

    std::unordered_map<point3D_t, std::unordered_set<point3D_t>> merge_trials_;

    std::unordered_map<image_pair_t, int> re_num_trials_;

    std::unordered_set<point3D_t> changed_point3D_ids_;
};


class IncrementalMapper {
public:
    struct Options {
        int init_min_num_inliers = 50;
        double init_max_error = 4.0;
        double init_max_forward_motion = 0.95;
        double init_min_tri_angle = 4.0;
        double abs_pose_max_error = 12.0;
        int abs_pose_min_num_inliers = 30;
        double abs_pose_min_inlier_ratio = 0.25;
        bool abs_pose_estimate_focal_length = true;
        int local_ba_num_images = 6;
        double min_focal_length_ratio = 0.1;
        double max_focal_length_ratio = 10;
        double max_extra_param = 1;
        double filter_max_reproj_error = 4.0;
        double filter_min_tri_angle = 1.5;
        int max_reg_trials = 3;
        int num_threads = -1;
        enum class ImageSelectionMethod {
            MAX_VISIBLE_POINTS_NUM,
            MAX_VISIBLE_POINTS_RATIO,
            MIN_UNCERTAINTY,
        };
        ImageSelectionMethod image_selection_method =
                ImageSelectionMethod::MIN_UNCERTAINTY;

        void Check() const;
    };

    struct LocalBundleAdjustmentReport {
        size_t num_merged_observations = 0;
        size_t num_completed_observations = 0;
        size_t num_filtered_observations = 0;
        size_t num_adjusted_observations = 0;
    };

    IncrementalMapper(const DatabaseCache* database_cache);

    void BeginReconstruction(Reconstruction* reconstruction);

    void EndReconstruction(const bool discard);

    bool FindInitialImagePair(const Options& options, image_t* image_id1,
                              image_t* image_id2);

    std::vector<image_t> FindNextImages(const Options& options);

    bool RegisterInitialImagePair(const Options& options, const image_t image_id1,
                                  const image_t image_id2);

    bool RegisterNextImage(const Options& options, const image_t image_id);

    size_t TriangulateImage(const IncrementalTriangulator::Options& tri_options,
                            const image_t image_id);

    size_t Retriangulate(const IncrementalTriangulator::Options& tri_options);

    size_t CompleteTracks(const IncrementalTriangulator::Options& tri_options);

    size_t MergeTracks(const IncrementalTriangulator::Options& tri_options);

    LocalBundleAdjustmentReport AdjustLocalBundle(
            const Options& options, const BundleAdjuster::Options& ba_options,
            const IncrementalTriangulator::Options& tri_options,
            const image_t image_id);

    bool AdjustGlobalBundle(const BundleAdjuster::Options& ba_options);

    size_t FilterImages(const Options& options);

    size_t FilterPoints(const Options& options);

    size_t NumTotalRegImages() const;

    size_t NumSharedRegImages() const;

private:
    std::vector<image_t> FindFirstInitialImage() const;

    std::vector<image_t> FindSecondInitialImage(const image_t image_id1) const;

    std::vector<image_t> FindLocalBundle(const Options& options,
                                         const image_t image_id) const;

    void RegisterImageEvent(const image_t image_id);

    void DeRegisterImageEvent(const image_t image_id);

    bool EstimateInitialTwoViewGeometry(const Options& options,
                                        const image_t image_id1,
                                        const image_t image_id2);

    const DatabaseCache* database_cache_;

    Reconstruction* reconstruction_;

    std::unique_ptr<IncrementalTriangulator> triangulator_;

    size_t num_total_reg_images_;

    size_t num_shared_reg_images_;

    image_pair_t prev_init_image_pair_id_;
    TwoViewGeometry prev_init_two_view_geometry_;

    std::unordered_set<image_pair_t> tried_init_image_pairs_;

    std::unordered_set<camera_t> refined_cameras_;

    std::unordered_map<image_t, size_t> num_registrations_;

    std::unordered_set<image_t> filtered_images_;

    std::unordered_map<image_t, size_t> num_reg_trials_;
};

#endif //INC_3D_RECONSTRUCTION_PROCESSOR_H
