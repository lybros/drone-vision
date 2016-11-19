#ifndef INC_3D_RECONSTRUCTION_INCREMENTAL_TRIANGULATOR_H
#define INC_3D_RECONSTRUCTION_INCREMENTAL_TRIANGULATOR_H

#include "../storage.h"
#include "../reconstruction.h"
#include "../optimization.h"
#include "../refinement.h"

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

    size_t CompleteTracks(const Options& options, const std::unordered_set<point3D_t>& point3D_ids);

    size_t CompleteAllTracks(const Options& options);

    size_t MergeTracks(const Options& options, const std::unordered_set<point3D_t>& point3D_ids);

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

    size_t Create(const Options& options, const std::vector<CorrData>& corrs_data);

    size_t Continue(const Options& options, const CorrData& ref_corr_data, const std::vector<CorrData>& corrs_data);

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

#endif //INC_3D_RECONSTRUCTION_INCREMENTAL_TRIANGULATOR_H
