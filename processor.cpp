#include "processor.h"

void IncrementalTriangulator::Options::Check() const {
}

IncrementalTriangulator::IncrementalTriangulator(const SceneGraph* scene_graph,
                                                 Reconstruction* reconstruction)
        : scene_graph_(scene_graph), reconstruction_(reconstruction) {}

size_t IncrementalTriangulator::TriangulateImage(const Options& options,
                                                 const image_t image_id) {
    options.Check();

    size_t num_tris = 0;

    ClearCaches();

    const Image& image = reconstruction_->Image(image_id);
    if (!image.IsRegistered()) {
        return num_tris;
    }

    const Camera& camera = reconstruction_->Camera(image.CameraId());
    if (HasCameraBogusParams(options, camera)) {
        return num_tris;
    }

    CorrData ref_corr_data;
    ref_corr_data.image_id = image_id;
    ref_corr_data.image = &image;
    ref_corr_data.camera = &camera;
    ref_corr_data.proj_matrix = image.ProjectionMatrix();

    std::vector<CorrData> corrs_data;

    for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
         ++point2D_idx) {
        const size_t num_triangulated =
                Find(options, image_id, point2D_idx,
                     static_cast<size_t>(options.max_transitivity), &corrs_data);
        if (corrs_data.empty()) {
            continue;
        }

        const Point2D& point2D = image.Point2D(point2D_idx);
        ref_corr_data.point2D_idx = point2D_idx;
        ref_corr_data.point2D = &point2D;

        if (num_triangulated == 0) {
            corrs_data.push_back(ref_corr_data);
            num_tris += Create(options, corrs_data);
        } else {
            num_tris += Continue(options, ref_corr_data, corrs_data);
            corrs_data.push_back(ref_corr_data);
            num_tris += Create(options, corrs_data);
        }
    }

    return num_tris;
}

size_t IncrementalTriangulator::CompleteImage(const Options& options,
                                              const image_t image_id) {
    options.Check();

    size_t num_tris = 0;

    ClearCaches();

    const Image& image = reconstruction_->Image(image_id);
    if (!image.IsRegistered()) {
        return num_tris;
    }

    const Camera& camera = reconstruction_->Camera(image.CameraId());
    if (HasCameraBogusParams(options, camera)) {
        return num_tris;
    }

    CorrData ref_corr_data;
    ref_corr_data.image_id = image_id;
    ref_corr_data.image = &image;
    ref_corr_data.camera = &camera;
    ref_corr_data.proj_matrix = image.ProjectionMatrix();

    std::vector<CorrData> corrs_data;

    for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
         ++point2D_idx) {
        const Point2D& point2D = image.Point2D(point2D_idx);
        if (point2D.HasPoint3D()) {
            num_tris += Complete(options, point2D.Point3DId());
            continue;
        }

        if (options.ignore_two_view_tracks &&
            scene_graph_->IsTwoViewObservation(image_id, point2D_idx)) {
            continue;
        }

        const size_t num_triangulated =
                Find(options, image_id, point2D_idx,
                     static_cast<size_t>(options.max_transitivity), &corrs_data);
        if (num_triangulated || corrs_data.empty()) {
            continue;
        }

        ref_corr_data.point2D = &point2D;
        ref_corr_data.point2D_idx = point2D_idx;
        corrs_data.push_back(ref_corr_data);

        std::vector<TriangulationEstimator::PointData> point_data;
        point_data.resize(corrs_data.size());
        std::vector<TriangulationEstimator::PoseData> pose_data;
        pose_data.resize(corrs_data.size());
        for (size_t i = 0; i < corrs_data.size(); ++i) {
            const CorrData& corr_data = corrs_data[i];
            point_data[i].point = corr_data.point2D->XY();
            point_data[i].point_normalized =
                    corr_data.camera->ImageToWorld(point_data[i].point);
            pose_data[i].proj_matrix = corr_data.image->ProjectionMatrix();
            pose_data[i].proj_center = corr_data.image->ProjectionCenter();
            pose_data[i].camera = corr_data.camera;
        }

        EstimateTriangulationOptions tri_options;
        tri_options.min_tri_angle = DegToRad(options.min_angle);
        tri_options.residual_type =
                TriangulationEstimator::ResidualType::REPROJECTION_ERROR;
        tri_options.ransac_options.max_error = options.complete_max_reproj_error;
        tri_options.ransac_options.confidence = 0.9999;
        tri_options.ransac_options.min_inlier_ratio = 0.02;
        tri_options.ransac_options.max_num_trials = 10000;

        const size_t kExhaustiveSamplingThreshold = 15;
        if (point_data.size() <= kExhaustiveSamplingThreshold) {
            tri_options.ransac_options.min_num_trials =
                    NChooseK(point_data.size(), 2);
        }

        Eigen::Vector3d xyz;
        std::vector<bool> inlier_mask;
        if (!EstimateTriangulation(tri_options, point_data, pose_data, &inlier_mask,
                                   &xyz)) {
            continue;
        }

        Track track;
        track.Reserve(corrs_data.size());
        for (size_t i = 0; i < inlier_mask.size(); ++i) {
            if (inlier_mask[i]) {
                const CorrData& corr_data = corrs_data[i];
                track.AddElement(corr_data.image_id, corr_data.point2D_idx);
                num_tris += 1;
            }
        }

        const point3D_t point3D_id = reconstruction_->AddPoint3D(xyz, track);
        changed_point3D_ids_.insert(point3D_id);
    }

    return num_tris;
}

size_t IncrementalTriangulator::CompleteTracks(
        const Options& options, const std::unordered_set<point3D_t>& point3D_ids) {
    options.Check();

    size_t num_completed = 0;

    ClearCaches();

    for (const point3D_t point3D_id : point3D_ids) {
        num_completed += Complete(options, point3D_id);
    }

    return num_completed;
}

size_t IncrementalTriangulator::CompleteAllTracks(const Options& options) {
    options.Check();

    size_t num_completed = 0;

    ClearCaches();

    for (const point3D_t point3D_id : reconstruction_->Point3DIds()) {
        num_completed += Complete(options, point3D_id);
    }

    return num_completed;
}

size_t IncrementalTriangulator::MergeTracks(
        const Options& options, const std::unordered_set<point3D_t>& point3D_ids) {
    options.Check();

    size_t num_merged = 0;

    ClearCaches();

    for (const point3D_t point3D_id : point3D_ids) {
        num_merged += Merge(options, point3D_id);
    }

    return num_merged;
}

size_t IncrementalTriangulator::MergeAllTracks(const Options& options) {
    options.Check();

    size_t num_merged = 0;

    ClearCaches();

    for (const point3D_t point3D_id : reconstruction_->Point3DIds()) {
        num_merged += Merge(options, point3D_id);
    }

    return num_merged;
}

size_t IncrementalTriangulator::Retriangulate(const Options& options) {
    options.Check();

    size_t num_tris = 0;

    ClearCaches();

    Options re_options = options;
    re_options.continue_max_angle_error = options.re_max_angle_error;

    for (const auto& image_pair : reconstruction_->ImagePairs()) {
        const double tri_ratio =
                static_cast<double>(image_pair.second.first) / image_pair.second.second;
        if (tri_ratio >= options.re_min_ratio) {
            continue;
        }

        image_t image_id1;
        image_t image_id2;
        Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);

        const Image& image1 = reconstruction_->Image(image_id1);
        if (!image1.IsRegistered()) {
            continue;
        }

        const Image& image2 = reconstruction_->Image(image_id2);
        if (!image2.IsRegistered()) {
            continue;
        }

        int& num_re_trials = re_num_trials_[image_pair.first];
        if (num_re_trials >= options.re_max_trials) {
            continue;
        }
        num_re_trials += 1;

        const Eigen::Matrix3x4d proj_matrix1 = image1.ProjectionMatrix();
        const Eigen::Matrix3x4d proj_matrix2 = image2.ProjectionMatrix();

        const Camera& camera1 = reconstruction_->Camera(image1.CameraId());
        const Camera& camera2 = reconstruction_->Camera(image2.CameraId());
        if (HasCameraBogusParams(options, camera1) ||
            HasCameraBogusParams(options, camera2)) {
            continue;
        }

        const std::vector<std::pair<point2D_t, point2D_t>> corrs =
                scene_graph_->FindCorrespondencesBetweenImages(image_id1, image_id2);

        for (const auto& corr : corrs) {
            const Point2D& point2D1 = image1.Point2D(corr.first);
            const Point2D& point2D2 = image2.Point2D(corr.second);

            if (point2D1.HasPoint3D() && point2D2.HasPoint3D()) {
                continue;
            }

            CorrData corr_data1;
            corr_data1.image_id = image_id1;
            corr_data1.point2D_idx = corr.first;
            corr_data1.image = &image1;
            corr_data1.camera = &camera1;
            corr_data1.point2D = &point2D1;
            corr_data1.proj_matrix = proj_matrix1;

            CorrData corr_data2;
            corr_data2.image_id = image_id2;
            corr_data2.point2D_idx = corr.second;
            corr_data2.image = &image2;
            corr_data2.camera = &camera2;
            corr_data2.point2D = &point2D2;
            corr_data2.proj_matrix = proj_matrix2;

            if (point2D1.HasPoint3D() && !point2D2.HasPoint3D()) {
                const std::vector<CorrData> corrs_data1 = {corr_data1};
                num_tris += Continue(re_options, corr_data2, corrs_data1);
            } else if (!point2D1.HasPoint3D() && point2D2.HasPoint3D()) {
                const std::vector<CorrData> corrs_data2 = {corr_data2};
                num_tris += Continue(re_options, corr_data1, corrs_data2);
            } else if (!point2D1.HasPoint3D() && !point2D2.HasPoint3D()) {
                const std::vector<CorrData> corrs_data = {corr_data1, corr_data2};
                num_tris += Create(options, corrs_data);
            }
        }
    }

    return num_tris;
}

std::unordered_set<point3D_t> IncrementalTriangulator::ChangedPoints3D() const {
    std::unordered_set<point3D_t> point3D_ids;
    point3D_ids.reserve(changed_point3D_ids_.size());

    for (const point3D_t point3D_id : changed_point3D_ids_) {
        if (reconstruction_->ExistsPoint3D(point3D_id)) {
            point3D_ids.insert(point3D_id);
        }
    }

    return point3D_ids;
}

void IncrementalTriangulator::ClearChangedPoints3D() {
    changed_point3D_ids_.clear();
}

void IncrementalTriangulator::ClearCaches() {
    camera_has_bogus_params_.clear();
    merge_trials_.clear();
}

size_t IncrementalTriangulator::Find(const Options& options,
                                     const image_t image_id,
                                     const point2D_t point2D_idx,
                                     const size_t transitivity,
                                     std::vector<CorrData>* corrs_data) {
    const std::vector<SceneGraph::Correspondence>& corrs =
            scene_graph_->FindTransitiveCorrespondences(image_id, point2D_idx,
                                                        transitivity);

    corrs_data->clear();
    corrs_data->reserve(corrs.size());

    size_t num_triangulated = 0;

    for (const SceneGraph::Correspondence corr : corrs) {
        const Image& corr_image = reconstruction_->Image(corr.image_id);
        if (!corr_image.IsRegistered()) {
            continue;
        }

        const Camera& corr_camera = reconstruction_->Camera(corr_image.CameraId());
        if (HasCameraBogusParams(options, corr_camera)) {
            continue;
        }

        CorrData corr_data;
        corr_data.image_id = corr.image_id;
        corr_data.point2D_idx = corr.point2D_idx;
        corr_data.image = &corr_image;
        corr_data.camera = &corr_camera;
        corr_data.point2D = &corr_image.Point2D(corr.point2D_idx);
        corr_data.proj_matrix = corr_image.ProjectionMatrix();

        corrs_data->push_back(corr_data);

        if (corr_data.point2D->HasPoint3D()) {
            num_triangulated += 1;
        }
    }

    return num_triangulated;
}

size_t IncrementalTriangulator::Create(
        const Options& options, const std::vector<CorrData>& corrs_data) {
    std::vector<CorrData> create_corrs_data;
    create_corrs_data.reserve(corrs_data.size());
    for (const CorrData& corr_data : corrs_data) {
        if (!corr_data.point2D->HasPoint3D()) {
            create_corrs_data.push_back(corr_data);
        }
    }

    if (create_corrs_data.size() < 2) {
        return 0;
    } else if (options.ignore_two_view_tracks && create_corrs_data.size() == 2) {
        const CorrData& corr_data1 = create_corrs_data[0];
        if (scene_graph_->IsTwoViewObservation(corr_data1.image_id,
                                               corr_data1.point2D_idx)) {
            return 0;
        }
    }

    std::vector<TriangulationEstimator::PointData> point_data;
    point_data.resize(create_corrs_data.size());
    std::vector<TriangulationEstimator::PoseData> pose_data;
    pose_data.resize(create_corrs_data.size());
    for (size_t i = 0; i < create_corrs_data.size(); ++i) {
        const CorrData& corr_data = create_corrs_data[i];
        point_data[i].point = corr_data.point2D->XY();
        point_data[i].point_normalized =
                corr_data.camera->ImageToWorld(point_data[i].point);
        pose_data[i].proj_matrix = corr_data.image->ProjectionMatrix();
        pose_data[i].proj_center = corr_data.image->ProjectionCenter();
        pose_data[i].camera = corr_data.camera;
    }

    EstimateTriangulationOptions tri_options;
    tri_options.min_tri_angle = DegToRad(options.min_angle);
    tri_options.residual_type =
            TriangulationEstimator::ResidualType::ANGULAR_ERROR;
    tri_options.ransac_options.max_error =
            DegToRad(options.create_max_angle_error);
    tri_options.ransac_options.confidence = 0.9999;
    tri_options.ransac_options.min_inlier_ratio = 0.02;
    tri_options.ransac_options.max_num_trials = 10000;

    const size_t kExhaustiveSamplingThreshold = 15;
    if (point_data.size() <= kExhaustiveSamplingThreshold) {
        tri_options.ransac_options.min_num_trials = NChooseK(point_data.size(), 2);
    }

    Eigen::Vector3d xyz;
    std::vector<bool> inlier_mask;
    if (!EstimateTriangulation(tri_options, point_data, pose_data, &inlier_mask,
                               &xyz)) {
        return 0;
    }

    Track track;
    track.Reserve(create_corrs_data.size());
    for (size_t i = 0; i < inlier_mask.size(); ++i) {
        if (inlier_mask[i]) {
            const CorrData& corr_data = create_corrs_data[i];
            track.AddElement(corr_data.image_id, corr_data.point2D_idx);
        }
    }

    const point3D_t point3D_id = reconstruction_->AddPoint3D(xyz, track);
    changed_point3D_ids_.insert(point3D_id);

    const size_t kMinRecursiveTrackLength = 3;
    if (create_corrs_data.size() - track.Length() >= kMinRecursiveTrackLength) {
        return track.Length() + Create(options, create_corrs_data);
    }

    return track.Length();
}

size_t IncrementalTriangulator::Continue(
        const Options& options, const CorrData& ref_corr_data,
        const std::vector<CorrData>& corrs_data) {
    if (ref_corr_data.point2D->HasPoint3D()) {
        return 0;
    }

    double best_angle_error = std::numeric_limits<double>::max();
    size_t best_idx = std::numeric_limits<size_t>::max();

    for (size_t idx = 0; idx < corrs_data.size(); ++idx) {
        const CorrData& corr_data = corrs_data[idx];
        if (!corr_data.point2D->HasPoint3D()) {
            continue;
        }

        const Point_3D& point3D =
                reconstruction_->Point3D(corr_data.point2D->Point3DId());

        if (!HasPointPositiveDepth(ref_corr_data.proj_matrix, point3D.XYZ())) {
            continue;
        }

        const double angle_error =
                CalculateAngularError(ref_corr_data.point2D->XY(), point3D.XYZ(),
                                      ref_corr_data.proj_matrix, *ref_corr_data.camera);
        if (angle_error < best_angle_error) {
            best_angle_error = angle_error;
            best_idx = idx;
        }
    }

    const double max_angle_error = DegToRad(options.continue_max_angle_error);
    if (best_angle_error <= max_angle_error &&
        best_idx != std::numeric_limits<size_t>::max()) {
        const CorrData& corr_data = corrs_data[best_idx];
        const TrackElement track_el(ref_corr_data.image_id,
                                    ref_corr_data.point2D_idx);
        reconstruction_->AddObservation(corr_data.point2D->Point3DId(), track_el);
        changed_point3D_ids_.insert(corr_data.point2D->Point3DId());
        return 1;
    }

    return 0;
}

size_t IncrementalTriangulator::Merge(const Options& options,
                                      const point3D_t point3D_id) {
    size_t num_merged = 0;

    if (!reconstruction_->ExistsPoint3D(point3D_id)) {
        return num_merged;
    }

    const auto& point3D = reconstruction_->Point3D(point3D_id);

    for (const auto& track_el : point3D.Track().Elements()) {
        const std::vector<SceneGraph::Correspondence>& corrs =
                scene_graph_->FindCorrespondences(track_el.image_id,
                                                  track_el.point2D_idx);

        for (const auto corr : corrs) {
            Image& image = reconstruction_->Image(corr.image_id);
            if (!image.IsRegistered()) {
                continue;
            }

            const Point2D& corr_point2D = image.Point2D(corr.point2D_idx);
            if (!corr_point2D.HasPoint3D() ||
                corr_point2D.Point3DId() == point3D_id ||
                merge_trials_[point3D_id].count(corr_point2D.Point3DId()) > 0) {
                continue;
            }

            const Point_3D& corr_point3D =
                    reconstruction_->Point3D(corr_point2D.Point3DId());

            merge_trials_[point3D_id].insert(corr_point2D.Point3DId());
            merge_trials_[corr_point2D.Point3DId()].insert(point3D_id);

            const Eigen::Vector3d merged_xyz =
                    (point3D.Track().Length() * point3D.XYZ() +
                     corr_point3D.Track().Length() * corr_point3D.XYZ()) /
                    (point3D.Track().Length() + corr_point3D.Track().Length());

            size_t num_inliers = 0;
            for (const Track* track : {&point3D.Track(), &corr_point3D.Track()}) {
                for (const auto test_track_el : track->Elements()) {
                    const Image& test_image =
                            reconstruction_->Image(test_track_el.image_id);
                    const Camera& test_camera =
                            reconstruction_->Camera(test_image.CameraId());
                    const Point2D& test_point2D =
                            test_image.Point2D(test_track_el.point2D_idx);

                    const Eigen::Matrix3x4d test_proj_matrix =
                            test_image.ProjectionMatrix();

                    if (HasPointPositiveDepth(test_proj_matrix, merged_xyz) &&
                        CalculateReprojectionError(test_point2D.XY(), merged_xyz,
                                                   test_proj_matrix, test_camera) <=
                        options.merge_max_reproj_error) {
                        num_inliers += 1;
                    } else {
                        break;
                    }
                }
            }

            if (num_inliers ==
                point3D.Track().Length() + corr_point3D.Track().Length()) {
                num_merged += num_inliers;

                const point3D_t merged_point3D_id = reconstruction_->MergePoints3D(
                        point3D_id, corr_point2D.Point3DId());

                changed_point3D_ids_.erase(point3D_id);
                changed_point3D_ids_.erase(corr_point2D.Point3DId());
                changed_point3D_ids_.insert(merged_point3D_id);

                return num_merged + Merge(options, merged_point3D_id);
            }
        }
    }

    return num_merged;
}

size_t IncrementalTriangulator::Complete(const Options& options,
                                         const point3D_t point3D_id) {
    size_t num_completed = 0;

    if (!reconstruction_->ExistsPoint3D(point3D_id)) {
        return num_completed;
    }

    const Point_3D& point3D = reconstruction_->Point3D(point3D_id);

    std::vector<TrackElement> queue;
    queue.reserve(point3D.Track().Length());

    for (const auto& track_el : point3D.Track().Elements()) {
        queue.emplace_back(track_el.image_id, track_el.point2D_idx);
    }

    const int max_transitivity = options.complete_max_transitivity;
    for (int transitivity = 0; transitivity < max_transitivity; ++transitivity) {
        if (queue.empty()) {
            break;
        }

        const auto prev_queue = queue;
        queue.clear();

        for (const TrackElement queue_elem : prev_queue) {
            const std::vector<SceneGraph::Correspondence>& corrs =
                    scene_graph_->FindCorrespondences(queue_elem.image_id,
                                                      queue_elem.point2D_idx);

            for (const auto corr : corrs) {
                if (corr.image_id == queue_elem.image_id) {
                    continue;
                }

                const Image& image = reconstruction_->Image(corr.image_id);
                if (!image.IsRegistered()) {
                    continue;
                }

                const Point2D& point2D = image.Point2D(corr.point2D_idx);
                if (point2D.HasPoint3D()) {
                    continue;
                }

                const Eigen::Matrix3x4d proj_matrix = image.ProjectionMatrix();
                if (!HasPointPositiveDepth(proj_matrix, point3D.XYZ())) {
                    continue;
                }

                const Camera& camera = reconstruction_->Camera(image.CameraId());
                if (HasCameraBogusParams(options, camera)) {
                    continue;
                }

                const double reproj_error = CalculateReprojectionError(
                        point2D.XY(), point3D.XYZ(), proj_matrix, camera);
                if (reproj_error > options.complete_max_reproj_error) {
                    continue;
                }

                const TrackElement track_el(corr.image_id, corr.point2D_idx);
                reconstruction_->AddObservation(point3D_id, track_el);
                changed_point3D_ids_.insert(point3D_id);

                queue.emplace_back(corr.image_id, corr.point2D_idx);

                num_completed += 1;
            }
        }
    }

    return num_completed;
}

bool IncrementalTriangulator::HasCameraBogusParams(const Options& options,
                                                   const Camera& camera) {
    if (camera_has_bogus_params_.count(camera.CameraId()) == 0) {
        const bool has_bogus_params = camera.HasBogusParams(
                options.min_focal_length_ratio, options.max_focal_length_ratio,
                options.max_extra_param);
        camera_has_bogus_params_.emplace(camera.CameraId(), has_bogus_params);
        return has_bogus_params;
    } else {
        return camera_has_bogus_params_.at(camera.CameraId());
    }
}


namespace {
    void SortAndAppendNextImages(std::vector<std::pair<image_t, float>> image_ranks,
                                 std::vector<image_t>* sorted_images_ids) {
        std::sort(image_ranks.begin(), image_ranks.end(),
                  [](const std::pair<image_t, float>& image1,
                     const std::pair<image_t, float>& image2) {
                      return image1.second > image2.second;
                  });

        sorted_images_ids->reserve(sorted_images_ids->size() + image_ranks.size());
        for (const auto& image : image_ranks) {
            sorted_images_ids->push_back(image.first);
        }

        image_ranks.clear();
    }

    float RankNextImageMaxVisiblePointsNum(const Image& image) {
        return static_cast<float>(image.NumVisiblePoints3D());
    }

    float RankNextImageMaxVisiblePointsRatio(const Image& image) {
        return static_cast<float>(image.NumVisiblePoints3D()) /
               static_cast<float>(image.NumObservations());
    }

    float RankNextImageMinUncertainty(const Image& image) {
        return static_cast<float>(image.Point3DVisibilityScore());
    }

}

void IncrementalMapper::Options::Check() const {
}

IncrementalMapper::IncrementalMapper(const DatabaseCache* database_cache)
        : database_cache_(database_cache),
          reconstruction_(nullptr),
          triangulator_(nullptr),
          num_total_reg_images_(0),
          num_shared_reg_images_(0),
          prev_init_image_pair_id_(kInvalidImagePairId) {}

void IncrementalMapper::BeginReconstruction(Reconstruction* reconstruction) {
    reconstruction_ = reconstruction;
    reconstruction_->Load(*database_cache_);
    reconstruction_->SetUp(&database_cache_->SceneGraph());
    triangulator_.reset(new IncrementalTriangulator(
            &database_cache_->SceneGraph(), reconstruction));

    num_shared_reg_images_ = 0;
    for (const image_t image_id : reconstruction_->RegImageIds()) {
        RegisterImageEvent(image_id);
    }

    prev_init_image_pair_id_ = kInvalidImagePairId;
    prev_init_two_view_geometry_ = TwoViewGeometry();

    refined_cameras_.clear();
    filtered_images_.clear();
    num_reg_trials_.clear();
}

void IncrementalMapper::EndReconstruction(const bool discard) {
    if (discard) {
        for (const image_t image_id : reconstruction_->RegImageIds()) {
            DeRegisterImageEvent(image_id);
        }
    }

    reconstruction_->TearDown();
    reconstruction_ = nullptr;
    triangulator_.reset();
}

bool IncrementalMapper::FindInitialImagePair(const Options& options,
                                             image_t* image_id1,
                                             image_t* image_id2) {
    options.Check();

    std::vector<image_t> image_ids1;
    if (*image_id1 != kInvalidImageId && *image_id2 == kInvalidImageId) {
        if (!database_cache_->ExistsImage(*image_id1)) {
            return false;
        }
        image_ids1.push_back(*image_id1);
    } else if (*image_id1 == kInvalidImageId && *image_id2 != kInvalidImageId) {
        if (!database_cache_->ExistsImage(*image_id2)) {
            return false;
        }
        image_ids1.push_back(*image_id2);
    } else {
        image_ids1 = FindFirstInitialImage();
    }

    for (size_t i1 = 0; i1 < image_ids1.size(); ++i1) {
        *image_id1 = image_ids1[i1];

        const std::vector<image_t> image_ids2 = FindSecondInitialImage(*image_id1);

        for (size_t i2 = 0; i2 < image_ids2.size(); ++i2) {
            *image_id2 = image_ids2[i2];

            const image_pair_t pair_id =
                    Database::ImagePairToPairId(*image_id1, *image_id2);

            if (tried_init_image_pairs_.count(pair_id) > 0) {
                continue;
            }

            tried_init_image_pairs_.insert(pair_id);

            if (EstimateInitialTwoViewGeometry(options, *image_id1, *image_id2)) {
                return true;
            }
        }
    }

    *image_id1 = kInvalidImageId;
    *image_id2 = kInvalidImageId;

    return false;
}

std::vector<image_t> IncrementalMapper::FindNextImages(const Options& options) {
    options.Check();

    std::function<float(const Image&)> rank_image_func;
    switch (options.image_selection_method) {
        case Options::ImageSelectionMethod::MAX_VISIBLE_POINTS_NUM:
            rank_image_func = RankNextImageMaxVisiblePointsNum;
            break;
        case Options::ImageSelectionMethod::MAX_VISIBLE_POINTS_RATIO:
            rank_image_func = RankNextImageMaxVisiblePointsRatio;
            break;
        case Options::ImageSelectionMethod::MIN_UNCERTAINTY:
            rank_image_func = RankNextImageMinUncertainty;
            break;
    }

    std::vector<std::pair<image_t, float>> image_ranks;
    std::vector<std::pair<image_t, float>> other_image_ranks;

    for (const auto& image : reconstruction_->Images()) {
        if (image.second.IsRegistered()) {
            continue;
        }

        if (image.second.NumVisiblePoints3D() <
            static_cast<size_t>(options.abs_pose_min_num_inliers)) {
            continue;
        }

        const size_t num_reg_trials = num_reg_trials_[image.first];
        if (num_reg_trials >= static_cast<size_t>(options.max_reg_trials)) {
            continue;
        }

        const float rank = rank_image_func(image.second);
        if (filtered_images_.count(image.first) == 0 && num_reg_trials == 0) {
            image_ranks.emplace_back(image.first, rank);
        } else {
            other_image_ranks.emplace_back(image.first, rank);
        }
    }

    std::vector<image_t> ranked_images_ids;
    SortAndAppendNextImages(image_ranks, &ranked_images_ids);
    SortAndAppendNextImages(other_image_ranks, &ranked_images_ids);

    return ranked_images_ids;
}

bool IncrementalMapper::RegisterInitialImagePair(const Options& options,
                                                 const image_t image_id1,
                                                 const image_t image_id2) {
    options.Check();

    num_reg_trials_[image_id1] += 1;
    num_reg_trials_[image_id2] += 1;

    const image_pair_t pair_id =
            Database::ImagePairToPairId(image_id1, image_id2);
    tried_init_image_pairs_.insert(pair_id);

    Image& image1 = reconstruction_->Image(image_id1);
    const Camera& camera1 = reconstruction_->Camera(image1.CameraId());

    Image& image2 = reconstruction_->Image(image_id2);
    const Camera& camera2 = reconstruction_->Camera(image2.CameraId());

    if (!EstimateInitialTwoViewGeometry(options, image_id1, image_id2)) {
        return false;
    }

    image1.Qvec() = Eigen::Vector4d(1, 0, 0, 0);
    image1.Tvec() = Eigen::Vector3d(0, 0, 0);
    image2.Qvec() = prev_init_two_view_geometry_.qvec;
    image2.Tvec() = prev_init_two_view_geometry_.tvec;

    const Eigen::Matrix3x4d proj_matrix1 = image1.ProjectionMatrix();
    const Eigen::Matrix3x4d proj_matrix2 = image2.ProjectionMatrix();
    const Eigen::Vector3d proj_center1 = image1.ProjectionCenter();
    const Eigen::Vector3d proj_center2 = image2.ProjectionCenter();

    reconstruction_->RegisterImage(image_id1);
    reconstruction_->RegisterImage(image_id2);
    RegisterImageEvent(image_id1);
    RegisterImageEvent(image_id2);

    const SceneGraph& scene_graph = database_cache_->SceneGraph();
    const std::vector<std::pair<point2D_t, point2D_t>>& corrs =
            scene_graph.FindCorrespondencesBetweenImages(image_id1, image_id2);

    const double min_tri_angle_rad = DegToRad(options.init_min_tri_angle);

    Track track;
    track.Reserve(2);
    track.AddElement(TrackElement());
    track.AddElement(TrackElement());
    track.Element(0).image_id = image_id1;
    track.Element(1).image_id = image_id2;
    for (size_t i = 0; i < corrs.size(); ++i) {
        const point2D_t point2D_idx1 = corrs[i].first;
        const point2D_t point2D_idx2 = corrs[i].second;
        const Eigen::Vector2d point1_N =
                camera1.ImageToWorld(image1.Point2D(point2D_idx1).XY());
        const Eigen::Vector2d point2_N =
                camera2.ImageToWorld(image2.Point2D(point2D_idx2).XY());
        const Eigen::Vector3d& xyz =
                TriangulatePoint(proj_matrix1, proj_matrix2, point1_N, point2_N);
        const double tri_angle =
                CalculateTriangulationAngle(proj_center1, proj_center2, xyz);
        if (tri_angle >= min_tri_angle_rad &&
            HasPointPositiveDepth(proj_matrix1, xyz) &&
            HasPointPositiveDepth(proj_matrix2, xyz)) {
            track.Element(0).point2D_idx = point2D_idx1;
            track.Element(1).point2D_idx = point2D_idx2;
            reconstruction_->AddPoint3D(xyz, track);
        }
    }

    return true;
}

bool IncrementalMapper::RegisterNextImage(const Options& options,
                                          const image_t image_id) {
    options.Check();

    Image& image = reconstruction_->Image(image_id);
    Camera& camera = reconstruction_->Camera(image.CameraId());

    num_reg_trials_[image_id] += 1;

    if (image.NumVisiblePoints3D() <
        static_cast<size_t>(options.abs_pose_min_num_inliers)) {
        return false;
    }

    const int kCorrTransitivity = 1;

    std::vector<std::pair<point2D_t, point3D_t>> tri_corrs;
    std::vector<Eigen::Vector2d> tri_points2D;
    std::vector<Eigen::Vector3d> tri_points3D;

    for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
         ++point2D_idx) {
        const Point2D& point2D = image.Point2D(point2D_idx);
        const SceneGraph& scene_graph = database_cache_->SceneGraph();
        const std::vector<SceneGraph::Correspondence> corrs =
                scene_graph.FindTransitiveCorrespondences(image_id, point2D_idx,
                                                          kCorrTransitivity);

        std::unordered_set<point3D_t> point3D_ids;

        for (const auto corr : corrs) {
            const Image& corr_image = reconstruction_->Image(corr.image_id);
            if (!corr_image.IsRegistered()) {
                continue;
            }

            const Point2D& corr_point2D = corr_image.Point2D(corr.point2D_idx);
            if (!corr_point2D.HasPoint3D()) {
                continue;
            }

            if (point3D_ids.count(corr_point2D.Point3DId()) > 0) {
                continue;
            }

            const Camera& corr_camera =
                    reconstruction_->Camera(corr_image.CameraId());

            if (corr_camera.HasBogusParams(options.min_focal_length_ratio,
                                           options.max_focal_length_ratio,
                                           options.max_extra_param)) {
                continue;
            }

            const Point_3D& point3D =
                    reconstruction_->Point3D(corr_point2D.Point3DId());

            tri_corrs.emplace_back(point2D_idx, corr_point2D.Point3DId());
            point3D_ids.insert(corr_point2D.Point3DId());
            tri_points2D.push_back(point2D.XY());
            tri_points3D.push_back(point3D.XYZ());
        }
    }

    if (tri_points2D.size() <
        static_cast<size_t>(options.abs_pose_min_num_inliers)) {
        return false;
    }

    AbsolutePoseEstimationOptions abs_pose_options;
    abs_pose_options.num_threads = options.num_threads;
    abs_pose_options.num_focal_length_samples = 30;
    abs_pose_options.min_focal_length_ratio = options.min_focal_length_ratio;
    abs_pose_options.max_focal_length_ratio = options.max_focal_length_ratio;
    abs_pose_options.ransac_options.max_error = options.abs_pose_max_error;
    abs_pose_options.ransac_options.min_inlier_ratio =
            options.abs_pose_min_inlier_ratio;
    abs_pose_options.ransac_options.confidence = 0.9999;

    AbsolutePoseRefinementOptions abs_pose_refinement_options;
    if (refined_cameras_.count(image.CameraId()) > 0) {
        if (camera.HasBogusParams(options.min_focal_length_ratio,
                                  options.max_focal_length_ratio,
                                  options.max_extra_param)) {
            refined_cameras_.erase(image.CameraId());
            camera.SetParams(database_cache_->Camera(image.CameraId()).Params());
            abs_pose_options.estimate_focal_length = !camera.HasPriorFocalLength();
            abs_pose_refinement_options.refine_focal_length = true;
        } else {
            abs_pose_options.estimate_focal_length = false;
            abs_pose_refinement_options.refine_focal_length = false;
        }
    } else {
        abs_pose_options.estimate_focal_length = !camera.HasPriorFocalLength();
        abs_pose_refinement_options.refine_focal_length = true;
    }

    if (!options.abs_pose_estimate_focal_length) {
        abs_pose_options.estimate_focal_length = false;
        abs_pose_refinement_options.refine_focal_length = false;
    }

    size_t num_inliers;
    std::vector<bool> inlier_mask;

    if (!EstimateAbsolutePose(abs_pose_options, tri_points2D, tri_points3D,
                              &image.Qvec(), &image.Tvec(), &camera, &num_inliers,
                              &inlier_mask)) {
        return false;
    }

    if (num_inliers < static_cast<size_t>(options.abs_pose_min_num_inliers)) {
        return false;
    }

    if (!RefineAbsolutePose(abs_pose_refinement_options, inlier_mask,
                            tri_points2D, tri_points3D, &image.Qvec(),
                            &image.Tvec(), &camera)) {
        return false;
    }

    reconstruction_->RegisterImage(image_id);
    RegisterImageEvent(image_id);

    for (size_t i = 0; i < inlier_mask.size(); ++i) {
        if (inlier_mask[i]) {
            const point2D_t point2D_idx = tri_corrs[i].first;
            const Point2D& point2D = image.Point2D(point2D_idx);
            if (!point2D.HasPoint3D()) {
                const point3D_t point3D_id = tri_corrs[i].second;
                const TrackElement track_el(image_id, point2D_idx);
                reconstruction_->AddObservation(point3D_id, track_el);
            }
        }
    }

    refined_cameras_.insert(image.CameraId());

    return true;
}

size_t IncrementalMapper::TriangulateImage(
        const IncrementalTriangulator::Options& tri_options,
        const image_t image_id) {
    return triangulator_->TriangulateImage(tri_options, image_id);
}

size_t IncrementalMapper::Retriangulate(
        const IncrementalTriangulator::Options& tri_options) {
    return triangulator_->Retriangulate(tri_options);
}

size_t IncrementalMapper::CompleteTracks(
        const IncrementalTriangulator::Options& tri_options) {
    return triangulator_->CompleteAllTracks(tri_options);
}

size_t IncrementalMapper::MergeTracks(
        const IncrementalTriangulator::Options& tri_options) {
    return triangulator_->MergeAllTracks(tri_options);
}

IncrementalMapper::LocalBundleAdjustmentReport
IncrementalMapper::AdjustLocalBundle(
        const Options& options, const BundleAdjuster::Options& ba_options,
        const IncrementalTriangulator::Options& tri_options,
        const image_t image_id) {
    options.Check();

    LocalBundleAdjustmentReport report;

    const std::vector<image_t> local_bundle = FindLocalBundle(options, image_id);

    if (local_bundle.size() > 0) {
        BundleAdjustmentConfiguration ba_config;
        ba_config.AddImage(image_id);
        for (const image_t local_image_id : local_bundle) {
            ba_config.AddImage(local_image_id);
        }

        image_t constant_image_id = kInvalidImageId;
        if (local_bundle.size() == 1) {
            ba_config.SetConstantPose(local_bundle[0]);
            ba_config.SetConstantTvec(image_id, {0});
            constant_image_id = local_bundle[0];
        } else if (local_bundle.size() > 1) {
            ba_config.SetConstantPose(local_bundle[local_bundle.size() - 1]);
            ba_config.SetConstantTvec(local_bundle[local_bundle.size() - 2], {0});
            constant_image_id = local_bundle[local_bundle.size() - 1];
        }

        const Image& constant_image = reconstruction_->Image(constant_image_id);
        ba_config.SetConstantCamera(constant_image.CameraId());

        std::unordered_set<point3D_t> variable_point3D_ids;
        for (const point3D_t point3D_id : triangulator_->ChangedPoints3D()) {
            const Point_3D& point3D = reconstruction_->Point3D(point3D_id);
            const size_t kMaxTrackLength = 15;
            if (!point3D.HasError() || point3D.Track().Length() <= kMaxTrackLength) {
                ba_config.AddVariablePoint(point3D_id);
                variable_point3D_ids.insert(point3D_id);
            }
        }

        BundleAdjuster bundle_adjuster(ba_options, ba_config);
        bundle_adjuster.Solve(reconstruction_);

        report.num_adjusted_observations =
                bundle_adjuster.Summary().num_residuals / 2;

        report.num_merged_observations =
                triangulator_->MergeTracks(tri_options, variable_point3D_ids);
        report.num_completed_observations =
                triangulator_->CompleteTracks(tri_options, variable_point3D_ids);
        report.num_completed_observations +=
                triangulator_->CompleteImage(tri_options, image_id);
    }

    std::unordered_set<image_t> filter_image_ids;
    filter_image_ids.insert(image_id);
    filter_image_ids.insert(local_bundle.begin(), local_bundle.end());
    report.num_filtered_observations = reconstruction_->FilterPoints3DInImages(
            options.filter_max_reproj_error, options.filter_min_tri_angle,
            filter_image_ids);
    report.num_filtered_observations += reconstruction_->FilterPoints3D(
            options.filter_max_reproj_error, options.filter_min_tri_angle,
            triangulator_->ChangedPoints3D());

    triangulator_->ClearChangedPoints3D();

    return report;
}

bool IncrementalMapper::AdjustGlobalBundle(
        const BundleAdjuster::Options& ba_options) {

    const std::vector<image_t>& reg_image_ids = reconstruction_->RegImageIds();

    reconstruction_->FilterObservationsWithNegativeDepth();

    BundleAdjustmentConfiguration ba_config;
    for (const image_t image_id : reg_image_ids) {
        ba_config.AddImage(image_id);
    }
    ba_config.SetConstantPose(reg_image_ids[0]);
    ba_config.SetConstantTvec(reg_image_ids[1], {0});

    BundleAdjuster bundle_adjuster(ba_options, ba_config);
    if (!bundle_adjuster.Solve(reconstruction_)) {
        return false;
    }

    reconstruction_->Normalize();

    return true;
}

size_t IncrementalMapper::FilterImages(const Options& options) {
    options.Check();

    const size_t kMinNumImages = 20;
    if (reconstruction_->NumRegImages() < kMinNumImages) {
        return {};
    }

    const std::vector<image_t> image_ids = reconstruction_->FilterImages(
            options.min_focal_length_ratio, options.max_focal_length_ratio,
            options.max_extra_param);

    for (const image_t image_id : image_ids) {
        DeRegisterImageEvent(image_id);
        filtered_images_.insert(image_id);
    }

    return image_ids.size();
}

size_t IncrementalMapper::FilterPoints(const Options& options) {
    options.Check();
    return reconstruction_->FilterAllPoints3D(options.filter_max_reproj_error,
                                              options.filter_min_tri_angle);
}

size_t IncrementalMapper::NumTotalRegImages() const {
    return num_total_reg_images_;
}

size_t IncrementalMapper::NumSharedRegImages() const {
    return num_shared_reg_images_;
}

std::vector<image_t> IncrementalMapper::FindFirstInitialImage() const {
    struct ImageInfo {
        image_t image_id;
        bool prior_focal_length;
        image_t num_correspondences;
    };

    std::vector<ImageInfo> image_infos;
    image_infos.reserve(reconstruction_->NumImages());
    for (const auto& image : reconstruction_->Images()) {
        if (image.second.NumCorrespondences() == 0) {
            continue;
        }

        if (num_registrations_.count(image.first) > 0 &&
            num_registrations_.at(image.first) > 0) {
            continue;
        }

        const class Camera& camera =
                reconstruction_->Camera(image.second.CameraId());
        ImageInfo image_info;
        image_info.image_id = image.first;
        image_info.prior_focal_length = camera.HasPriorFocalLength();
        image_info.num_correspondences = image.second.NumCorrespondences();
        image_infos.push_back(image_info);
    }

    std::sort(
            image_infos.begin(), image_infos.end(),
            [](const ImageInfo& image_info1, const ImageInfo& image_info2) {
                if (image_info1.prior_focal_length && !image_info2.prior_focal_length) {
                    return true;
                } else if (!image_info1.prior_focal_length &&
                           image_info2.prior_focal_length) {
                    return false;
                } else {
                    return image_info1.num_correspondences >
                           image_info2.num_correspondences;
                }
            });

    std::vector<image_t> image_ids;
    image_ids.reserve(image_infos.size());
    for (const ImageInfo& image_info : image_infos) {
        image_ids.push_back(image_info.image_id);
    }

    return image_ids;
}

std::vector<image_t> IncrementalMapper::FindSecondInitialImage(
        const image_t image_id1) const {
    const SceneGraph& scene_graph = database_cache_->SceneGraph();

    const class Image& image1 = reconstruction_->Image(image_id1);
    std::unordered_map<image_t, point2D_t> num_correspondences;
    for (point2D_t point2D_idx = 0; point2D_idx < image1.NumPoints2D();
         ++point2D_idx) {
        const std::vector<SceneGraph::Correspondence>& corrs =
                scene_graph.FindCorrespondences(image_id1, point2D_idx);
        for (const SceneGraph::Correspondence& corr : corrs) {
            if (num_registrations_.count(corr.image_id) == 0 ||
                num_registrations_.at(corr.image_id) == 0) {
                num_correspondences[corr.image_id] += 1;
            }
        }
    }

    struct ImageInfo {
        image_t image_id;
        bool prior_focal_length;
        point2D_t num_correspondences;
    };

    std::vector<ImageInfo> image_infos;
    image_infos.reserve(reconstruction_->NumImages());
    for (const auto elem : num_correspondences) {
        const class Image& image = reconstruction_->Image(elem.first);
        const class Camera& camera = reconstruction_->Camera(image.CameraId());
        ImageInfo image_info;
        image_info.image_id = elem.first;
        image_info.prior_focal_length = camera.HasPriorFocalLength();
        image_info.num_correspondences = elem.second;
        image_infos.push_back(image_info);
    }

    std::sort(
            image_infos.begin(), image_infos.end(),
            [](const ImageInfo& image_info1, const ImageInfo& image_info2) {
                if (image_info1.prior_focal_length && !image_info2.prior_focal_length) {
                    return true;
                } else if (!image_info1.prior_focal_length &&
                           image_info2.prior_focal_length) {
                    return false;
                } else {
                    return image_info1.num_correspondences >
                           image_info2.num_correspondences;
                }
            });

    std::vector<image_t> image_ids;
    image_ids.reserve(image_infos.size());
    for (const ImageInfo& image_info : image_infos) {
        image_ids.push_back(image_info.image_id);
    }

    return image_ids;
}

std::vector<image_t> IncrementalMapper::FindLocalBundle(
        const Options& options, const image_t image_id) const {
    options.Check();

    const Image& image = reconstruction_->Image(image_id);

    std::unordered_map<image_t, size_t> num_shared_observations;
    for (const Point2D& point2D : image.Points2D()) {
        if (point2D.HasPoint3D()) {
            const Point_3D& point3D = reconstruction_->Point3D(point2D.Point3DId());
            for (const TrackElement& track_el : point3D.Track().Elements()) {
                if (track_el.image_id != image_id) {
                    num_shared_observations[track_el.image_id] += 1;
                }
            }
        }
    }

    std::vector<std::pair<image_t, size_t>> local_bundle;
    for (const auto elem : num_shared_observations) {
        local_bundle.emplace_back(elem.first, elem.second);
    }

    const size_t num_images =
            static_cast<size_t>(options.local_ba_num_images - 1);
    const size_t num_eff_images = std::min(num_images, local_bundle.size());

    std::partial_sort(local_bundle.begin(), local_bundle.begin() + num_eff_images,
                      local_bundle.end(),
                      [](const std::pair<image_t, size_t>& image1,
                         const std::pair<image_t, size_t>& image2) {
                          return image1.second > image2.second;
                      });

    std::vector<image_t> image_ids(num_eff_images);
    for (size_t i = 0; i < num_eff_images; ++i) {
        image_ids[i] = local_bundle[i].first;
    }

    return image_ids;
}

void IncrementalMapper::RegisterImageEvent(const image_t image_id) {
    size_t& num_regs_for_image = num_registrations_[image_id];
    num_regs_for_image += 1;
    if (num_regs_for_image == 1) {
        num_total_reg_images_ += 1;
    } else if (num_regs_for_image > 1) {
        num_shared_reg_images_ += 1;
    }
}

void IncrementalMapper::DeRegisterImageEvent(const image_t image_id) {
    size_t& num_regs_for_image = num_registrations_[image_id];
    num_regs_for_image -= 1;
    if (num_regs_for_image == 0) {
        num_total_reg_images_ -= 1;
    } else if (num_regs_for_image > 0) {
        num_shared_reg_images_ -= 1;
    }
}

bool IncrementalMapper::EstimateInitialTwoViewGeometry(
        const Options& options, const image_t image_id1, const image_t image_id2) {
    const image_pair_t image_pair_id =
            Database::ImagePairToPairId(image_id1, image_id2);

    if (prev_init_image_pair_id_ == image_pair_id) {
        return true;
    }

    const Image& image1 = database_cache_->Image(image_id1);
    const Camera& camera1 = database_cache_->Camera(image1.CameraId());

    const Image& image2 = database_cache_->Image(image_id2);
    const Camera& camera2 = database_cache_->Camera(image2.CameraId());

    const SceneGraph& scene_graph = database_cache_->SceneGraph();
    const std::vector<std::pair<point2D_t, point2D_t>>& corrs =
            scene_graph.FindCorrespondencesBetweenImages(image_id1, image_id2);

    std::vector<Eigen::Vector2d> points1;
    points1.reserve(image1.NumPoints2D());
    for (const auto& point : image1.Points2D()) {
        points1.push_back(point.XY());
    }

    std::vector<Eigen::Vector2d> points2;
    points2.reserve(image2.NumPoints2D());
    for (const auto& point : image2.Points2D()) {
        points2.push_back(point.XY());
    }

    FeatureMatches matches(corrs.size());
    for (size_t i = 0; i < corrs.size(); ++i) {
        matches[i].point2D_idx1 = corrs[i].first;
        matches[i].point2D_idx2 = corrs[i].second;
    }

    TwoViewGeometry two_view_geometry;
    TwoViewGeometry::Options two_view_geometry_options;
    two_view_geometry_options.ransac_options.max_error = options.init_max_error;
    two_view_geometry.EstimateWithRelativePose(
            camera1, points1, camera2, points2, matches, two_view_geometry_options);

    if (static_cast<int>(two_view_geometry.inlier_matches.size()) >=
        options.init_min_num_inliers &&
        std::abs(two_view_geometry.tvec.z()) < options.init_max_forward_motion &&
        two_view_geometry.tri_angle > DegToRad(options.init_min_tri_angle)) {
        prev_init_image_pair_id_ = image_pair_id;
        prev_init_two_view_geometry_ = two_view_geometry;
        return true;
    }

    return false;
}
