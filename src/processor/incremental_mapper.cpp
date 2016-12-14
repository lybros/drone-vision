#include "incremental_mapper.h"

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

// This method begins reconstruction.
// Called only once from run() method of IncrementalMapperController.
void IncrementalMapper::BeginReconstruction(Reconstruction* reconstruction) {
    reconstruction_ = reconstruction;

    // Retrieving cameras, images and correspondences between images from database cache.
    reconstruction_->Load(*database_cache_);

    // Setting up images.
    // Seems like here we're looking for corresponded 2d points.
    // Quiet much magic happens here.
    reconstruction_->SetUp(&database_cache_->SceneGraph());

    // Here we're initializing our mapper with IncrementalTriangulator, which is initialized with Scene Graph from
    // database cache and our current reconstruction.
    triangulator_.reset(new IncrementalTriangulator(&database_cache_->SceneGraph(), reconstruction));

    num_shared_reg_images_ = 0;
    // I can't see where reg_image_ids_ was initialized before we reached this line.
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

// Method searches for good initial pair of images to proceed with reconstruction process.
bool IncrementalMapper::FindInitialImagePair(const Options& options, image_t* image_id1, image_t* image_id2) {
    // Method that does nothing.
    options.Check();

    std::vector<image_t> image_ids1;
    // Checking if only one of image_ids is invalid or both of them.
    // We want to choose one image and then to look for a good second image for it.
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
        // If both images are not valid (not images, in fact).
        // So here by some criteria we're getting a sorted vector of image_ids.
        image_ids1 = FindFirstInitialImage();
    }

    for (size_t i1 = 0; i1 < image_ids1.size(); ++i1) {
        *image_id1 = image_ids1[i1];

        // Looking for a good fit for our first image chosen before.
        const std::vector<image_t> image_ids2 = FindSecondInitialImage(*image_id1);

        for (size_t i2 = 0; i2 < image_ids2.size(); ++i2) {
            *image_id2 = image_ids2[i2];

            const image_pair_t pair_id = Database::ImagePairToPairId(*image_id1, *image_id2);

            if (tried_init_image_pairs_.count(pair_id) > 0) {
                continue;
            }

            tried_init_image_pairs_.insert(pair_id);

            // So geometry magic in order to understand if chosen pair of images is a good choice for being
            // "initializing images". As far as I remember we're checking, if one image can't be got from another
            // by simple homography.
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

    // Choosing what approach we're going to use to iteratively add new images.
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

        if (image.second.NumVisiblePoints3D() < static_cast<size_t>(options.abs_pose_min_num_inliers)) {
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
    // We've counted image ranks, which we then use to sort images in an appropriate order and to return back.

    std::vector<image_t> ranked_images_ids;
    SortAndAppendNextImages(image_ranks, &ranked_images_ids);
    SortAndAppendNextImages(other_image_ranks, &ranked_images_ids);

    // Images in the correct order to be processed with.
    return ranked_images_ids;
}

bool IncrementalMapper::RegisterInitialImagePair(const Options& options,
                                                 const image_t image_id1,
                                                 const image_t image_id2) {
    options.Check();

    num_reg_trials_[image_id1] += 1;
    num_reg_trials_[image_id2] += 1;

    const image_pair_t pair_id = Database::ImagePairToPairId(image_id1, image_id2);
    tried_init_image_pairs_.insert(pair_id);

    Image& image1 = reconstruction_->Image(image_id1);
    const Camera& camera1 = reconstruction_->Camera(image1.CameraId());

    Image& image2 = reconstruction_->Image(image_id2);
    const Camera& camera2 = reconstruction_->Camera(image2.CameraId());

    // Ok so now we have image1 and image2 which are initial images.
    // Checking it with geometry magic again.
    // The same function we've run while searching for initial pair.
    // Well, ok, why not to run it again.
    if (!EstimateInitialTwoViewGeometry(options, image_id1, image_id2)) {
        return false;
    }

    // Choosing image1 as point 0 of the model.
    // Locating image2 in regard to image1.
    image1.Qvec() = Eigen::Vector4d(1, 0, 0, 0);
    image1.Tvec() = Eigen::Vector3d(0, 0, 0);
    image2.Qvec() = prev_init_two_view_geometry_.qvec;
    image2.Tvec() = prev_init_two_view_geometry_.tvec;

    // Getting all this projection stuff. Where it goes from is so deep that it's not clear.
    const Eigen::Matrix3x4d proj_matrix1 = image1.ProjectionMatrix();
    const Eigen::Matrix3x4d proj_matrix2 = image2.ProjectionMatrix();
    const Eigen::Vector3d proj_center1 = image1.ProjectionCenter();
    const Eigen::Vector3d proj_center2 = image2.ProjectionCenter();

    // But now we have some projection/approximation matrices. Which is good.

    // Finally registering images.
    reconstruction_->RegisterImage(image_id1);
    reconstruction_->RegisterImage(image_id2);
    RegisterImageEvent(image_id1);
    RegisterImageEvent(image_id2);

    const SceneGraph& scene_graph = database_cache_->SceneGraph();
    const std::vector<std::pair<point2D_t, point2D_t>>& corrs =
            scene_graph.FindCorrespondencesBetweenImages(image_id1, image_id2);

    const double min_tri_angle_rad = DegToRad(options.init_min_tri_angle);

    Track track;
    // As for now our track is just two 2d points corresponding to the same 3d point (due to definition what track is).
    track.Reserve(2);
    track.AddElement(TrackElement());
    track.AddElement(TrackElement());
    track.Element(0).image_id = image_id1;
    track.Element(1).image_id = image_id2;
    for (size_t i = 0; i < corrs.size(); ++i) {
        const point2D_t point2D_idx1 = corrs[i].first;
        const point2D_t point2D_idx2 = corrs[i].second;
        const Eigen::Vector2d point1_N = camera1.ImageToWorld(image1.Point2D(point2D_idx1).XY());
        const Eigen::Vector2d point2_N = camera2.ImageToWorld(image2.Point2D(point2D_idx2).XY());
        const Eigen::Vector3d& xyz = TriangulatePoint(proj_matrix1, proj_matrix2, point1_N, point2_N);
        const double tri_angle = CalculateTriangulationAngle(proj_center1, proj_center2, xyz);
        if (tri_angle >= min_tri_angle_rad &&
            HasPointPositiveDepth(proj_matrix1, xyz) &&
            HasPointPositiveDepth(proj_matrix2, xyz)) {
            // Seems like everything is so great that we now have xyz of the 3d point!
            // And track with two correspondent images and 2d points.
            track.Element(0).point2D_idx = point2D_idx1;
            track.Element(1).point2D_idx = point2D_idx2;
            reconstruction_->AddPoint3D(xyz, track);
        }
    }

    // So we've registered images, created a track with 2 images and, which is, I guess, super important -
    // added a real 3d point into our reconstruction!
    return true;
}

bool IncrementalMapper::RegisterNextImage(const Options& options, const image_t image_id) {
    options.Check();

    // Fixing an image and a camera connected to this image.
    Image& image = reconstruction_->Image(image_id);
    Camera& camera = reconstruction_->Camera(image.CameraId());

    num_reg_trials_[image_id] += 1;

    if (image.NumVisiblePoints3D() < static_cast<size_t>(options.abs_pose_min_num_inliers)) {
        return false;
    }

    const int kCorrTransitivity = 1;

    std::vector<std::pair<point2D_t, point3D_t>> tri_corrs;
    std::vector<Eigen::Vector2d> tri_points2D;
    std::vector<Eigen::Vector3d> tri_points3D;

    for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D(); ++point2D_idx) {
        // Running the following code for every 2d point of the image we're trying to append.
        const Point2D& point2D = image.Point2D(point2D_idx);
        const SceneGraph& scene_graph = database_cache_->SceneGraph();
        // Getting Correspondences for our image and selected 2d point from scene graph.
        // TODO(uladbohdan): to understand what exactly is meant by Scene Graph and Correspondeces,
        // I guess it tightly connected with 'traces'.
        const std::vector<SceneGraph::Correspondence> corrs =
                scene_graph.FindTransitiveCorrespondences(image_id, point2D_idx, kCorrTransitivity);

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

            const Camera& corr_camera = reconstruction_->Camera(corr_image.CameraId());

            if (corr_camera.HasBogusParams(options.min_focal_length_ratio,
                                           options.max_focal_length_ratio,
                                           options.max_extra_param)) {
                continue;
            }

            // So here is the 3d point we were looking for!
            const Point_3D& point3D = reconstruction_->Point3D(corr_point2D.Point3DId());

            tri_corrs.emplace_back(point2D_idx, corr_point2D.Point3DId());
            point3D_ids.insert(corr_point2D.Point3DId());
            tri_points2D.push_back(point2D.XY());
            tri_points3D.push_back(point3D.XYZ());
        }
    }

    if (tri_points2D.size() < static_cast<size_t>(options.abs_pose_min_num_inliers)) {
        return false;
    }

    AbsolutePoseEstimationOptions abs_pose_options;
    abs_pose_options.num_threads = options.num_threads;
    abs_pose_options.num_focal_length_samples = 30;
    abs_pose_options.min_focal_length_ratio = options.min_focal_length_ratio;
    abs_pose_options.max_focal_length_ratio = options.max_focal_length_ratio;
    abs_pose_options.ransac_options.max_error = options.abs_pose_max_error;
    abs_pose_options.ransac_options.min_inlier_ratio = options.abs_pose_min_inlier_ratio;
    abs_pose_options.ransac_options.confidence = 0.9999;

    abs_pose_options.use_qvec_tvec_estimations = options.use_qvec_tvec_estimations;

    AbsolutePoseRefinementOptions abs_pose_refinement_options;
    if (refined_cameras_.count(image.CameraId()) > 0) {
        // If we already have our current camera in refined_cameras_.
        if (camera.HasBogusParams(options.min_focal_length_ratio,
                                  options.max_focal_length_ratio,
                                  options.max_extra_param)) {
            // Camera in refined params but params are still bogus. So camera will participate in BA.
            refined_cameras_.erase(image.CameraId());
            // Retrieving camera params from database cache.
            camera.SetParams(database_cache_->Camera(image.CameraId()).Params());
            // Unless camera has prior focal length we estimate it, so setting flag on.
            abs_pose_options.estimate_focal_length = !camera.HasPriorFocalLength();
            abs_pose_refinement_options.refine_focal_length = true;
        } else {
            // I guess that means our camera has real params, not bogus.
            // So no need in estimating and refining focal length.
            abs_pose_options.estimate_focal_length = false;
            abs_pose_refinement_options.refine_focal_length = false;
        }
    } else {
        // Our camera is not yet in refined_cameras_.
        // So camera will be refined and estimated unless it has prior focal length.
        abs_pose_options.estimate_focal_length = !camera.HasPriorFocalLength();
        abs_pose_refinement_options.refine_focal_length = true;
    }

    if (!options.abs_pose_estimate_focal_length) {
        // Overrides everything we got a step before.
        abs_pose_options.estimate_focal_length = false;
        abs_pose_refinement_options.refine_focal_length = false;
    }

    abs_pose_refinement_options.use_qvec_tvec_estimations = options.use_qvec_tvec_estimations;

    // That's weird. If my flags are set then that doesn't make much sense.
    size_t num_inliers;
    std::vector<bool> inlier_mask;

    /* if (options.use_drone_data && options.use_qvec_tvec_estimations) {
         // Keeping estimations for Qvec and Tvec as they were in a drone_data file.
         std::cout << "\e[31mSKIPPING ESTIMATION... Keeping following values:" << std::endl;
         std::cout << "IMAGE " << image_id << "  | " <<
                   *&image.Qvec()(0) << " " << *&image.Qvec()(1) << " " <<
                   *&image.Qvec()(2) << " " << *&image.Qvec()(3) << " | " <<
                   *&image.Tvec()(0) << " " << *&image.Tvec()(1) << " " << *&image.Tvec()(2);
         std::cout << "\e[0m\n";
     } else {*/

    // TODO(uladbohdan): to remove DEBUG output.
    std::cout << "\e[31mESTIMATION.\e[0m" << std::endl;
    std::cout << "BEFORE IMAGE " << image_id << "  Q and T: " <<
              *&image.Qvec()(0) << " " << *&image.Qvec()(1) << " " <<
              *&image.Qvec()(2) << " " << *&image.Qvec()(3) << " | " <<
              *&image.Tvec()(0) << " " << *&image.Tvec()(1) << " " << *&image.Tvec()(2) << std::endl;
    // ok so I was searching for usages of qvec and tvec and it seems to be first (only?)
    // place where we're using it (and some lines below). But that's fine.
    // I was wrong. We're not really using it hear: inside the EstimateAbsolutePose() we're ignoring the values
    // and initializing with other data.
    if (!EstimateAbsolutePose(abs_pose_options, tri_points2D, tri_points3D,
                              &image.Qvec(), &image.Tvec(), &camera, &num_inliers,
                              &inlier_mask)) {
        return false;
    }
    // TODO(uladbohdan): to remove DEBUG output.
    std::cout << "AFTER  IMAGE " << image_id << "  Q and T: " <<
              *&image.Qvec()(0) << " " << *&image.Qvec()(1) << " " <<
              *&image.Qvec()(2) << " " << *&image.Qvec()(3) << " | " <<
              *&image.Tvec()(0) << " " << *&image.Tvec()(1) << " " << *&image.Tvec()(2) << std::endl;
    std::cout << "\e[31mEND OF ESTIMATION.\e[0m" << std::endl;

    if (num_inliers < static_cast<size_t>(options.abs_pose_min_num_inliers)) {
        return false;
    }

    std::cout << "\e[31mREFINEMENT.\e[0m" << std::endl;
    std::cout << "BEFORE IMAGE " << image_id << "  Q and T: " <<
              *&image.Qvec()(0) << " " << *&image.Qvec()(1) << " " << *&image.Qvec()(2) << " " << *&image.Qvec()(3)
              << " | " <<
              *&image.Tvec()(0) << " " << *&image.Tvec()(1) << " " << *&image.Tvec()(2) << std::endl;
    // But here we're finally not ignoring the qvec and tvec and adding them into a problem.
    if (!RefineAbsolutePose(abs_pose_refinement_options, inlier_mask,
                            tri_points2D, tri_points3D, &image.Qvec(),
                            &image.Tvec(), &camera)) {
        return false;
    }
    std::cout << "AFTER  IMAGE " << image_id << "  Q and T: " <<
              *&image.Qvec()(0) << " " << *&image.Qvec()(1) << " " << *&image.Qvec()(2) << " " << *&image.Qvec()(3)
              << " | " <<
              *&image.Tvec()(0) << " " << *&image.Tvec()(1) << " " << *&image.Tvec()(2) << std::endl;
    std::cout << "\e[31mEND OF REFINEMENT.\e[0m" << std::endl;



    // Looks like after all, if image passes all the obstructions it met - it can be added into reconstruction!
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

    // That's the only place where we're adding cameras into refined_cameras_
    refined_cameras_.insert(image.CameraId());

    return true;
}

size_t IncrementalMapper::TriangulateImage(const IncrementalTriangulator::Options& tri_options,
                                           const image_t image_id) {
    return triangulator_->TriangulateImage(tri_options, image_id);
}

size_t IncrementalMapper::Retriangulate(const IncrementalTriangulator::Options& tri_options) {
    return triangulator_->Retriangulate(tri_options);
}

size_t IncrementalMapper::CompleteTracks(const IncrementalTriangulator::Options& tri_options) {
    return triangulator_->CompleteAllTracks(tri_options);
}

size_t IncrementalMapper::MergeTracks(const IncrementalTriangulator::Options& tri_options) {
    return triangulator_->MergeAllTracks(tri_options);
}

IncrementalMapper::LocalBundleAdjustmentReport
IncrementalMapper::AdjustLocalBundle(const Options& options,
                                     const BundleAdjuster::Options& ba_options,
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
            if (options.use_drone_data && options.use_qvec_tvec_estimations) {
                for (image_t image_id : local_bundle) {
                    ba_config.SetConstantPose(image_id);
                }
            } else {
                ba_config.SetConstantPose(local_bundle[local_bundle.size() - 1]);
                ba_config.SetConstantTvec(local_bundle[local_bundle.size() - 2], {0});
            }
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

bool IncrementalMapper::AdjustGlobalBundle(const BundleAdjuster::Options& ba_options) {

    const std::vector<image_t>& reg_image_ids = reconstruction_->RegImageIds();

    // Here we're removing some of the observations. What is a "negative depth"?
    reconstruction_->FilterObservationsWithNegativeDepth();

    // Configuring Bundle Adjustment.
    BundleAdjustmentConfiguration ba_config;
    for (const image_t image_id : reg_image_ids) {
        ba_config.AddImage(image_id);

        if (ba_options.use_drone_data) {
            camera_t camera_id = reconstruction_->Image(image_id).CameraId();
            ba_config.SetConstantCamera(camera_id);
        }
    }

    if (ba_options.use_drone_data && ba_options.use_qvec_tvec_estimations) {
        for (image_t image_id : reconstruction_->RegImageIds()) {
            ba_config.SetConstantPose(image_id);
        }
    } else {
        ba_config.SetConstantPose(reg_image_ids[0]);
        ba_config.SetConstantTvec(reg_image_ids[1], {0});
    }


    // This guy will finally run bundle adjustment with all the options and config parameters we're passing.
    BundleAdjuster bundle_adjuster(ba_options, ba_config);
    // Inside the Solve() we're adding all the data about our current reconstruction state as blocks to the ceres
    // problem, than choosing a type of a solver and running it.
    if (!bundle_adjuster.Solve(reconstruction_)) {
        return false;
    }

    // Lots of projection magic. Does something.
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
    return reconstruction_->FilterAllPoints3D(options.filter_max_reproj_error, options.filter_min_tri_angle);
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

        if (num_registrations_.count(image.first) > 0 && num_registrations_.at(image.first) > 0) {
            continue;
        }

        const class Camera& camera = reconstruction_->Camera(image.second.CameraId());
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
                } else if (!image_info1.prior_focal_length && image_info2.prior_focal_length) {
                    return false;
                } else {
                    return image_info1.num_correspondences > image_info2.num_correspondences;
                }
            });

    std::vector<image_t> image_ids;
    image_ids.reserve(image_infos.size());
    for (const ImageInfo& image_info : image_infos) {
        image_ids.push_back(image_info.image_id);
    }

    return image_ids;
}

std::vector<image_t> IncrementalMapper::FindSecondInitialImage(const image_t image_id1) const {
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

std::vector<image_t> IncrementalMapper::FindLocalBundle(const Options& options, const image_t image_id) const {
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

bool IncrementalMapper::EstimateInitialTwoViewGeometry(const Options& options,
                                                       const image_t image_id1,
                                                       const image_t image_id2) {
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
