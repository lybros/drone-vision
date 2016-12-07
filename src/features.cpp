#include "features.h"

namespace {
    void ScaleBitmap(
            const Camera& camera,
            const int max_image_size,
            double* scale_x,
            double* scale_y,
            Bitmap* bitmap
    ) {
        if (static_cast<int>(camera.Width()) > max_image_size ||
            static_cast<int>(camera.Height()) > max_image_size) {
            const double scale = static_cast<double>(max_image_size) / std::max(camera.Width(), camera.Height());
            const int new_width = static_cast<int>(camera.Width() * scale);
            const int new_height = static_cast<int>(camera.Height() * scale);

            std::cout << QString().sprintf(
                    "  WARNING: Image exceeds maximum dimensions - resizing to %dx%d.", new_width, new_height
            ).toStdString() << std::endl;

            *scale_x = static_cast<double>(new_width) / camera.Width();
            *scale_y = static_cast<double>(new_height) / camera.Height();

            *bitmap = bitmap->Rescale(new_width, new_height);
        } else {
            *scale_x = 1.0;
            *scale_y = 1.0;
        }
    }

}

void SIFTOptions::Check() const {
}

void FeatureExtractor::Options::Check() const {
}

FeatureExtractor::FeatureExtractor(
        const Options& options,
        const std::string& database_path,
        const std::string& image_path
) : stop_(false), options_(options), database_path_(database_path), image_path_(image_path),
    parent_thread_(QThread::currentThread()) {

    options_.Check();
    image_path_ = StringReplace(image_path_, "\\", "/");
    image_path_ = EnsureTrailingSlash(image_path_);
}

void FeatureExtractor::run() {
    last_camera_.SetModelIdFromName(options_.camera_model);
    last_camera_id_ = kInvalidCameraId;

    if (!options_.camera_params.empty() && !last_camera_.SetParamsFromString(options_.camera_params)) {
        std::cerr << "  ERROR: Invalid camera parameters." << std::endl;
        return;
    }

    Timer total_timer;
    total_timer.Start();

    database_.Open(database_path_);
    DoExtraction();
    database_.Close();

    total_timer.PrintMinutes();
}

void FeatureExtractor::Stop() {
    QMutexLocker locker(&mutex_);
    stop_ = true;
}

bool FeatureExtractor::ReadImage(const std::string& image_path, Image* image, Bitmap* bitmap) {
    image->SetName(image_path);
    image->SetName(StringReplace(image->Name(), "\\", "/"));
    image->SetName(StringReplace(image->Name(), image_path_, ""));

    std::cout << "  Name:           " << image->Name() << std::endl;

    const bool exists_image = database_.ExistsImageName(image->Name());

    if (exists_image) {
        database_.BeginTransaction();
        *image = database_.ReadImageFromName(image->Name());
        const bool exists_keypoints = database_.ExistsKeypoints(image->ImageId());
        const bool exists_descriptors = database_.ExistsDescriptors(image->ImageId());
        database_.EndTransaction();

        if (exists_keypoints && exists_descriptors) {
            std::cout << "  SKIP: Image already processed." << std::endl;
            return false;
        }
    }

    if (!bitmap->Read(image_path, false)) {
        std::cout << "  SKIP: Cannot read image at path " << image_path << std::endl;
        return false;
    }

    if (exists_image) {
        const Camera camera = database_.ReadCamera(image->CameraId());

        if (options_.single_camera && last_camera_id_ != kInvalidCameraId &&
            (camera.Width() != last_camera_.Width() || camera.Height() != last_camera_.Height())) {
            std::cerr << "  ERROR: Single camera specified, but images have different dimensions." << std::endl;
            return false;
        }

        if (static_cast<size_t>(bitmap->Width()) != camera.Width() ||
            static_cast<size_t>(bitmap->Height()) != camera.Height()) {
            std::cerr << "  ERROR: Image previously processed, but current version has different dimensions."
            << std::endl;
        }
    }

    if (options_.single_camera && last_camera_id_ != kInvalidCameraId &&
        (last_camera_.Width() != static_cast<size_t>(bitmap->Width()) ||
         last_camera_.Height() != static_cast<size_t>(bitmap->Height()))) {
        std::cerr << "  ERROR: Single camera specified, but images have different dimensions" << std::endl;
        return false;
    }

    last_camera_.SetWidth(static_cast<size_t>(bitmap->Width()));
    last_camera_.SetHeight(static_cast<size_t>(bitmap->Height()));

    std::cout << "  Width:          " << last_camera_.Width() << "px" << std::endl;
    std::cout << "  Height:         " << last_camera_.Height() << "px" << std::endl;

    if (!options_.single_camera || last_camera_id_ == kInvalidCameraId) {
        if (options_.camera_params.empty()) {
            double focal_length = 0.0;
            if (bitmap->ExifFocalLength(&focal_length)) {
                last_camera_.SetPriorFocalLength(true);
                std::cout << "  Focal length:   " << focal_length << "px (EXIF)" << std::endl;
            } else {
                focal_length = options_.default_focal_length_factor *
                               std::max(bitmap->Width(), bitmap->Height());
                last_camera_.SetPriorFocalLength(false);
                std::cout << "  Focal length:   " << focal_length << "px" << std::endl;
            }

            // Here we're finally initializing camera with focal_length if we have it.
            // Yes, not only focal length, but focal length is what I'm interested in at the moment.
            last_camera_.InitializeWithId(last_camera_.ModelId(),
                                          focal_length,
                                          last_camera_.Width(),
                                          last_camera_.Height());
        }

        if (!last_camera_.VerifyParams()) {
            std::cerr << "  ERROR: Invalid camera parameters." << std::endl;
            return false;
        }

        last_camera_id_ = database_.WriteCamera(last_camera_);
    }

    image->SetCameraId(last_camera_id_);

    std::cout << "  Camera ID:      " << last_camera_id_ << std::endl;
    std::cout << "  Camera Model:   " << last_camera_.ModelName() << std::endl;

    if (bitmap->ExifLatitude(&image->TvecPrior(0)) &&
        bitmap->ExifLongitude(&image->TvecPrior(1)) &&
        bitmap->ExifAltitude(&image->TvecPrior(2))) {
        std::cout << QString().sprintf(
                "  EXIF GPS:       LAT=%.3f, LON=%.3f, ALT=%.3f",
                image->TvecPrior(0), image->TvecPrior(1), image->TvecPrior(2)
        ).toStdString() << std::endl;
    } else {
        image->TvecPrior(0) = std::numeric_limits<double>::quiet_NaN();
        image->TvecPrior(1) = std::numeric_limits<double>::quiet_NaN();
        image->TvecPrior(2) = std::numeric_limits<double>::quiet_NaN();
    }

    return true;
}

void FeatureExtractor::TearDown() {
    context_->doneCurrent();
    context_->moveToThread(parent_thread_);
}

SiftGPUFeatureExtractor::SiftGPUFeatureExtractor(
        const Options& options,
        const SIFTOptions& sift_options,
        const std::string& database_path,
        const std::string& image_path) : FeatureExtractor(options, database_path, image_path),
                                         sift_options_(sift_options) {
    sift_options_.Check();
    surface_ = new QOffscreenSurface();
    surface_->create();
    context_ = new QOpenGLContext();
    context_->create();
    context_->makeCurrent(surface_);
    context_->doneCurrent();
    context_->moveToThread(this);
}

SiftGPUFeatureExtractor::~SiftGPUFeatureExtractor() {
    delete context_;
    surface_->deleteLater();
}

void SiftGPUFeatureExtractor::DoExtraction() {
    PrintHeading1("Feature extraction using SIFT (GPU)");

    context_->makeCurrent(surface_);

    const size_t num_files = std::distance(
            boost::filesystem::recursive_directory_iterator(image_path_),
            boost::filesystem::recursive_directory_iterator()
    );

    const std::string max_image_size_str = std::to_string(sift_options_.max_image_size);
    const std::string max_num_features_str = std::to_string(sift_options_.max_num_features);
    const std::string first_octave_str = std::to_string(sift_options_.first_octave);
    const std::string num_octaves_str = std::to_string(sift_options_.num_octaves);
    const std::string octave_resolution_str = std::to_string(sift_options_.octave_resolution);
    const std::string peak_threshold_str = std::to_string(sift_options_.peak_threshold);
    const std::string edge_threshold_str = std::to_string(sift_options_.edge_threshold);
    const std::string max_num_orientations_str = std::to_string(sift_options_.max_num_orientations);

    const int kNumArgs = 19;
    const char* sift_gpu_args[kNumArgs] = {
            "-da",
            "-v", "0",
            "-maxd", max_image_size_str.c_str(),
            "-tc2", max_num_features_str.c_str(),
            "-fo", first_octave_str.c_str(),
            "-no", num_octaves_str.c_str(),
            "-d", octave_resolution_str.c_str(),
            "-t", peak_threshold_str.c_str(),
            "-e", edge_threshold_str.c_str(),
            "-mo", max_num_orientations_str.c_str(),
    };

    SiftGPU sift_gpu;
    sift_gpu.ParseParam(kNumArgs, sift_gpu_args);

    if (sift_gpu.VerifyContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        std::cerr << "ERROR: SiftGPU not fully supported." << std::endl;
        TearDown();
        return;
    }

    const int max_image_size = sift_gpu.GetMaxDimension();

    size_t i_file = 0;
    auto dir_iter = boost::filesystem::recursive_directory_iterator(image_path_);
    for (auto it = dir_iter; it != boost::filesystem::recursive_directory_iterator(); ++it) {
        const boost::filesystem::path& image_path = *it;
        i_file += 1;

        if (!boost::filesystem::is_regular_file(image_path)) {
            continue;
        }

        {
            QMutexLocker locker(&mutex_);
            if (stop_) {
                TearDown();
                return;
            }
        }

        std::cout << "Processing file [" << i_file << "/" << num_files << "]" << std::endl;

        Image image;
        Bitmap bitmap;
        if (!ReadImage(image_path.string(), &image, &bitmap)) {
            continue;
        }

        double scale_x;
        double scale_y;
        ScaleBitmap(last_camera_, max_image_size, &scale_x, &scale_y, &bitmap);

        const std::vector<uint8_t> bitmap_raw_bits = bitmap.ConvertToRawBits();
        const int code = sift_gpu.RunSIFT(
                bitmap.ScanWidth(),
                bitmap.Height(),
                bitmap_raw_bits.data(),
                GL_LUMINANCE,
                GL_UNSIGNED_BYTE
        );

        const int kSuccessCode = 1;
        if (code == kSuccessCode) {
            database_.BeginTransaction();

            if (image.ImageId() == kInvalidImageId) {
                image.SetImageId(database_.WriteImage(image));
            }

            const size_t num_features = static_cast<size_t>(sift_gpu.GetFeatureNum());
            std::vector<SiftGPU::SiftKeypoint> sift_gpu_keypoints(num_features);

            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> descriptors(num_features, 128);

            sift_gpu.GetFeatureVector(sift_gpu_keypoints.data(), descriptors.data());

            if (!database_.ExistsKeypoints(image.ImageId())) {
                if (scale_x != 1.0 || scale_y != 1.0) {
                    const float inv_scale_x = static_cast<float>(1.0 / scale_x);
                    const float inv_scale_y = static_cast<float>(1.0 / scale_y);
                    const float inv_scale_xy = (inv_scale_x + inv_scale_y) / 2.0f;
                    for (size_t i = 0; i < sift_gpu_keypoints.size(); ++i) {
                        sift_gpu_keypoints[i].x *= inv_scale_x;
                        sift_gpu_keypoints[i].y *= inv_scale_y;
                        sift_gpu_keypoints[i].s *= inv_scale_xy;
                    }
                }

                FeatureKeypoints keypoints(num_features);
                for (size_t i = 0; i < sift_gpu_keypoints.size(); ++i) {
                    keypoints[i].x = sift_gpu_keypoints[i].x;
                    keypoints[i].y = sift_gpu_keypoints[i].y;
                    keypoints[i].scale = sift_gpu_keypoints[i].s;
                    keypoints[i].orientation = sift_gpu_keypoints[i].o;
                }

                database_.WriteKeypoints(image.ImageId(), keypoints);
            }

            if (!database_.ExistsDescriptors(image.ImageId())) {
                if (sift_options_.normalization == SIFTOptions::Normalization::L2) {
                    descriptors = L2NormalizeFeatureDescriptors(descriptors);
                } else if (sift_options_.normalization == SIFTOptions::Normalization::L1_ROOT) {
                    descriptors = L1RootNormalizeFeatureDescriptors(descriptors);
                }
                const FeatureDescriptors descriptors_byte = FeatureDescriptorsToUnsignedByte(descriptors);
                database_.WriteDescriptors(image.ImageId(), descriptors_byte);
            }

            std::cout << "  Features:       " << num_features << std::endl;

            database_.EndTransaction();
        } else { std::cerr << "  ERROR: Could not extract features." << std::endl; }
    }

    TearDown();
}

OpenCVFeatureExtractor::OpenCVFeatureExtractor(
        const std::string& detector_type,
        const std::string& extractor_type,
        const Options& options,
        const SIFTOptions& sift_options,
        const std::string& database_path,
        const std::string& image_path) : FeatureExtractor(options, database_path, image_path),
                                         sift_options_(sift_options),
                                         detector_type_(detector_type),
                                         extractor_type_(extractor_type) {
    sift_options_.Check();
    surface_ = new QOffscreenSurface();
    surface_->create();
    context_ = new QOpenGLContext();
    context_->create();
    context_->makeCurrent(surface_);
    context_->doneCurrent();
    context_->moveToThread(this);
}

OpenCVFeatureExtractor::~OpenCVFeatureExtractor() {
    delete context_;
    surface_->deleteLater();
}

void OpenCVFeatureExtractor::DoExtraction() {
    PrintHeading1(QString().sprintf("Feature extraction using OpenCV %s", detector_type_.c_str()).toStdString());

    context_->makeCurrent(surface_);

    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;

    if (detector_type_ == "SIFT") {
        detector = cv::xfeatures2d::SiftFeatureDetector::create(
                sift_options_.max_num_features,
                sift_options_.num_octaves,
                sift_options_.peak_threshold,
                sift_options_.edge_threshold,
                sift_options_.sigma
        );
    }

    if (extractor_type_ == "SIFT") {
        extractor = cv::xfeatures2d::SiftDescriptorExtractor::create(
                sift_options_.max_num_features,
                sift_options_.num_octaves,
                sift_options_.peak_threshold,
                sift_options_.edge_threshold,
                sift_options_.sigma
        );
    }

    QDir dir = QDir(QString::fromStdString(image_path_));
    dir.setFilter(QDir::Files);
    QDirIterator it(dir, QDirIterator::Subdirectories);
    dir.setFilter(QDir::Files | QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);

    const size_t num_files = dir.count();
    size_t i_file = 0;

    while (it.hasNext()) {
        i_file += 1;
        std::cout << "Processing file [" << i_file << "/" << num_files << "]" << std::endl;
        std::string image_path = it.next().toStdString();

        {
            QMutexLocker locker(&mutex_);
            if (stop_) {
                TearDown();
                return;
            }
        }

        Image image;
        Bitmap bitmap;
        if (!ReadImage(image_path, &image, &bitmap)) {
            continue;
        }

        double scale_x;
        double scale_y;
        ScaleBitmap(last_camera_, sift_options_.max_image_size, &scale_x, &scale_y, &bitmap);

        std::vector<cv::KeyPoint> cv_keypoints;
        cv::Mat cv_descriptors;
        cv::Mat image_gray = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

        detector->detect(image_gray, cv_keypoints);
        extractor->compute(image_gray, cv_keypoints, cv_descriptors);

        database_.BeginTransaction();

        if (image.ImageId() == kInvalidImageId) {
            image.SetImageId(database_.WriteImage(image));
        }

        const size_t num_features = cv_keypoints.size();

        if (!database_.ExistsKeypoints(image.ImageId())) {
            if (scale_x != 1.0 || scale_y != 1.0) {
                const float inv_scale_x = static_cast<float>(1.0 / scale_x);
                const float inv_scale_y = static_cast<float>(1.0 / scale_y);
                //const float inv_scale_xy = (inv_scale_x + inv_scale_y) / 2.0f;

                for (size_t i = 0; i < num_features; ++i) {
                    cv_keypoints[i].pt.x *= inv_scale_x;
                    cv_keypoints[i].pt.y *= inv_scale_y;
                    //cv_keypoints[i].pt.s *= inv_scale_xy;
                }
            }

            FeatureKeypoints keypoints(num_features);

            for (size_t i = 0; i < num_features; ++i) {
                keypoints[i].x = cv_keypoints[i].pt.x;
                keypoints[i].y = cv_keypoints[i].pt.y;
                //keypoints[i].orientation = cv_keypoints[i].angle; //(todo(drapegnik): check formats compatibility
            }

            database_.WriteKeypoints(image.ImageId(), keypoints);
        }

        FeatureDescriptors descriptors(num_features, 128);
        cv::cv2eigen(cv_descriptors, descriptors);

        if (!database_.ExistsDescriptors(image.ImageId())) {
            database_.WriteDescriptors(image.ImageId(), descriptors);
        }

        std::cout << "  Features:       " << num_features << std::endl;
        database_.EndTransaction();
        TearDown();
    }
}


namespace {
    FeatureDescriptors ExtractTopScaleDescriptors(
            const FeatureKeypoints& keypoints,
            const FeatureDescriptors& descriptors,
            const size_t num_features) {

        FeatureDescriptors top_scale_descriptors;

        if (static_cast<size_t>(descriptors.rows()) <= num_features) {
            top_scale_descriptors = descriptors;
        } else {
            std::vector<std::pair<size_t, float>> scales;
            scales.reserve(static_cast<size_t>(keypoints.size()));

            for (size_t i = 0; i < keypoints.size(); ++i) {
                scales.emplace_back(i, keypoints[i].scale);
            }

            std::partial_sort(scales.begin(), scales.begin() + num_features,
                              scales.end(), [](const std::pair<size_t, float> scale1,
                                               const std::pair<size_t, float> scale2) {
                        return scale1.second > scale2.second;
                    });

            top_scale_descriptors.resize(num_features, descriptors.cols());
            for (size_t i = 0; i < num_features; ++i) {
                top_scale_descriptors.row(i) = descriptors.row(scales[i].first);
            }
        }

        return top_scale_descriptors;
    }

}

void FeatureMatcher::Options::Check() const {
}

FeatureMatcher::FeatureMatcher(const Options& options,
                               const std::string& database_path)
        : stop_(false),
          options_(options),
          database_path_(database_path),
          parent_thread_(QThread::currentThread()),
          prev_uploaded_image_ids_{{kInvalidImageId, kInvalidImageId}} {
    options_.Check();

    surface_ = new QOffscreenSurface();
    surface_->create();
    context_ = new QOpenGLContext();
    context_->create();
    context_->makeCurrent(surface_);
    context_->doneCurrent();
    context_->moveToThread(this);
}

FeatureMatcher::~FeatureMatcher() {
    delete context_;
    surface_->deleteLater();
}

void FeatureMatcher::Stop() {
    QMutexLocker locker(&stop_mutex_);
    stop_ = true;
}

void FeatureMatcher::run() {
    total_timer_.Restart();

    SetupData();
    SetupWorkers();
    DoMatching();

    total_timer_.PrintMinutes();

    context_->doneCurrent();
    context_->moveToThread(parent_thread_);
    database_.Close();
    delete sift_gpu_;
    delete sift_match_gpu_;
    delete verifier_thread_pool_;
}

void FeatureMatcher::SetupWorkers() {
    context_->makeCurrent(surface_);

    sift_gpu_ = new SiftGPU();
    sift_match_gpu_ = new SiftMatchGPU(options_.max_num_matches);

    sift_gpu_->SetVerbose(0);
    sift_match_gpu_->SetLanguage(SiftMatchGPU::SIFTMATCH_GLSL);

    if (sift_match_gpu_->VerifyContextGL() == 0) {
        std::cerr << "ERROR: SiftMatchGPU not fully supported." << std::endl;
        return;
    }

    verifier_thread_pool_ = new ThreadPool(options_.num_threads);
}

void FeatureMatcher::SetupData() {
    database_.Open(database_path_);

    const std::vector<Camera> cameras = database_.ReadAllCameras();
    cameras_.clear();
    cameras_.reserve(cameras.size());
    for (const Camera& camera : cameras) {
        cameras_.emplace(camera.CameraId(), camera);
    }

    const std::vector<Image> images = database_.ReadAllImages();
    images_.clear();
    images_.reserve(images.size());
    for (const Image& image : images) {
        images_.emplace(image.ImageId(), image);
    }
}

bool FeatureMatcher::IsStopped() {
    QMutexLocker locker(&stop_mutex_);
    return stop_;
}

void FeatureMatcher::PrintElapsedTime(const Timer& timer) {
    std::cout << QString().sprintf(" in %.3fs", timer.ElapsedSeconds()).toStdString() << std::endl;
}

const FeatureKeypoints& FeatureMatcher::CacheKeypoints(const image_t image_id) {

    if (keypoints_cache_.count(image_id) == 0) {
        keypoints_cache_[image_id] = database_.ReadKeypoints(image_id);
    }
    return keypoints_cache_.at(image_id);
}

const FeatureDescriptors& FeatureMatcher::CacheDescriptors(const image_t image_id) {

    if (descriptors_cache_.count(image_id) == 0) {
        descriptors_cache_[image_id] = database_.ReadDescriptors(image_id);
    }
    return descriptors_cache_.at(image_id);
}

void FeatureMatcher::CleanCache(const std::unordered_set<image_t>& keep_image_ids) {
    for (auto it = keypoints_cache_.begin(); it != keypoints_cache_.end();) {
        if (keep_image_ids.count(it->first) == 0) {
            it = keypoints_cache_.erase(it);
        } else { ++it; }
    }

    for (auto it = descriptors_cache_.begin(); it != descriptors_cache_.end();) {
        if (keep_image_ids.count(it->first) == 0) {
            it = descriptors_cache_.erase(it);
        } else { ++it; }
    }
}

void FeatureMatcher::UploadKeypoints(const int index, const image_t image_id) {
    const FeatureKeypoints& keypoints = keypoints_cache_.at(image_id);
    sift_match_gpu_->SetFeautreLocation(index, reinterpret_cast<const float*>(keypoints.data()), 2);
}

void FeatureMatcher::UploadDescriptors(const int index, const image_t image_id) {
    if (prev_uploaded_image_ids_[index] != image_id) {
        const FeatureDescriptors& descriptors = descriptors_cache_.at(image_id);
        sift_match_gpu_->SetDescriptors(index, descriptors.rows(), descriptors.data());
        prev_uploaded_image_ids_[index] = image_id;
    }
}

void FeatureMatcher::ExtractMatchesFromBuffer(const size_t num_matches, FeatureMatches* matches) const {
    matches->resize(num_matches);
    for (size_t i = 0; i < num_matches; ++i) {
        (*matches)[i].point2D_idx1 = static_cast<point2D_t>(matches_buffer_[2 * i]);
        (*matches)[i].point2D_idx2 = static_cast<point2D_t>(matches_buffer_[2 * i + 1]);
    }
}

void FeatureMatcher::MatchImagePairs(const std::vector<std::pair<image_t, image_t>>& image_pairs) {

    std::vector<std::pair<bool, bool>> exists_mask;
    exists_mask.reserve(image_pairs.size());

    std::unordered_set<image_t> image_ids;
    image_ids.reserve(image_pairs.size());

    std::unordered_set<image_pair_t> pair_ids;
    pair_ids.reserve(image_pairs.size());

    bool exists_all = true;

    database_.BeginTransaction();

    for (const auto image_pair : image_pairs) {
        if (image_pair.first == image_pair.second) {
            exists_mask.emplace_back(true, true);
            continue;
        }

        const image_pair_t pair_id = Database::ImagePairToPairId(image_pair.first, image_pair.second);

        if (pair_ids.count(pair_id) > 0) {
            exists_mask.emplace_back(true, true);
            continue;
        }

        pair_ids.insert(pair_id);

        const bool exists_matches = database_.ExistsMatches(image_pair.first, image_pair.second);
        const bool exists_inlier_matches = database_.ExistsInlierMatches(image_pair.first, image_pair.second);

        exists_all = exists_all && exists_matches && exists_inlier_matches;
        exists_mask.emplace_back(exists_matches, exists_inlier_matches);

        if (!exists_matches || !exists_inlier_matches) {
            image_ids.insert(image_pair.first);
            image_ids.insert(image_pair.second);
        }

        if (!exists_matches || (!exists_inlier_matches && options_.guided_matching)) {
            CacheDescriptors(image_pair.first);
            CacheDescriptors(image_pair.second);
        }

        if (!exists_inlier_matches) {
            CacheKeypoints(image_pair.first);
            CacheKeypoints(image_pair.second);
        }
    }

    database_.EndTransaction();

    if (exists_all) { return; }

    CleanCache(image_ids);

    const size_t min_num_inliers = static_cast<size_t>(options_.min_num_inliers);

    matches_buffer_.resize(static_cast<size_t>(2 * options_.max_num_matches));

    struct MatchResult {
        image_t image_id1;
        image_t image_id2;
        FeatureMatches matches;
        bool write;
    };

    std::vector<MatchResult> match_results;
    match_results.reserve(image_pairs.size());

    std::vector<std::future<TwoViewGeometry>> verification_results;
    verification_results.reserve(image_pairs.size());
    std::vector<std::pair<image_t, image_t>> verification_image_pairs;
    verification_image_pairs.reserve(image_pairs.size());

    std::vector<std::pair<image_t, image_t>> empty_verification_results;

    TwoViewGeometry::Options two_view_geometry_options;
    two_view_geometry_options.min_num_inliers = static_cast<size_t>(options_.min_num_inliers);
    two_view_geometry_options.ransac_options.max_error = options_.max_error;
    two_view_geometry_options.ransac_options.confidence = options_.confidence;
    two_view_geometry_options.ransac_options.max_num_trials = static_cast<size_t>(options_.max_num_trials);
    two_view_geometry_options.ransac_options.min_inlier_ratio = options_.min_inlier_ratio;

    for (size_t i = 0; i < image_pairs.size(); ++i) {
        const auto exists = exists_mask[i];

        if (exists.first && exists.second) { continue; }

        const auto image_pair = image_pairs[i];
        const image_t image_id1 = image_pair.first;
        const image_t image_id2 = image_pair.second;

        match_results.emplace_back();
        auto& match_result = match_results.back();

        match_result.image_id1 = image_id1;
        match_result.image_id2 = image_id2;

        if (exists.first) {
            match_result.matches = database_.ReadMatches(image_id1, image_id2);
            match_result.write = false;
        } else {
            UploadDescriptors(0, image_id1);
            UploadDescriptors(1, image_id2);

            const int num_matches = sift_match_gpu_->GetSiftMatch(
                    options_.max_num_matches,
                    reinterpret_cast<int (*)[2]>(matches_buffer_.data()),
                    static_cast<float>(options_.max_distance),
                    static_cast<float>(options_.max_ratio),
                    options_.cross_check
            );

            if (num_matches >= options_.min_num_inliers) {
                ExtractMatchesFromBuffer(num_matches, &match_result.matches);
            } else {
                match_result.matches = {};
            }

            match_result.write = true;
        }

        if (!exists.second) {
            if (match_result.matches.size() >= min_num_inliers) {
                GeometricVerificationData data;
                data.camera1 = &cameras_.at(images_.at(image_id1).CameraId());
                data.camera2 = &cameras_.at(images_.at(image_id2).CameraId());
                data.keypoints1 = &keypoints_cache_.at(image_id1);
                data.keypoints2 = &keypoints_cache_.at(image_id2);
                data.matches = &match_result.matches;
                data.options = &two_view_geometry_options;

                std::function<TwoViewGeometry(GeometricVerificationData, bool)>
                        verifier_func = FeatureMatcher::VerifyImagePair;

                verification_results.push_back(
                        verifier_thread_pool_->AddTask(verifier_func, data, options_.multiple_models)
                );

                verification_image_pairs.push_back(image_pair);
            } else {
                empty_verification_results.push_back(image_pair);
            }
        }
    }

    database_.BeginTransaction();

    for (const auto& result : match_results) {
        if (result.write) {
            database_.WriteMatches(result.image_id1, result.image_id2, result.matches);
        }
    }

    for (size_t i = 0; i < verification_results.size(); ++i) {
        const auto& image_pair = verification_image_pairs[i];

        auto result = verification_results[i].get();
        if (result.inlier_matches.size() >= min_num_inliers) {

            if (options_.guided_matching) {
                const image_t image_id1 = image_pair.first;
                const image_t image_id2 = image_pair.second;
                MatchImagePairGuided(image_id1, image_id2, &result);
            }
            database_.WriteInlierMatches(image_pair.first, image_pair.second, result);
        } else {
            database_.WriteInlierMatches(image_pair.first, image_pair.second, TwoViewGeometry());
        }
    }

    for (auto& result : empty_verification_results) {
        database_.WriteInlierMatches(result.first, result.second, TwoViewGeometry());
    }

    database_.EndTransaction();
}

void FeatureMatcher::MatchImagePairGuided(
        const image_t image_id1,
        const image_t image_id2,
        TwoViewGeometry* two_view_geometry) {

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> F;
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> H;
    float* F_ptr = nullptr;
    float* H_ptr = nullptr;
    if (two_view_geometry->config == TwoViewGeometry::CALIBRATED ||
        two_view_geometry->config == TwoViewGeometry::UNCALIBRATED) {

        F = two_view_geometry->F.cast<float>();
        F_ptr = F.data();

    } else if (two_view_geometry->config == TwoViewGeometry::PLANAR ||
               two_view_geometry->config == TwoViewGeometry::PANORAMIC ||
               two_view_geometry->config == TwoViewGeometry::PLANAR_OR_PANORAMIC) {

        H = two_view_geometry->H.cast<float>();
        H_ptr = H.data();

    }

    if (F_ptr == nullptr && H_ptr == nullptr) { return; }

    UploadDescriptors(0, image_id1);
    UploadDescriptors(1, image_id2);
    UploadKeypoints(0, image_id1);
    UploadKeypoints(1, image_id2);

    matches_buffer_.resize(static_cast<size_t>(2 * options_.max_num_matches));

    const int num_matches = sift_match_gpu_->GetGuidedSiftMatch(
            options_.max_num_matches,
            reinterpret_cast<int (*)[2]>(matches_buffer_.data()), H_ptr, F_ptr,
            static_cast<float>(options_.max_distance),
            static_cast<float>(options_.max_ratio),
            static_cast<float>(options_.max_error * options_.max_error),
            static_cast<float>(options_.max_error * options_.max_error),
            options_.cross_check
    );

    if (num_matches <= static_cast<int>(two_view_geometry->inlier_matches.size())) { return; }

    ExtractMatchesFromBuffer(num_matches, &two_view_geometry->inlier_matches);
}

TwoViewGeometry FeatureMatcher::VerifyImagePair(const GeometricVerificationData data, const bool multiple_models) {

    TwoViewGeometry two_view_geometry;
    const auto points1 = FeatureKeypointsToPointsVector(*data.keypoints1);
    const auto points2 = FeatureKeypointsToPointsVector(*data.keypoints2);

    if (multiple_models) {
        two_view_geometry.EstimateMultiple(
                *data.camera1,
                points1,
                *data.camera2,
                points2,
                *data.matches,
                *data.options
        );
    } else {
        two_view_geometry.Estimate(*data.camera1, points1, *data.camera2, points2, *data.matches, *data.options);
    }
    return two_view_geometry;
}

void ExhaustiveFeatureMatcher::ExhaustiveOptions::Check() const { }

ExhaustiveFeatureMatcher::ExhaustiveFeatureMatcher(
        const Options& options,
        const ExhaustiveOptions& exhaustive_options,
        const std::string& database_path
) : FeatureMatcher(options, database_path),
    exhaustive_options_(exhaustive_options) {
    exhaustive_options_.Check();
}

void ExhaustiveFeatureMatcher::DoMatching() {
    PrintHeading1("Exhaustive feature matching");

    std::vector<image_t> image_ids;
    image_ids.reserve(images_.size());

    for (const auto image : images_) {
        image_ids.push_back(image.first);
    }

    const size_t block_size = static_cast<size_t>(exhaustive_options_.block_size);
    const size_t num_blocks = static_cast<size_t>(std::ceil(static_cast<double>(image_ids.size()) / block_size));

    std::vector<std::pair<image_t, image_t>> image_pairs;

    for (size_t start_idx1 = 0; start_idx1 < image_ids.size(); start_idx1 += block_size) {

        const size_t end_idx1 = std::min(image_ids.size(), start_idx1 + block_size) - 1;

        for (size_t start_idx2 = 0; start_idx2 < image_ids.size(); start_idx2 += block_size) {
            const size_t end_idx2 = std::min(image_ids.size(), start_idx2 + block_size) - 1;

            if (IsStopped()) { return; }

            Timer timer;
            timer.Start();

            std::cout << QString().sprintf(
                    "Matching block [%lu/%lu, %lu/%lu]",
                    (start_idx1 / block_size + 1), num_blocks,
                    (start_idx2 / block_size + 1), num_blocks
            ).toStdString() << std::flush;

            image_pairs.clear();

            for (size_t idx1 = start_idx1; idx1 <= end_idx1; ++idx1) {
                for (size_t idx2 = start_idx2; idx2 <= end_idx2; ++idx2) {
                    const size_t block_id1 = idx1 % block_size;
                    const size_t block_id2 = idx2 % block_size;

                    // to Avoid duplicate pairs
                    if ((idx1 > idx2 && block_id1 <= block_id2) || (idx1 < idx2 && block_id1 < block_id2)) {
                        image_pairs.emplace_back(image_ids[idx1], image_ids[idx2]);
                    }
                }
            }

            if (exhaustive_options_.preemptive) {
                image_pairs = PreemptivelyFilterImagePairs(image_pairs);
            }

            MatchImagePairs(image_pairs);
            PrintElapsedTime(timer);
        }
    }
}

std::vector<std::pair<image_t, image_t>>
ExhaustiveFeatureMatcher::PreemptivelyFilterImagePairs(const std::vector<std::pair<image_t, image_t>>& image_pairs) {

    const size_t num_features = static_cast<size_t>(exhaustive_options_.preemptive_num_features);

    std::unordered_map<image_t, FeatureDescriptors> top_descriptors;

    image_t prev_image_id1 = kInvalidImageId;
    image_t prev_image_id2 = kInvalidImageId;

    FeatureMatches matches_buffer_(static_cast<size_t>(options_.max_num_matches));

    std::vector<std::pair<image_t, image_t>> filtered_image_pairs;

    database_.BeginTransaction();

    for (const auto image_pair : image_pairs) {
        if (top_descriptors.count(image_pair.first) == 0) {

            top_descriptors.emplace(image_pair.first, ExtractTopScaleDescriptors(
                    CacheKeypoints(image_pair.first),
                    CacheDescriptors(image_pair.first),
                    num_features)
            );
        }
        if (top_descriptors.count(image_pair.second) == 0) {

            top_descriptors.emplace(image_pair.second, ExtractTopScaleDescriptors(
                    CacheKeypoints(image_pair.second),
                    CacheDescriptors(image_pair.second),
                    num_features)
            );
        }

        if (image_pair.first != prev_image_id1) {
            const auto& descriptors1 = top_descriptors[image_pair.first];
            sift_match_gpu_->SetDescriptors(0, descriptors1.rows(), descriptors1.data());
            prev_image_id1 = image_pair.first;
        }
        if (image_pair.second != prev_image_id2) {
            const auto& descriptors2 = top_descriptors[image_pair.second];
            sift_match_gpu_->SetDescriptors(1, descriptors2.rows(), descriptors2.data());
            prev_image_id2 = image_pair.second;
        }

        const int num_matches = sift_match_gpu_->GetSiftMatch(
                options_.max_num_matches,
                (int (*)[2]) matches_buffer_.data(),
                options_.max_distance,
                options_.max_ratio,
                options_.cross_check
        );

        if (num_matches >= exhaustive_options_.preemptive_min_num_matches) {
            filtered_image_pairs.push_back(image_pair);
        }
    }

    database_.EndTransaction();

    std::cout << QString().sprintf(
            " P(%lu/%lu)", filtered_image_pairs.size(), image_pairs.size()
    ).toStdString() << std::flush;

    return filtered_image_pairs;
}
