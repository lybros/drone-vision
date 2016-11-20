#ifndef INC_3D_RECONSTRUCTION_FEATURES_H
#define INC_3D_RECONSTRUCTION_FEATURES_H

#include "storage.h"
#include "entities/point_3d.h"
#include "entities/point_2d.h"
#include "../3rdparty/sift_gpu/SiftGPU.h"

#include <iostream>
#include <QtCore/QThread>
#include <QtCore/QMutex>
#include <QtGui/QOpenGLContext>
#include <QtGui/QOffscreenSurface>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

struct SIFTOptions {
    int max_image_size = 3200;

    int max_num_features = 8192;

    int first_octave = -1;

    int num_octaves = 4;

    int octave_resolution = 3;

    double peak_threshold = 0.02 / octave_resolution;

    double edge_threshold = 10.0;

    int max_num_orientations = 2;

    enum class Normalization {
        L1_ROOT,
        L2,
    };
    Normalization normalization = Normalization::L1_ROOT;

    void Check() const;
};

class FeatureExtractor : public QThread {
public:
    struct Options {
        std::string camera_model = "RADIAL";

        bool single_camera = false;

        std::string camera_params = "";

        double default_focal_length_factor = 1.2;

        void Check() const;
    };

    FeatureExtractor(const Options& options, const std::string& database_path,
                     const std::string& image_path);

    void run();

    void Stop();

protected:
    virtual void DoExtraction() = 0;

    bool ReadImage(const std::string& image_path, Image* image, Bitmap* bitmap);

    bool stop_;
    QMutex mutex_;

    Options options_;

    Database database_;

    std::string database_path_;

    std::string image_path_;

    Camera last_camera_;

    camera_t last_camera_id_;
};


class SiftGPUFeatureExtractor : public FeatureExtractor {
public:
    SiftGPUFeatureExtractor(const Options& options,
                            const SIFTOptions& sift_options,
                            const std::string& database_path,
                            const std::string& image_path);

    ~SiftGPUFeatureExtractor();

private:
    void TearDown();

    void DoExtraction() override;

    SIFTOptions sift_options_;

    QThread* parent_thread_;
    QOpenGLContext* context_;
    QOffscreenSurface* surface_;
};


class FeatureMatcher : public QThread {
public:
    struct Options {
        int num_threads = ThreadPool::kMaxNumThreads;

        int gpu_index = -1;

        double max_ratio = 0.8;

        double max_distance = 0.7;

        bool cross_check = true;

        int max_num_matches = 8192;

        double max_error = 4.0;

        double confidence = 0.999;

        int max_num_trials = 10000;

        double min_inlier_ratio = 0.25;

        int min_num_inliers = 15;

        bool multiple_models = false;

        bool guided_matching = false;

        void Check() const;
    };

    FeatureMatcher(const Options& options, const std::string& database_path);

    ~FeatureMatcher();

    void run();

    virtual void Stop();

protected:
    struct GeometricVerificationData {
        const Camera* camera1;
        const Camera* camera2;
        const FeatureKeypoints* keypoints1;
        const FeatureKeypoints* keypoints2;
        const FeatureMatches* matches;
        TwoViewGeometry::Options* options;
    };

    virtual void DoMatching() = 0;

    void SetupWorkers();

    void SetupData();

    bool IsStopped();

    void PrintElapsedTime(const Timer& timer);

    const FeatureKeypoints& CacheKeypoints(const image_t image_id);

    const FeatureDescriptors& CacheDescriptors(const image_t image_id);

    void CleanCache(const std::unordered_set<image_t>& keep_image_ids);

    void UploadKeypoints(const int index, const image_t image_id);

    void UploadDescriptors(const int index, const image_t image_id);

    void ExtractMatchesFromBuffer(const size_t num_matches, FeatureMatches* matches) const;

    void MatchImagePairs(const std::vector<std::pair<image_t, image_t>>& image_pairs);

    void MatchImagePairGuided(const image_t image_id1, const image_t image_id2, TwoViewGeometry* two_view_geometry);

    static TwoViewGeometry VerifyImagePair(const GeometricVerificationData data, const bool multiple_models);

    Timer total_timer_;

    bool stop_;
    QMutex stop_mutex_;

    Options options_;
    Database database_;
    std::string database_path_;

    QThread* parent_thread_;
    QOpenGLContext* context_;
    QOffscreenSurface* surface_;

    SiftGPU* sift_gpu_;
    SiftMatchGPU* sift_match_gpu_;
    ThreadPool* verifier_thread_pool_;

    std::unordered_map<camera_t, Camera> cameras_;
    std::unordered_map<image_t, Image> images_;

    std::unordered_map<image_t, FeatureKeypoints> keypoints_cache_;
    std::unordered_map<image_t, FeatureDescriptors> descriptors_cache_;

    std::array<image_t, 2> prev_uploaded_image_ids_;

    std::vector<int> matches_buffer_;
};

class ExhaustiveFeatureMatcher : public FeatureMatcher {
public:
    struct ExhaustiveOptions {
        int block_size = 35;

        bool preemptive = false;

        int preemptive_num_features = 100;

        int preemptive_min_num_matches = 4;

        void Check() const;
    };

    ExhaustiveFeatureMatcher(const Options& options,
                             const ExhaustiveOptions& exhaustive_options,
                             const std::string& database_path);

private:
    virtual void DoMatching();

    std::vector<std::pair<image_t, image_t>> PreemptivelyFilterImagePairs(
            const std::vector<std::pair<image_t, image_t>>& image_pairs
    );

    ExhaustiveOptions exhaustive_options_;
};


#endif //INC_3D_RECONSTRUCTION_FEATURES_H
