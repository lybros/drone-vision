#ifndef INC_3D_RECONSTRUCTION_STORAGE_H
#define INC_3D_RECONSTRUCTION_STORAGE_H

#include "utils.h"
#include "entities/image.h"
#include "geometry.h"
#include "entities/point_3d.h"
#include "entities/point_2d.h"
#include "../3rdparty/sqlite/sqlite3.h"

#include <unordered_map>
#include <unordered_set>
#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

extern "C" {
#undef DLL_API
#include <FreeImage.h>
#undef DLL_API
}

class Bitmap {
public:
    Bitmap();

    explicit Bitmap(FIBITMAP* data);

    bool Allocate(const int width, const int height, const bool as_rgb);

    const FIBITMAP* Data() const;

    FIBITMAP* Data();

    int Width() const;

    int Height() const;

    int Channels() const;

    unsigned int BitsPerPixel() const;

    unsigned int ScanWidth() const;

    bool IsRGB() const;

    bool IsGrey() const;

    std::vector<uint8_t> ConvertToRawBits() const;

    std::vector<uint8_t> ConvertToRowMajorArray() const;

    std::vector<uint8_t> ConvertToColMajorArray() const;

    bool GetPixel(const int x, const int y, Eigen::Vector3ub* color) const;

    bool SetPixel(const int x, const int y, const Eigen::Vector3ub& color);

    void Fill(const Eigen::Vector3ub& color);

    bool InterpolateNearestNeighbor(const double x, const double y,
                                    Eigen::Vector3ub* color) const;

    bool InterpolateBilinear(const double x, const double y,
                             Eigen::Vector3d* color) const;

    bool ExifFocalLength(double* focal_length);

    bool ExifLatitude(double* latitude);

    bool ExifLongitude(double* longitude);

    bool ExifAltitude(double* altitude);

    bool Read(const std::string& path, const bool as_rgb = true);

    bool Write(const std::string& path,
               const FREE_IMAGE_FORMAT format = FIF_UNKNOWN,
               const int flags = 0) const;

    Bitmap Rescale(const int new_width, const int new_height,
                   const FREE_IMAGE_FILTER filter = FILTER_BILINEAR);

    Bitmap Clone() const;

    Bitmap CloneAsGrey() const;

    Bitmap CloneAsRGB() const;

    void CloneMetadata(Bitmap* target) const;

    bool ReadExifTag(const FREE_IMAGE_MDMODEL model, const std::string& tag_name,
                     std::string* result) const;

private:
    std::shared_ptr<FIBITMAP> data_;
    int width_;
    int height_;
    int channels_;
};


class SceneGraph {
public:
    struct Correspondence {
        Correspondence()
                : image_id(kInvalidImageId), point2D_idx(kInvalidPoint2DIdx) { }

        Correspondence(const image_t image_id, const point2D_t point2D_idx)
                : image_id(image_id), point2D_idx(point2D_idx) { }

        image_t image_id;

        point2D_t point2D_idx;
    };

    SceneGraph();

    size_t NumImages() const;

    bool ExistsImage(const image_t image_id) const;

    point2D_t NumObservationsForImage(const image_t image_id) const;

    point2D_t NumCorrespondencesForImage(const image_t image_id) const;

    point2D_t NumCorrespondencesBetweenImages(
            const image_t image_id1, const image_t image_id2) const;

    const std::unordered_map<image_pair_t, point2D_t>&
            NumCorrespondencesBetweenImages() const;

    void Finalize();

    void AddImage(const image_t image_id, const size_t num_points2D);

    void AddCorrespondences(const image_t image_id1, const image_t image_id2,
                            const FeatureMatches& matches);

    const std::vector<Correspondence>& FindCorrespondences(
            const image_t image_id, const point2D_t point2D_idx) const;

    std::vector<Correspondence> FindTransitiveCorrespondences(
            const image_t image_id, const point2D_t point2D_idx,
            const size_t transitivity) const;

    std::vector<std::pair<point2D_t, point2D_t>> FindCorrespondencesBetweenImages(
            const image_t image_id1, const image_t image_id2) const;

    bool HasCorrespondences(const image_t image_id,
                            const point2D_t point2D_idx) const;

    bool IsTwoViewObservation(const image_t image_id,
                              const point2D_t point2D_idx) const;

private:
    struct Image {
        point2D_t num_observations = 0;

        point2D_t num_correspondences = 0;

        std::vector<std::vector<Correspondence>> corrs;
    };

    std::unordered_map<image_t, Image> images_;

    std::unordered_map<image_pair_t, point2D_t> image_pairs_;
};

inline int SQLite3CallHelper(const int result_code, const std::string& filename,
                             const int line_number) {
    switch (result_code) {
        case SQLITE_OK:
        case SQLITE_ROW:
        case SQLITE_DONE:
            return result_code;
        default:
            fprintf(stderr, "SQLite error [%s, line %i]: %s\n", filename.c_str(),
                    line_number, sqlite3_errstr(result_code));
            exit(EXIT_FAILURE);
    }
}

#define SQLITE3_CALL(func) SQLite3CallHelper(func, __FILE__, __LINE__)

#define SQLITE3_EXEC(database, sql, callback)                                 \
  {                                                                           \
    char* err_msg = nullptr;                                                  \
    int rc = sqlite3_exec(database, sql, callback, nullptr, &err_msg);        \
    if (rc != SQLITE_OK) {                                                    \
      fprintf(stderr, "SQLite error [%s, line %i]: %s\n", __FILE__, __LINE__, \
              err_msg);                                                       \
      sqlite3_free(err_msg);                                                  \
    }                                                                         \
  }

class Database {
public:
    const static int kSchemaVersion = 1;

    const static size_t kMaxNumImages;

    Database();

    ~Database();

    void Open(const std::string& path);

    void Close();

    void BeginTransaction() const;

    void EndTransaction() const;

    bool ExistsCamera(const camera_t camera_id) const;

    bool ExistsImage(const image_t image_id) const;

    bool ExistsImageName(std::string name) const;

    bool ExistsKeypoints(const image_t image_id) const;

    bool ExistsDescriptors(const image_t image_id) const;

    bool ExistsMatches(const image_t image_id1, const image_t image_id2) const;

    bool ExistsInlierMatches(const image_t image_id1,
                             const image_t image_id2) const;

    size_t NumCameras() const;

    size_t NumImages() const;

    size_t NumKeypoints() const;

    size_t NumDescriptors() const;

    size_t NumMatches() const;

    size_t NumInlierMatches() const;

    size_t NumMatchedImagePairs() const;

    size_t NumVerifiedImagePairs() const;

    static image_pair_t ImagePairToPairId(const image_t image_id1, const image_t image_id2);

    static void PairIdToImagePair(const image_pair_t pair_id, image_t* image_id1, image_t* image_id2);

    static bool SwapImagePair(const image_t image_id1, const image_t image_id2);

    Camera ReadCamera(const camera_t camera_id) const;

    std::vector<Camera> ReadAllCameras() const;

    Image ReadImage(const image_t image_id) const;

    Image ReadImageFromName(const std::string& name) const;

    std::vector<Image> ReadAllImages() const;

    FeatureKeypoints ReadKeypoints(const image_t image_id) const;

    FeatureDescriptors ReadDescriptors(const image_t image_id) const;

    FeatureMatches ReadMatches(const image_t image_id1, const image_t image_id2) const;

    std::vector<std::pair<image_pair_t, FeatureMatches>> ReadAllMatches() const;

    TwoViewGeometry ReadInlierMatches(const image_t image_id1, const image_t image_id2) const;

    std::vector<std::pair<image_pair_t, TwoViewGeometry>> ReadAllInlierMatches()
            const;

    void ReadInlierMatchesGraph(
            std::vector<std::pair<image_t, image_t>>* image_pairs,
            std::vector<int>* num_inliers) const;

    camera_t WriteCamera(const Camera& camera, const bool use_camera_id = false) const;

    image_t WriteImage(const Image& image, const bool use_image_id = false) const;

    void WriteKeypoints(const image_t image_id, const FeatureKeypoints& keypoints) const;

    void WriteDescriptors(const image_t image_id, const FeatureDescriptors& descriptors) const;

    void WriteMatches(const image_t image_id1, const image_t image_id2, const FeatureMatches& matches) const;

    void WriteInlierMatches(
            const image_t image_id1,
            const image_t image_id2,
            const TwoViewGeometry& two_view_geometry
    ) const;

    void UpdateCamera(const Camera& camera);

    void UpdateImage(const Image& image);

private:
    void PrepareSQLStatements();

    void FinalizeSQLStatements();

    void CreateTables() const;

    void CreateCameraTable() const;

    void CreateImageTable() const;

    void CreateKeypointsTable() const;

    void CreateDescriptorsTable() const;

    void CreateMatchesTable() const;

    void CreateInlierMatchesTable() const;

    void UpdateSchema() const;

    bool ExistsRowId(sqlite3_stmt* sql_stmt, const size_t row_id) const;

    bool ExistsRowString(sqlite3_stmt* sql_stmt,
                         const std::string& row_entry) const;

    size_t CountRows(const std::string& table) const;

    size_t SumColumn(const std::string& column, const std::string& table) const;

    sqlite3* database_;

    std::vector<sqlite3_stmt*> sql_stmts_;

    sqlite3_stmt* sql_stmt_exists_camera_;
    sqlite3_stmt* sql_stmt_exists_image_id_;
    sqlite3_stmt* sql_stmt_exists_image_name_;
    sqlite3_stmt* sql_stmt_exists_keypoints_;
    sqlite3_stmt* sql_stmt_exists_descriptors_;
    sqlite3_stmt* sql_stmt_exists_matches_;
    sqlite3_stmt* sql_stmt_exists_inlier_matches_;

    sqlite3_stmt* sql_stmt_add_camera_;
    sqlite3_stmt* sql_stmt_add_image_;

    sqlite3_stmt* sql_stmt_update_camera_;
    sqlite3_stmt* sql_stmt_update_image_;

    sqlite3_stmt* sql_stmt_read_camera_;
    sqlite3_stmt* sql_stmt_read_cameras_;
    sqlite3_stmt* sql_stmt_read_image_id_;
    sqlite3_stmt* sql_stmt_read_image_name_;
    sqlite3_stmt* sql_stmt_read_images_;
    sqlite3_stmt* sql_stmt_read_keypoints_;
    sqlite3_stmt* sql_stmt_read_descriptors_;
    sqlite3_stmt* sql_stmt_read_matches_;
    sqlite3_stmt* sql_stmt_read_matches_all_;
    sqlite3_stmt* sql_stmt_read_inlier_matches_;
    sqlite3_stmt* sql_stmt_read_inlier_matches_all_;
    sqlite3_stmt* sql_stmt_read_inlier_matches_graph_;

    sqlite3_stmt* sql_stmt_write_keypoints_;
    sqlite3_stmt* sql_stmt_write_descriptors_;
    sqlite3_stmt* sql_stmt_write_matches_;
    sqlite3_stmt* sql_stmt_write_inlier_matches_;
};


class DatabaseCache {
public:
    DatabaseCache();

    size_t NumCameras() const;

    size_t NumImages() const;

    class Camera& Camera(const camera_t camera_id);

    const class Camera& Camera(const camera_t camera_id) const;

    class Image& Image(const image_t image_id);

    const class Image& Image(const image_t image_id) const;

    const std::unordered_map<camera_t, class Camera>& Cameras() const;

    const std::unordered_map<image_t, class Image>& Images() const;

    bool ExistsCamera(const camera_t camera_id) const;

    bool ExistsImage(const image_t image_id) const;

    const class SceneGraph& SceneGraph() const;

    void AddCamera(const class Camera& camera);

    void AddImage(const class Image& image);

    void Load(const Database& database, const size_t min_num_matches,
              const bool ignore_watermarks);

private:
    class SceneGraph scene_graph_;

    std::unordered_map<camera_t, class Camera> cameras_;
    std::unordered_map<image_t, class Image> images_;
};


#endif //INC_3D_RECONSTRUCTION_STORAGE_H
