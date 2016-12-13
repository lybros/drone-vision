#include "storage.h"

Bitmap::Bitmap() : width_(0), height_(0), channels_(0) { }

Bitmap::Bitmap(FIBITMAP* data) {
    data_.reset(data, &FreeImage_Unload);

    width_ = FreeImage_GetWidth(data);
    height_ = FreeImage_GetHeight(data);

    const FREE_IMAGE_COLOR_TYPE color_type = FreeImage_GetColorType(data);

    const bool is_grey =
            color_type == FIC_MINISBLACK && FreeImage_GetBPP(data) == 8;
    const bool is_rgb = color_type == FIC_RGB && FreeImage_GetBPP(data) == 24;

    if (!is_grey && !is_rgb) {
        FIBITMAP* data_converted = FreeImage_ConvertTo24Bits(data);
        data_.reset(data_converted, &FreeImage_Unload);
        channels_ = 3;
    } else {
        channels_ = is_rgb ? 3 : 1;
    }
}

bool Bitmap::Allocate(const int width, const int height, const bool as_rgb) {
    FIBITMAP* data = nullptr;
    width_ = width;
    height_ = height;
    if (as_rgb) {
        const int kNumBitsPerPixel = 24;
        data = FreeImage_Allocate(width, height, kNumBitsPerPixel);
        channels_ = 3;
    } else {
        const int kNumBitsPerPixel = 8;
        data = FreeImage_Allocate(width, height, kNumBitsPerPixel);
        channels_ = 1;
    }
    data_.reset(data, &FreeImage_Unload);
    return data != nullptr;
}

std::vector<uint8_t> Bitmap::ConvertToRawBits() const {
    const unsigned int scan_width = ScanWidth();
    const unsigned int bpp = BitsPerPixel();
    const bool kTopDown = true;
    std::vector<uint8_t> raw_bits(bpp * scan_width * height_, 0);
    FreeImage_ConvertToRawBits(raw_bits.data(), data_.get(), scan_width, bpp,
                               FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK,
                               FI_RGBA_BLUE_MASK, kTopDown);
    return raw_bits;
}

std::vector<uint8_t> Bitmap::ConvertToRowMajorArray() const {
    const unsigned int scan_width = ScanWidth();
    const std::vector<uint8_t> raw_bits = ConvertToRawBits();
    std::vector<uint8_t> array(width_ * height_ * channels_, 0);

    size_t i = 0;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            for (int d = 0; d < channels_; ++d) {
                array[i] = raw_bits[y * scan_width + x * channels_ + d];
                i += 1;
            }
        }
    }

    return array;
}

std::vector<uint8_t> Bitmap::ConvertToColMajorArray() const {
    const unsigned int scan_width = ScanWidth();
    const std::vector<uint8_t> raw_bits = ConvertToRawBits();
    std::vector<uint8_t> array(width_ * height_ * channels_, 0);

    size_t i = 0;
    for (int d = 0; d < channels_; ++d) {
        for (int x = 0; x < width_; ++x) {
            for (int y = 0; y < height_; ++y) {
                array[i] = raw_bits[y * scan_width + x * channels_ + d];
                i += 1;
            }
        }
    }

    return array;
}

bool Bitmap::GetPixel(const int x, const int y, Eigen::Vector3ub* color) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
    }

    const uint8_t* line = FreeImage_GetScanLine(data_.get(), height_ - 1 - y);

    if (IsGrey()) {
        (*color)(0) = line[x];
        (*color)(1) = (*color)(0);
        (*color)(2) = (*color)(0);
        return true;
    } else if (IsRGB()) {
        (*color)(0) = line[3 * x + FI_RGBA_RED];
        (*color)(1) = line[3 * x + FI_RGBA_GREEN];
        (*color)(2) = line[3 * x + FI_RGBA_BLUE];
        return true;
    }

    return false;
}

bool Bitmap::SetPixel(const int x, const int y, const Eigen::Vector3ub& color) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
    }

    uint8_t* line = FreeImage_GetScanLine(data_.get(), height_ - 1 - y);

    if (IsGrey()) {
        line[x] = color(0);
        return true;
    } else if (IsRGB()) {
        line[3 * x + FI_RGBA_RED] = color(0);
        line[3 * x + FI_RGBA_GREEN] = color(1);
        line[3 * x + FI_RGBA_BLUE] = color(2);
        return true;
    }

    return false;
}

void Bitmap::Fill(const Eigen::Vector3ub& color) {
    for (int y = 0; y < height_; ++y) {
        uint8_t* line = FreeImage_GetScanLine(data_.get(), height_ - 1 - y);
        for (int x = 0; x < width_; ++x) {
            if (IsGrey()) {
                line[x] = color(0);
            } else if (IsRGB()) {
                line[3 * x + FI_RGBA_RED] = color(0);
                line[3 * x + FI_RGBA_GREEN] = color(1);
                line[3 * x + FI_RGBA_BLUE] = color(2);
            }
        }
    }
}

bool Bitmap::InterpolateNearestNeighbor(const double x, const double y,
                                        Eigen::Vector3ub* color) const {
    const int xx = static_cast<int>(std::round(x));
    const int yy = static_cast<int>(std::round(y));
    return GetPixel(xx, yy, color);
}

bool Bitmap::InterpolateBilinear(const double x, const double y,
                                 Eigen::Vector3d* color) const {
    const double inv_y = height_ - 1 - y;

    const int x0 = static_cast<int>(std::floor(x));
    const int x1 = x0 + 1;
    const int y0 = static_cast<int>(std::floor(inv_y));
    const int y1 = y0 + 1;

    if (x0 < 0 || x1 >= width_ || y0 < 0 || y1 >= height_) {
        return false;
    }

    const double dx = x - x0;
    const double dy = inv_y - y0;
    const double dx_1 = 1 - dx;
    const double dy_1 = 1 - dy;

    const uint8_t* line0 = FreeImage_GetScanLine(data_.get(), y0);
    const uint8_t* line1 = FreeImage_GetScanLine(data_.get(), y1);

    if (IsGrey()) {
        const double v0 = dx_1 * line0[x0] + dx * line0[x1];

        const double v1 = dx_1 * line1[x0] + dx * line1[x1];

        (*color)(0) = dy_1 * v0 + dy * v1;
        (*color)(1) = (*color)(0);
        (*color)(2) = (*color)(0);
        return true;
    } else if (IsRGB()) {
        const uint8_t* p00 = &line0[3 * x0];
        const uint8_t* p01 = &line0[3 * x1];
        const uint8_t* p10 = &line1[3 * x0];
        const uint8_t* p11 = &line1[3 * x1];

        const double v0_r = dx_1 * p00[FI_RGBA_RED] + dx * p01[FI_RGBA_RED];
        const double v0_g = dx_1 * p00[FI_RGBA_GREEN] + dx * p01[FI_RGBA_GREEN];
        const double v0_b = dx_1 * p00[FI_RGBA_BLUE] + dx * p01[FI_RGBA_BLUE];

        const double v1_r = dx_1 * p10[FI_RGBA_RED] + dx * p11[FI_RGBA_RED];
        const double v1_g = dx_1 * p10[FI_RGBA_GREEN] + dx * p11[FI_RGBA_GREEN];
        const double v1_b = dx_1 * p10[FI_RGBA_BLUE] + dx * p11[FI_RGBA_BLUE];

        (*color)(0) = dy_1 * v0_r + dy * v1_r;
        (*color)(1) = dy_1 * v0_g + dy * v1_g;
        (*color)(2) = dy_1 * v0_b + dy * v1_b;
        return true;
    }

    return false;
}

bool Bitmap::ExifFocalLength(double* focal_length) {
    const double max_size = std::max(width_, height_);

    std::string focal_length_35mm_str;
    if (ReadExifTag(FIMD_EXIF_EXIF, "FocalLengthIn35mmFilm",
                    &focal_length_35mm_str)) {
        const boost::regex regex(".*?([0-9.]+).*?mm.*?");
        boost::cmatch result;
        if (boost::regex_search(focal_length_35mm_str.c_str(), result, regex)) {
            const double focal_length_35 = boost::lexical_cast<double>(result[1]);
            if (focal_length_35 > 0) {
                *focal_length = focal_length_35 / 35.0 * max_size;
                return true;
            }
        }
    }

    std::string focal_length_str;
    if (ReadExifTag(FIMD_EXIF_EXIF, "FocalLength", &focal_length_str)) {
        boost::regex regex(".*?([0-9.]+).*?mm");
        boost::cmatch result;
        if (boost::regex_search(focal_length_str.c_str(), result, regex)) {
            const double focal_length_mm = boost::lexical_cast<double>(result[1]);

            std::string pixel_x_dim_str;
            std::string x_res_str;
            std::string res_unit_str;
            if (ReadExifTag(FIMD_EXIF_EXIF, "PixelXDimension", &pixel_x_dim_str) &&
                ReadExifTag(FIMD_EXIF_EXIF, "FocalPlaneXResolution", &x_res_str) &&
                ReadExifTag(FIMD_EXIF_EXIF, "FocalPlaneResolutionUnit", &res_unit_str)) {
                regex = boost::regex(".*?([0-9.]+).*?");
                if (boost::regex_search(pixel_x_dim_str.c_str(), result, regex)) {
                    const double pixel_x_dim = boost::lexical_cast<double>(result[1]);
                    regex = boost::regex(".*?([0-9.]+).*?/.*?([0-9.]+).*?");
                    if (boost::regex_search(x_res_str.c_str(), result, regex)) {
                        const double x_res = boost::lexical_cast<double>(result[2]) /
                                             boost::lexical_cast<double>(result[1]);
                        const double ccd_width = x_res * pixel_x_dim;
                        if (ccd_width > 0 && focal_length_mm > 0) {
                            if (res_unit_str == "cm") {
                                *focal_length = focal_length_mm / (ccd_width * 10.0) * max_size;
                                return true;
                            } else if (res_unit_str == "inches") {
                                *focal_length = focal_length_mm / (ccd_width * 25.4) * max_size;
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }

    return false;
}

bool Bitmap::ExifLatitude(double* latitude) {
    std::string str;
    if (ReadExifTag(FIMD_EXIF_GPS, "GPSLatitude", &str)) {
        const boost::regex regex(".*?([0-9.]+):([0-9.]+):([0-9.]+).*?");
        boost::cmatch result;
        if (boost::regex_search(str.c_str(), result, regex)) {
            const double hours = boost::lexical_cast<double>(result[1]);
            const double minutes = boost::lexical_cast<double>(result[2]);
            const double seconds = boost::lexical_cast<double>(result[3]);
            *latitude = hours + minutes / 60.0 + seconds / 3600.0;
            return true;
        }
    }
    return false;
}

bool Bitmap::ExifLongitude(double* longitude) {
    std::string str;
    if (ReadExifTag(FIMD_EXIF_GPS, "GPSLongitude", &str)) {
        const boost::regex regex(".*?([0-9.]+):([0-9.]+):([0-9.]+).*?");
        boost::cmatch result;
        if (boost::regex_search(str.c_str(), result, regex)) {
            const double hours = boost::lexical_cast<double>(result[1]);
            const double minutes = boost::lexical_cast<double>(result[2]);
            const double seconds = boost::lexical_cast<double>(result[3]);
            *longitude = hours + minutes / 60.0 + seconds / 3600.0;
            return true;
        }
    }
    return false;
}

bool Bitmap::ExifAltitude(double* altitude) {
    std::string str;
    if (ReadExifTag(FIMD_EXIF_GPS, "GPSAltitude", &str)) {
        const boost::regex regex(".*?([0-9.]+).*?/.*?([0-9.]+).*?");
        boost::cmatch result;
        if (boost::regex_search(str.c_str(), result, regex)) {
            *altitude = boost::lexical_cast<double>(result[1]) / boost::lexical_cast<double>(result[2]);
            return true;
        }
    }
    return false;
}

bool Bitmap::Read(const std::string& path, const bool as_rgb) {
    if (!boost::filesystem::exists(path)) { return false; }

    const FREE_IMAGE_FORMAT format = FreeImage_GetFileType(path.c_str(), 0);

    if (format == FIF_UNKNOWN) { return false; }

    FIBITMAP* fi_bitmap = FreeImage_Load(format, path.c_str());
    data_.reset(fi_bitmap, &FreeImage_Unload);

    const FREE_IMAGE_COLOR_TYPE color_type = FreeImage_GetColorType(fi_bitmap);

    const bool is_grey = color_type == FIC_MINISBLACK && FreeImage_GetBPP(fi_bitmap) == 8;
    const bool is_rgb = color_type == FIC_RGB && FreeImage_GetBPP(fi_bitmap) == 24;

    if (!is_rgb && as_rgb) {
        FIBITMAP* converted_bitmap = FreeImage_ConvertTo24Bits(fi_bitmap);
        data_.reset(converted_bitmap, &FreeImage_Unload);
    } else if (!is_grey && !as_rgb) {
        FIBITMAP* converted_bitmap = FreeImage_ConvertToGreyscale(fi_bitmap);
        data_.reset(converted_bitmap, &FreeImage_Unload);
    }

    width_ = FreeImage_GetWidth(data_.get());
    height_ = FreeImage_GetHeight(data_.get());
    channels_ = as_rgb ? 3 : 1;

    return true;
}

bool Bitmap::Write(const std::string& path, const FREE_IMAGE_FORMAT format, const int flags) const {
    FREE_IMAGE_FORMAT save_format;
    if (format == FIF_UNKNOWN) {
        save_format = FreeImage_GetFIFFromFilename(path.c_str());
    } else {
        save_format = format;
    }

    if (flags == 0) {
        FreeImage_Save(save_format, data_.get(), path.c_str());
    } else {
        FreeImage_Save(save_format, data_.get(), path.c_str(), flags);
    }

    return true;
}

Bitmap Bitmap::Rescale(const int new_width, const int new_height, const FREE_IMAGE_FILTER filter) {
    FIBITMAP* rescaled = FreeImage_Rescale(data_.get(), new_width, new_height, filter);
    return Bitmap(rescaled);
}

Bitmap Bitmap::Clone() const { return Bitmap(FreeImage_Clone(data_.get())); }

Bitmap Bitmap::CloneAsGrey() const {
    if (IsGrey()) { return Clone(); }

    return Bitmap(FreeImage_ConvertToGreyscale(data_.get()));
}

Bitmap Bitmap::CloneAsRGB() const {
    if (IsRGB()) { return Clone(); }

    return Bitmap(FreeImage_ConvertTo24Bits(data_.get()));
}

void Bitmap::CloneMetadata(Bitmap* target) const {
    CHECK_NOTNULL(target);
    CHECK_NOTNULL(target->Data());
    FreeImage_CloneMetadata(data_.get(), target->Data());
}

bool Bitmap::ReadExifTag(const FREE_IMAGE_MDMODEL model, const std::string& tag_name, std::string* result) const {
    FITAG* tag = nullptr;
    FreeImage_GetMetadata(model, data_.get(), tag_name.c_str(), &tag);

    if (tag == nullptr) {
        *result = "";
        return false;
    }

    if (tag_name == "FocalPlaneXResolution") {
        *result = std::string(FreeImage_TagToString(FIMD_EXIF_INTEROP, tag));
    } else {
        *result = FreeImage_TagToString(model, tag);
    }

    return true;
}

FIBITMAP* Bitmap::Data() { return data_.get(); }

const FIBITMAP* Bitmap::Data() const { return data_.get(); }

int Bitmap::Width() const { return width_; }

int Bitmap::Height() const { return height_; }

int Bitmap::Channels() const { return channels_; }

unsigned int Bitmap::BitsPerPixel() const { return FreeImage_GetBPP(data_.get()); }

unsigned int Bitmap::ScanWidth() const { return FreeImage_GetPitch(data_.get()); }

bool Bitmap::IsRGB() const { return channels_ == 3; }

bool Bitmap::IsGrey() const { return channels_ == 1; }


SceneGraph::SceneGraph() { }

void SceneGraph::Finalize() {
    for (auto it = images_.begin(); it != images_.end();) {
        it->second.num_observations = 0;
        for (auto& corr : it->second.corrs) {
            corr.shrink_to_fit();
            if (corr.size() > 0) {
                it->second.num_observations += 1;
            }
        }
        if (it->second.num_observations == 0) {
            images_.erase(it++);
        } else { ++it; }
    }
}

void SceneGraph::AddImage(const image_t image_id, const size_t num_points) {
    images_[image_id].corrs.resize(num_points);
}

void SceneGraph::AddCorrespondences(const image_t image_id1, const image_t image_id2, const FeatureMatches& matches) {
    if (image_id1 == image_id2) {
        std::cout << "WARNING: Cannot use self-matches for image_id=" << image_id1 << std::endl;
        return;
    }

    struct Image& image1 = images_.at(image_id1);
    struct Image& image2 = images_.at(image_id2);

    image1.num_correspondences += matches.size();
    image2.num_correspondences += matches.size();

    const image_pair_t pair_id = Database::ImagePairToPairId(image_id1, image_id2);
    point2D_t& num_correspondences = image_pairs_[pair_id];
    num_correspondences += static_cast<point2D_t>(matches.size());

    for (size_t i = 0; i < matches.size(); ++i) {
        const point2D_t point2D_idx1 = matches[i].point2D_idx1;
        const point2D_t point2D_idx2 = matches[i].point2D_idx2;

        const bool valid_idx1 = point2D_idx1 < image1.corrs.size();
        const bool valid_idx2 = point2D_idx2 < image2.corrs.size();

        if (valid_idx1 && valid_idx2) {
            const bool duplicate = std::find_if(
                    image1.corrs[point2D_idx1].begin(),
                    image1.corrs[point2D_idx1].end(),
                    [image_id2, point2D_idx2](const Correspondence& corr) {
                        return corr.image_id == image_id2 && corr.point2D_idx == point2D_idx2;
                    }
            ) != image1.corrs[point2D_idx1].end();

            if (duplicate) {
                image1.num_correspondences -= 1;
                image2.num_correspondences -= 1;
                num_correspondences -= 1;
                std::cout << (QString(
                        "WARNING: Duplicate correspondence between "
                                "point2D_idx=%1 in image_id=%2 and point2D_idx=%3 in image_id=%4"
                ).arg(point2D_idx1).arg(image_id1).arg(point2D_idx2).arg(image_id2)).toStdString() << std::endl;
            } else {
                std::vector<Correspondence>& corrs1 = image1.corrs[point2D_idx1];
                corrs1.emplace_back(image_id2, point2D_idx2);

                std::vector<Correspondence>& corrs2 = image2.corrs[point2D_idx2];
                corrs2.emplace_back(image_id1, point2D_idx1);
            }
        } else {
            image1.num_correspondences -= 1;
            image2.num_correspondences -= 1;
            num_correspondences -= 1;
            if (!valid_idx1) {
                std::cout << QString().sprintf(
                        "WARNING: point2D_idx=%u in image_id=%u does not exist", point2D_idx1, image_id1
                ).toStdString() << std::endl;
            }
            if (!valid_idx2) {
                std::cout << QString().sprintf(
                        "WARNING: point2D_idx=%u in image_id=%u does not exist", point2D_idx2, image_id2
                ).toStdString() << std::endl;
            }
        }
    }
}

std::vector<SceneGraph::Correspondence>SceneGraph::FindTransitiveCorrespondences(
        const image_t image_id,
        const point2D_t point2D_idx,
        const size_t transitivity
) const {

    if (transitivity == 1) { return FindCorrespondences(image_id, point2D_idx); }

    std::vector<Correspondence> found_corrs;
    if (!HasCorrespondences(image_id, point2D_idx)) { return found_corrs; }

    found_corrs.emplace_back(image_id, point2D_idx);

    std::unordered_map<image_t, std::unordered_set<point2D_t>> image_corrs;
    image_corrs[image_id].insert(point2D_idx);

    size_t corr_queue_begin = 0;
    size_t corr_queue_end = found_corrs.size();

    for (size_t t = 0; t < transitivity; ++t) {
        for (size_t i = corr_queue_begin; i < corr_queue_end; ++i) {
            const Correspondence ref_corr = found_corrs[i];

            const Image& image = images_.at(ref_corr.image_id);
            const std::vector<Correspondence>& ref_corrs = image.corrs[ref_corr.point2D_idx];

            for (const Correspondence corr : ref_corrs) {
                if (image_corrs[corr.image_id].count(corr.point2D_idx) == 0) {
                    image_corrs[corr.image_id].insert(corr.point2D_idx);
                    found_corrs.emplace_back(corr.image_id, corr.point2D_idx);
                }
            }
        }

        corr_queue_begin = corr_queue_end;
        corr_queue_end = found_corrs.size();

        if (corr_queue_begin == corr_queue_end) { break; }
    }

    if (found_corrs.size() > 1) {
        found_corrs.front() = found_corrs.back();
    }
    found_corrs.pop_back();

    return found_corrs;
}

std::vector<std::pair<point2D_t, point2D_t>>SceneGraph::FindCorrespondencesBetweenImages(
        const image_t image_id1,
        const image_t image_id2
) const {

    std::vector<std::pair<point2D_t, point2D_t>> found_corrs;
    const struct Image& image1 = images_.at(image_id1);
    for (point2D_t point2D_idx1 = 0; point2D_idx1 < image1.corrs.size(); ++point2D_idx1) {

        for (const Correspondence& corr1 : image1.corrs[point2D_idx1]) {

            if (corr1.image_id == image_id2) {
                found_corrs.emplace_back(point2D_idx1, corr1.point2D_idx);
            }
        }
    }
    return found_corrs;
}

bool SceneGraph::IsTwoViewObservation(const image_t image_id, const point2D_t point2D_idx) const {

    const struct Image& image = images_.at(image_id);
    const std::vector<Correspondence>& corrs = image.corrs.at(point2D_idx);

    if (corrs.size() != 1) { return false; }
    const struct Image& other_image = images_.at(corrs[0].image_id);
    const std::vector<Correspondence>& other_corrs = other_image.corrs.at(corrs[0].point2D_idx);
    return other_corrs.size() == 1;
}

size_t SceneGraph::NumImages() const { return images_.size(); }

bool SceneGraph::ExistsImage(const image_t image_id) const { return images_.count(image_id) > 0; }

point2D_t SceneGraph::NumObservationsForImage(const image_t image_id) const {
    return images_.at(image_id).num_observations;
}

point2D_t SceneGraph::NumCorrespondencesForImage(const image_t image_id) const {
    return images_.at(image_id).num_correspondences;
}

point2D_t SceneGraph::NumCorrespondencesBetweenImages(const image_t image_id1, const image_t image_id2) const {
    const image_pair_t pair_id = Database::ImagePairToPairId(image_id1, image_id2);
    if (image_pairs_.count(pair_id) == 0) {
        return 0;
    } else {
        return static_cast<point2D_t>(image_pairs_.at(pair_id));
    }
}

const std::unordered_map<image_pair_t, point2D_t>& SceneGraph::CorrespondencesBetweenImages() const {
    return image_pairs_;
}

const std::vector<SceneGraph::Correspondence>& SceneGraph::FindCorrespondences(
        const image_t image_id,
        const point2D_t point2D_idx
) const { return images_.at(image_id).corrs.at(point2D_idx); }

bool SceneGraph::HasCorrespondences(const image_t image_id, const point2D_t point2D_idx) const {
    return !images_.at(image_id).corrs.at(point2D_idx).empty();
}


namespace {

    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> FeatureKeypointsBlob;
    typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> FeatureDescriptorsBlob;
    typedef Eigen::Matrix<point2D_t, Eigen::Dynamic, 2, Eigen::RowMajor> FeatureMatchesBlob;

    void SwapFeatureMatchesBlob(FeatureMatchesBlob* matches) {
        matches->col(0).swap(matches->col(1));
    }

    FeatureKeypointsBlob FeatureKeypointsToBlob(const FeatureKeypoints& keypoints) {
        const FeatureKeypointsBlob::Index kNumCols = 4;
        FeatureKeypointsBlob blob(keypoints.size(), kNumCols);
        for (size_t i = 0; i < keypoints.size(); ++i) {
            blob(i, 0) = keypoints[i].x;
            blob(i, 1) = keypoints[i].y;
            blob(i, 2) = keypoints[i].scale_or_size;
            blob(i, 3) = keypoints[i].orientation_or_angle;
        }
        return blob;
    }

    FeatureKeypoints FeatureKeypointsFromBlob(const FeatureKeypointsBlob& blob) {
        FeatureKeypoints keypoints(static_cast<size_t>(blob.rows()));
        for (FeatureKeypointsBlob::Index i = 0; i < blob.rows(); ++i) {
            keypoints[i].x = blob(i, 0);
            keypoints[i].y = blob(i, 1);
            keypoints[i].scale_or_size = blob(i, 2);
            keypoints[i].orientation_or_angle = blob(i, 3);
        }
        return keypoints;
    }

    FeatureMatchesBlob FeatureMatchesToBlob(const FeatureMatches& matches) {
        const FeatureMatchesBlob::Index kNumCols = 2;
        FeatureMatchesBlob blob(matches.size(), kNumCols);
        for (size_t i = 0; i < matches.size(); ++i) {
            blob(i, 0) = matches[i].point2D_idx1;
            blob(i, 1) = matches[i].point2D_idx2;
        }
        return blob;
    }

    FeatureMatches FeatureMatchesFromBlob(const FeatureMatchesBlob& blob) {
        FeatureMatches matches(static_cast<size_t>(blob.rows()));
        for (FeatureMatchesBlob::Index i = 0; i < blob.rows(); ++i) {
            matches[i].point2D_idx1 = blob(i, 0);
            matches[i].point2D_idx2 = blob(i, 1);
        }
        return matches;
    }

    template<typename MatrixType>
    MatrixType ReadMatrixBlob(sqlite3_stmt* sql_stmt, const int rc, const int col) {
        MatrixType matrix;

        if (rc == SQLITE_ROW) {
            const size_t rows = static_cast<size_t>(sqlite3_column_int64(sql_stmt, col + 0));
            const size_t cols = static_cast<size_t>(sqlite3_column_int64(sql_stmt, col + 1));

            matrix = MatrixType(rows, cols);

            const size_t num_bytes = static_cast<size_t>(sqlite3_column_bytes(sql_stmt, col + 2));

            memcpy(reinterpret_cast<char*>(matrix.data()), sqlite3_column_blob(sql_stmt, col + 2), num_bytes);
        } else {
            const typename MatrixType::Index rows = (MatrixType::RowsAtCompileTime == Eigen::Dynamic) ?
                                                    0 : MatrixType::RowsAtCompileTime;

            const typename MatrixType::Index cols = (MatrixType::ColsAtCompileTime == Eigen::Dynamic) ?
                                                    0 : MatrixType::ColsAtCompileTime;
            matrix = MatrixType(rows, cols);
        }

        return matrix;
    }

    template<typename MatrixType>
    void WriteMatrixBlob(sqlite3_stmt* sql_stmt, const MatrixType& matrix, const int col) {
        const size_t num_bytes = matrix.size() * sizeof(typename MatrixType::Scalar);
        SQLITE3_CALL(sqlite3_bind_int64(sql_stmt, col + 0, matrix.rows()));
        SQLITE3_CALL(sqlite3_bind_int64(sql_stmt, col + 1, matrix.cols()));
        SQLITE3_CALL(sqlite3_bind_blob(
                sql_stmt,
                col + 2,
                reinterpret_cast<const char*>(matrix.data()),
                static_cast<int>(num_bytes),
                SQLITE_STATIC)
        );
    }

    Camera ReadCameraRow(sqlite3_stmt* sql_stmt) {
        Camera camera;

        camera.SetCameraId(static_cast<camera_t>(sqlite3_column_int64(sql_stmt, 0)));
        camera.SetModelId(sqlite3_column_int64(sql_stmt, 1));
        camera.SetWidth(static_cast<size_t>(sqlite3_column_int64(sql_stmt, 2)));
        camera.SetHeight(static_cast<size_t>(sqlite3_column_int64(sql_stmt, 3)));

        const size_t num_params_bytes = static_cast<size_t>(sqlite3_column_bytes(sql_stmt, 4));
        camera.Params().resize(num_params_bytes / sizeof(double));
        memcpy(camera.ParamsData(), sqlite3_column_blob(sql_stmt, 4), num_params_bytes);
        camera.SetPriorFocalLength(sqlite3_column_int64(sql_stmt, 5) != 0);
        return camera;
    }

    Image ReadImageRow(sqlite3_stmt* sql_stmt) {
        Image image;
        image.SetImageId(static_cast<image_t>(sqlite3_column_int64(sql_stmt, 0)));
        image.SetName(std::string(reinterpret_cast<const char*>(sqlite3_column_text(sql_stmt, 1))));
        image.SetCameraId(static_cast<camera_t>(sqlite3_column_int64(sql_stmt, 2)));
        image.QvecPrior(0) = sqlite3_column_double(sql_stmt, 3);
        image.QvecPrior(1) = sqlite3_column_double(sql_stmt, 4);
        image.QvecPrior(2) = sqlite3_column_double(sql_stmt, 5);
        image.QvecPrior(3) = sqlite3_column_double(sql_stmt, 6);
        image.TvecPrior(0) = sqlite3_column_double(sql_stmt, 7);
        image.TvecPrior(1) = sqlite3_column_double(sql_stmt, 8);
        image.TvecPrior(2) = sqlite3_column_double(sql_stmt, 9);
        return image;
    }

}

const size_t Database::kMaxNumImages = static_cast<size_t>(std::numeric_limits<int32_t>::max());

Database::Database() { database_ = nullptr; }

Database::~Database() { Close(); }

void Database::Open(const std::string& path) {
    Close();

    SQLITE3_CALL(sqlite3_open_v2(
            path.c_str(), &database_,
            SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX,
            nullptr));

    SQLITE3_EXEC(database_, "PRAGMA synchronous=OFF", nullptr);

    SQLITE3_EXEC(database_, "PRAGMA journal_mode=WAL", nullptr);

    SQLITE3_EXEC(database_, "PRAGMA temp_store=MEMORY", nullptr);

    SQLITE3_EXEC(database_, "PRAGMA foreign_keys=ON", nullptr);

    CreateTables();
    UpdateSchema();

    PrepareSQLStatements();
}

void Database::Close() {
    if (database_ != nullptr) {
        FinalizeSQLStatements();
        sqlite3_close_v2(database_);
        database_ = nullptr;
    }
}

void Database::BeginTransaction() const {
    SQLITE3_EXEC(database_, "BEGIN TRANSACTION", nullptr);
}

void Database::EndTransaction() const {
    SQLITE3_EXEC(database_, "END TRANSACTION", nullptr);
}

bool Database::ExistsCamera(const camera_t camera_id) const {
    return ExistsRowId(sql_stmt_exists_camera_, camera_id);
}

bool Database::ExistsImage(const image_t image_id) const {
    return ExistsRowId(sql_stmt_exists_image_id_, image_id);
}

bool Database::ExistsImageName(std::string name) const {
    return ExistsRowString(sql_stmt_exists_image_name_, name);
}

bool Database::ExistsKeypoints(const image_t image_id) const {
    return ExistsRowId(sql_stmt_exists_keypoints_, image_id);
}

bool Database::ExistsDescriptors(const image_t image_id) const {
    return ExistsRowId(sql_stmt_exists_descriptors_, image_id);
}

bool Database::ExistsMatches(const image_t image_id1, const image_t image_id2) const {

    return ExistsRowId(sql_stmt_exists_matches_, ImagePairToPairId(image_id1, image_id2));
}

bool Database::ExistsInlierMatches(const image_t image_id1, const image_t image_id2) const {

    return ExistsRowId(sql_stmt_exists_inlier_matches_, ImagePairToPairId(image_id1, image_id2));
}

size_t Database::NumCameras() const { return CountRows("cameras"); }

size_t Database::NumImages() const { return CountRows("images"); }

size_t Database::NumKeypoints() const { return SumColumn("rows", "keypoints"); }

size_t Database::NumDescriptors() const { return SumColumn("rows", "descriptors"); }

size_t Database::NumMatches() const { return SumColumn("rows", "matches"); }

size_t Database::NumInlierMatches() const { return SumColumn("rows", "inlier_matches"); }

size_t Database::NumMatchedImagePairs() const { return CountRows("matches"); }

size_t Database::NumVerifiedImagePairs() const { return CountRows("inlier_matches"); }

Camera Database::ReadCamera(const camera_t camera_id) const {
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_read_camera_, 1, camera_id));

    Camera camera;

    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_camera_));
    if (rc == SQLITE_ROW) {
        camera = ReadCameraRow(sql_stmt_read_camera_);
    }

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_camera_));

    return camera;
}

std::vector<Camera> Database::ReadAllCameras() const {
    std::vector<Camera> cameras;

    while (SQLITE3_CALL(sqlite3_step(sql_stmt_read_cameras_)) == SQLITE_ROW) {
        cameras.push_back(ReadCameraRow(sql_stmt_read_cameras_));
    }

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_cameras_));

    return cameras;
}

Image Database::ReadImage(const image_t image_id) const {
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_read_image_id_, 1, image_id));

    Image image;

    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_image_id_));
    if (rc == SQLITE_ROW) {
        image = ReadImageRow(sql_stmt_read_image_id_);
    }

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_image_id_));

    return image;
}

Image Database::ReadImageFromName(const std::string& name) const {
    SQLITE3_CALL(sqlite3_bind_text(
            sql_stmt_read_image_name_,
            1,
            name.c_str(),
            static_cast<int>(name.size()), SQLITE_STATIC)
    );

    Image image;

    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_image_name_));
    if (rc == SQLITE_ROW) {
        image = ReadImageRow(sql_stmt_read_image_name_);
    }

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_image_name_));

    return image;
}

std::vector<Image> Database::ReadAllImages() const {
    std::vector<Image> images;
    images.reserve(NumImages());

    while (SQLITE3_CALL(sqlite3_step(sql_stmt_read_images_)) == SQLITE_ROW) {
        images.push_back(ReadImageRow(sql_stmt_read_images_));
    }

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_images_));

    return images;
}

FeatureKeypoints Database::ReadKeypoints(const image_t image_id) const {
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_read_keypoints_, 1, image_id));

    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_keypoints_));
    const FeatureKeypointsBlob blob = ReadMatrixBlob<FeatureKeypointsBlob>(sql_stmt_read_keypoints_, rc, 0);

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_keypoints_));

    return FeatureKeypointsFromBlob(blob);
}

FeatureDescriptors Database::ReadDescriptors(const image_t image_id) const {
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_read_descriptors_, 1, image_id));

    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_descriptors_));
    const FeatureDescriptors descriptors = ReadMatrixBlob<FeatureDescriptors>(sql_stmt_read_descriptors_, rc, 0);

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_descriptors_));

    return descriptors;
}

FeatureMatches Database::ReadMatches(image_t image_id1, image_t image_id2) const {

    const image_pair_t pair_id = ImagePairToPairId(image_id1, image_id2);
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_read_matches_, 1, pair_id));

    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_matches_));
    FeatureMatchesBlob blob = ReadMatrixBlob<FeatureMatchesBlob>(sql_stmt_read_matches_, rc, 0);

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_matches_));

    if (SwapImagePair(image_id1, image_id2)) {
        SwapFeatureMatchesBlob(&blob);
    }

    return FeatureMatchesFromBlob(blob);
}

std::vector<std::pair<image_pair_t, FeatureMatches>> Database::ReadAllMatches() const {
    std::vector<std::pair<image_pair_t, FeatureMatches>> all_matches;

    int rc;
    while ((rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_matches_all_))) == SQLITE_ROW) {
        const image_pair_t pair_id = static_cast<image_pair_t>(sqlite3_column_int64(sql_stmt_read_matches_all_, 0));
        const FeatureMatchesBlob blob = ReadMatrixBlob<FeatureMatchesBlob>(sql_stmt_read_matches_all_, rc, 1);
        all_matches.emplace_back(pair_id, FeatureMatchesFromBlob(blob));
    }

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_matches_all_));

    return all_matches;
}

TwoViewGeometry Database::ReadInlierMatches(const image_t image_id1, const image_t image_id2) const {
    const image_pair_t pair_id = ImagePairToPairId(image_id1, image_id2);
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_read_inlier_matches_, 1, pair_id));

    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_inlier_matches_));

    TwoViewGeometry two_view_geometry;

    FeatureMatchesBlob blob = ReadMatrixBlob<FeatureMatchesBlob>(sql_stmt_read_inlier_matches_, rc, 0);

    two_view_geometry.config = static_cast<int>(sqlite3_column_int64(sql_stmt_read_inlier_matches_, 3));

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_inlier_matches_));

    if (SwapImagePair(image_id1, image_id2)) {
        SwapFeatureMatchesBlob(&blob);
    }

    two_view_geometry.inlier_matches = FeatureMatchesFromBlob(blob);

    return two_view_geometry;
}

std::vector<std::pair<image_pair_t, TwoViewGeometry>>Database::ReadAllInlierMatches() const {

    std::vector<std::pair<image_pair_t, TwoViewGeometry>> results;

    int rc;
    while ((rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_inlier_matches_all_))) == SQLITE_ROW) {
        const image_pair_t pair_id = static_cast<image_pair_t>(
                sqlite3_column_int64(sql_stmt_read_inlier_matches_all_, 0)
        );

        TwoViewGeometry two_view_geometry;
        const FeatureMatchesBlob blob = ReadMatrixBlob<FeatureMatchesBlob>(sql_stmt_read_inlier_matches_all_, rc, 1);
        two_view_geometry.config = static_cast<int>(sqlite3_column_int64(sql_stmt_read_inlier_matches_all_, 4));

        two_view_geometry.inlier_matches = FeatureMatchesFromBlob(blob);

        results.emplace_back(pair_id, two_view_geometry);
    }

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_inlier_matches_all_));

    return results;
}

void Database::ReadInlierMatchesGraph(
        std::vector<std::pair<image_t, image_t>>* image_pairs,
        std::vector<int>* num_inliers
) const {

    const auto num_inlier_matches = NumInlierMatches();
    image_pairs->reserve(num_inlier_matches);
    num_inliers->reserve(num_inlier_matches);

    int rc;
    while ((rc = SQLITE3_CALL(sqlite3_step(sql_stmt_read_inlier_matches_graph_))) == SQLITE_ROW) {
        image_t image_id1;
        image_t image_id2;
        const image_pair_t pair_id = static_cast<image_pair_t>(
                sqlite3_column_int64(sql_stmt_read_inlier_matches_graph_, 0)
        );
        PairIdToImagePair(pair_id, &image_id1, &image_id2);
        image_pairs->emplace_back(image_id1, image_id2);

        const int rows = static_cast<int>(sqlite3_column_int64(sql_stmt_read_inlier_matches_graph_, 1));
        num_inliers->push_back(rows);
    }

    SQLITE3_CALL(sqlite3_reset(sql_stmt_read_inlier_matches_graph_));
}

camera_t Database::WriteCamera(const Camera& camera, const bool use_camera_id) const {
    if (use_camera_id) {
        SQLITE3_CALL(
                sqlite3_bind_int64(sql_stmt_add_camera_, 1, camera.CameraId()));
    } else {
        SQLITE3_CALL(sqlite3_bind_null(sql_stmt_add_camera_, 1));
    }

    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_add_camera_, 2, camera.ModelId()));
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_add_camera_, 3, static_cast<sqlite3_int64>(camera.Width())));
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_add_camera_, 4, static_cast<sqlite3_int64>(camera.Height())));

    // What are the camera parameters? Database doesn't care about that.
    // It writes (and in the other place - reads) just a vector of doubles which is params_ in class Camera.
    // Seems like these values are:
    // "fx, fy, cx, cy" for pinhole camera,
    // "f, cx, cy, k1, k2" for radial camera.
    const size_t num_params_bytes = sizeof(double) * camera.NumParams();
    SQLITE3_CALL(sqlite3_bind_blob(sql_stmt_add_camera_,
                                   5,
                                   camera.ParamsData(),
                                   static_cast<int>(num_params_bytes),
                                   SQLITE_STATIC)
    );

    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_add_camera_, 6, camera.HasPriorFocalLength()));

    SQLITE3_CALL(sqlite3_step(sql_stmt_add_camera_));
    SQLITE3_CALL(sqlite3_reset(sql_stmt_add_camera_));

    return static_cast<camera_t>(sqlite3_last_insert_rowid(database_));
}

image_t Database::WriteImage(const Image& image, const bool use_image_id) const {
    if (use_image_id) {
        SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_add_image_, 1, image.ImageId()));
    } else {
        SQLITE3_CALL(sqlite3_bind_null(sql_stmt_add_image_, 1));
    }

    SQLITE3_CALL(sqlite3_bind_text(
            sql_stmt_add_image_,
            2,
            image.Name().c_str(),
            static_cast<int>(image.Name().size()),
            SQLITE_STATIC)
    );
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_add_image_, 3, image.CameraId()));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_add_image_, 4, image.QvecPrior(0)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_add_image_, 5, image.QvecPrior(1)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_add_image_, 6, image.QvecPrior(2)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_add_image_, 7, image.QvecPrior(3)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_add_image_, 8, image.TvecPrior(0)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_add_image_, 9, image.TvecPrior(1)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_add_image_, 10, image.TvecPrior(2)));

    SQLITE3_CALL(sqlite3_step(sql_stmt_add_image_));
    SQLITE3_CALL(sqlite3_reset(sql_stmt_add_image_));

    return static_cast<image_t>(sqlite3_last_insert_rowid(database_));
}

void Database::WriteKeypoints(const image_t image_id, const FeatureKeypoints& keypoints) const {
    const FeatureKeypointsBlob blob = FeatureKeypointsToBlob(keypoints);

    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_write_keypoints_, 1, image_id));
    WriteMatrixBlob(sql_stmt_write_keypoints_, blob, 2);

    SQLITE3_CALL(sqlite3_step(sql_stmt_write_keypoints_));
    SQLITE3_CALL(sqlite3_reset(sql_stmt_write_keypoints_));
}

void Database::WriteDescriptors(const image_t image_id, const FeatureDescriptors& descriptors) const {
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_write_descriptors_, 1, image_id));
    WriteMatrixBlob(sql_stmt_write_descriptors_, descriptors, 2);

    SQLITE3_CALL(sqlite3_step(sql_stmt_write_descriptors_));
    SQLITE3_CALL(sqlite3_reset(sql_stmt_write_descriptors_));
}

void Database::WriteMatches(const image_t image_id1, const image_t image_id2, const FeatureMatches& matches) const {
    const image_pair_t pair_id = ImagePairToPairId(image_id1, image_id2);
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_write_matches_, 1, pair_id));

    FeatureMatchesBlob blob = FeatureMatchesToBlob(matches);
    if (SwapImagePair(image_id1, image_id2)) {
        SwapFeatureMatchesBlob(&blob);
        WriteMatrixBlob(sql_stmt_write_matches_, blob, 2);
    } else {
        WriteMatrixBlob(sql_stmt_write_matches_, blob, 2);
    }

    SQLITE3_CALL(sqlite3_step(sql_stmt_write_matches_));
    SQLITE3_CALL(sqlite3_reset(sql_stmt_write_matches_));
}

void Database::WriteInlierMatches(
        const image_t image_id1,
        const image_t image_id2,
        const TwoViewGeometry& two_view_geometry
) const {
    const image_pair_t pair_id = ImagePairToPairId(image_id1, image_id2);
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_write_inlier_matches_, 1, pair_id));

    FeatureMatchesBlob blob = FeatureMatchesToBlob(two_view_geometry.inlier_matches);
    if (SwapImagePair(image_id1, image_id2)) {
        SwapFeatureMatchesBlob(&blob);
    }

    WriteMatrixBlob(sql_stmt_write_inlier_matches_, blob, 2);

    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_write_inlier_matches_, 5, two_view_geometry.config));

    SQLITE3_CALL(sqlite3_step(sql_stmt_write_inlier_matches_));
    SQLITE3_CALL(sqlite3_reset(sql_stmt_write_inlier_matches_));
}

void Database::UpdateCamera(const Camera& camera) {
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_update_camera_, 1, camera.ModelId()));
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_update_camera_, 2, static_cast<sqlite3_int64>(camera.Width())));
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_update_camera_, 3, static_cast<sqlite3_int64>(camera.Height())));

    const size_t num_params_bytes = sizeof(double) * camera.NumParams();
    SQLITE3_CALL(sqlite3_bind_blob(
            sql_stmt_update_camera_,
            4,
            camera.ParamsData(),
            static_cast<int>(num_params_bytes),
            SQLITE_STATIC)
    );

    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_update_camera_, 5, camera.HasPriorFocalLength()));

    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_update_camera_, 6, camera.CameraId()));

    SQLITE3_CALL(sqlite3_step(sql_stmt_update_camera_));
    SQLITE3_CALL(sqlite3_reset(sql_stmt_update_camera_));
}

void Database::UpdateImage(const Image& image) {
    SQLITE3_CALL(sqlite3_bind_text(
            sql_stmt_update_image_,
            1,
            image.Name().c_str(),
            static_cast<int>(image.Name().size()), SQLITE_STATIC)
    );
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_update_image_, 2, image.CameraId()));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_update_image_, 3, image.QvecPrior(0)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_update_image_, 4, image.QvecPrior(1)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_update_image_, 5, image.QvecPrior(2)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_update_image_, 6, image.QvecPrior(3)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_update_image_, 7, image.TvecPrior(0)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_update_image_, 8, image.TvecPrior(1)));
    SQLITE3_CALL(sqlite3_bind_double(sql_stmt_update_image_, 9, image.TvecPrior(2)));

    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt_update_image_, 10, image.ImageId()));

    SQLITE3_CALL(sqlite3_step(sql_stmt_update_image_));
    SQLITE3_CALL(sqlite3_reset(sql_stmt_update_image_));
}

void Database::PrepareSQLStatements() {
    sql_stmts_.clear();

    std::string sql;

    sql = "SELECT 1 FROM cameras WHERE camera_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_exists_camera_, 0));
    sql_stmts_.push_back(sql_stmt_exists_camera_);

    sql = "SELECT 1 FROM images WHERE image_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_exists_image_id_, 0));
    sql_stmts_.push_back(sql_stmt_exists_image_id_);

    sql = "SELECT 1 FROM images WHERE name = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_exists_image_name_, 0));
    sql_stmts_.push_back(sql_stmt_exists_image_name_);

    sql = "SELECT 1 FROM keypoints WHERE image_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_exists_keypoints_, 0));
    sql_stmts_.push_back(sql_stmt_exists_keypoints_);

    sql = "SELECT 1 FROM descriptors WHERE image_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_exists_descriptors_, 0));
    sql_stmts_.push_back(sql_stmt_exists_descriptors_);

    sql = "SELECT 1 FROM matches WHERE pair_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_exists_matches_, 0));
    sql_stmts_.push_back(sql_stmt_exists_matches_);

    sql = "SELECT 1 FROM inlier_matches WHERE pair_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_exists_inlier_matches_, 0));
    sql_stmts_.push_back(sql_stmt_exists_inlier_matches_);

    sql = "INSERT INTO cameras(camera_id, model, width, height, params, prior_focal_length) VALUES(?, ?, ?, ?, ?, ?);";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_add_camera_, 0));
    sql_stmts_.push_back(sql_stmt_add_camera_);

    sql = "INSERT INTO "
            "images(image_id, name, camera_id, prior_qw, prior_qx, prior_qy, prior_qz, prior_tx, prior_ty, prior_tz) "
            "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_add_image_, 0));
    sql_stmts_.push_back(sql_stmt_add_image_);

    sql = "UPDATE cameras SET model=?, width=?, height=?, params=?, prior_focal_length=? WHERE camera_id=?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_update_camera_, 0));
    sql_stmts_.push_back(sql_stmt_update_camera_);

    sql = "UPDATE images SET name=?, camera_id=?, prior_qw=?, prior_qx=?, "
            "prior_qy=?, prior_qz=?, prior_tx=?, prior_ty=?, prior_tz=? "
            "WHERE image_id=?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_update_image_, 0));
    sql_stmts_.push_back(sql_stmt_update_image_);

    sql = "SELECT * FROM cameras WHERE camera_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_camera_, 0));
    sql_stmts_.push_back(sql_stmt_read_camera_);

    sql = "SELECT * FROM cameras;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_cameras_, 0));
    sql_stmts_.push_back(sql_stmt_read_cameras_);

    sql = "SELECT * FROM images WHERE image_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_image_id_, 0));
    sql_stmts_.push_back(sql_stmt_read_image_id_);

    sql = "SELECT * FROM images WHERE name = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_image_name_, 0));
    sql_stmts_.push_back(sql_stmt_read_image_name_);

    sql = "SELECT * FROM images;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_images_, 0));
    sql_stmts_.push_back(sql_stmt_read_images_);

    sql = "SELECT rows, cols, data FROM keypoints WHERE image_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_keypoints_, 0));
    sql_stmts_.push_back(sql_stmt_read_keypoints_);

    sql = "SELECT rows, cols, data FROM descriptors WHERE image_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_descriptors_, 0));
    sql_stmts_.push_back(sql_stmt_read_descriptors_);

    sql = "SELECT rows, cols, data FROM matches WHERE pair_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_matches_, 0));
    sql_stmts_.push_back(sql_stmt_read_matches_);

    sql = "SELECT * FROM matches WHERE rows > 0;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_matches_all_, 0));
    sql_stmts_.push_back(sql_stmt_read_matches_all_);

    sql = "SELECT rows, cols, data, config FROM inlier_matches WHERE pair_id = ?;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_inlier_matches_, 0));
    sql_stmts_.push_back(sql_stmt_read_inlier_matches_);

    sql = "SELECT * FROM inlier_matches WHERE rows > 0;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_inlier_matches_all_, 0));
    sql_stmts_.push_back(sql_stmt_read_inlier_matches_all_);

    sql = "SELECT pair_id, rows FROM inlier_matches WHERE rows > 0;";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_read_inlier_matches_graph_, 0));
    sql_stmts_.push_back(sql_stmt_read_inlier_matches_graph_);

    sql = "INSERT INTO keypoints(image_id, rows, cols, data) VALUES(?, ?, ?, ?);";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_write_keypoints_, 0));
    sql_stmts_.push_back(sql_stmt_write_keypoints_);

    sql = "INSERT INTO descriptors(image_id, rows, cols, data) VALUES(?, ?, ?, ?);";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_write_descriptors_, 0));
    sql_stmts_.push_back(sql_stmt_write_descriptors_);

    sql = "INSERT INTO matches(pair_id, rows, cols, data) VALUES(?, ?, ?, ?);";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_write_matches_, 0));
    sql_stmts_.push_back(sql_stmt_write_matches_);

    sql = "INSERT INTO inlier_matches(pair_id, rows, cols, data, config) VALUES(?, ?, ?, ?, ?);";
    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt_write_inlier_matches_, 0));
    sql_stmts_.push_back(sql_stmt_write_inlier_matches_);
}

void Database::FinalizeSQLStatements() {
    for (const auto& sql_stmt : sql_stmts_) {
        SQLITE3_CALL(sqlite3_finalize(sql_stmt));
    }
}

void Database::CreateTables() const {
    CreateCameraTable();
    CreateImageTable();
    CreateKeypointsTable();
    CreateDescriptorsTable();
    CreateMatchesTable();
    CreateInlierMatchesTable();
}

void Database::CreateCameraTable() const {
    const std::string sql = "CREATE TABLE IF NOT EXISTS cameras"
            "   (camera_id            INTEGER  PRIMARY KEY AUTOINCREMENT  NOT NULL,"
            "    model                INTEGER                             NOT NULL,"
            "    width                INTEGER                             NOT NULL,"
            "    height               INTEGER                             NOT NULL,"
            "    params               BLOB,"
            "    prior_focal_length   INTEGER                             NOT NULL);";

    SQLITE3_EXEC(database_, sql.c_str(), nullptr);
}

void Database::CreateImageTable() const {
    const std::string sql = (QString().sprintf(
            "CREATE TABLE IF NOT EXISTS images"
                    "   (image_id   INTEGER  PRIMARY KEY AUTOINCREMENT  NOT NULL,"
                    "    name       TEXT                                NOT NULL UNIQUE,"
                    "    camera_id  INTEGER                             NOT NULL,"
                    "    prior_qw   REAL,"
                    "    prior_qx   REAL,"
                    "    prior_qy   REAL,"
                    "    prior_qz   REAL,"
                    "    prior_tx   REAL,"
                    "    prior_ty   REAL,"
                    "    prior_tz   REAL,"
                    "CONSTRAINT image_id_check CHECK(image_id >= 0 and image_id < %lu),"
                    "FOREIGN KEY(camera_id) REFERENCES cameras(camera_id));"
                    "CREATE UNIQUE INDEX IF NOT EXISTS index_name ON images(name);", kMaxNumImages).toStdString());

    SQLITE3_EXEC(database_, sql.c_str(), nullptr);
}

void Database::CreateKeypointsTable() const {
    const std::string sql = "CREATE TABLE IF NOT EXISTS keypoints"
            "   (image_id  INTEGER  PRIMARY KEY  NOT NULL,"
            "    rows      INTEGER               NOT NULL,"
            "    cols      INTEGER               NOT NULL,"
            "    data      BLOB,"
            "FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE);";

    SQLITE3_EXEC(database_, sql.c_str(), nullptr);
}

void Database::CreateDescriptorsTable() const {
    const std::string sql = "CREATE TABLE IF NOT EXISTS descriptors"
            "   (image_id  INTEGER  PRIMARY KEY  NOT NULL,"
            "    rows      INTEGER               NOT NULL,"
            "    cols      INTEGER               NOT NULL,"
            "    data      BLOB,"
            "FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE);";

    SQLITE3_EXEC(database_, sql.c_str(), nullptr);
}

void Database::CreateMatchesTable() const {
    const std::string sql = "CREATE TABLE IF NOT EXISTS matches"
            "   (pair_id  INTEGER  PRIMARY KEY  NOT NULL,"
            "    rows     INTEGER               NOT NULL,"
            "    cols     INTEGER               NOT NULL,"
            "    data     BLOB);";

    SQLITE3_EXEC(database_, sql.c_str(), nullptr);
}

void Database::CreateInlierMatchesTable() const {
    const std::string sql = "CREATE TABLE IF NOT EXISTS inlier_matches"
            "   (pair_id  INTEGER  PRIMARY KEY  NOT NULL,"
            "    rows     INTEGER               NOT NULL,"
            "    cols     INTEGER               NOT NULL,"
            "    data     BLOB,"
            "    config   INTEGER               NOT NULL);";

    SQLITE3_EXEC(database_, sql.c_str(), nullptr);
}

void Database::UpdateSchema() const {
    const std::string query_user_version_sql = "PRAGMA user_version;";
    sqlite3_stmt* query_user_version_sql_stmt;
    SQLITE3_CALL(sqlite3_prepare_v2(database_, query_user_version_sql.c_str(), -1, &query_user_version_sql_stmt, 0));

    if (SQLITE3_CALL(sqlite3_step(query_user_version_sql_stmt)) == SQLITE_ROW) {
        const int user_version = sqlite3_column_int(query_user_version_sql_stmt, 0);
        if (user_version > 0) {
        }
    }

    SQLITE3_CALL(sqlite3_finalize(query_user_version_sql_stmt));

    const std::string update_user_version_sql = "PRAGMA user_version = " + std::to_string(kSchemaVersion) + ";";
    SQLITE3_EXEC(database_, update_user_version_sql.c_str(), nullptr);
}

bool Database::ExistsRowId(sqlite3_stmt* sql_stmt, const size_t row_id) const {
    SQLITE3_CALL(sqlite3_bind_int64(sql_stmt, 1, static_cast<sqlite3_int64>(row_id)));

    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt));

    const bool exists = rc == SQLITE_ROW;

    SQLITE3_CALL(sqlite3_reset(sql_stmt));

    return exists;
}

bool Database::ExistsRowString(sqlite3_stmt* sql_stmt, const std::string& row_entry) const {
    SQLITE3_CALL(sqlite3_bind_text(sql_stmt, 1, row_entry.c_str(), static_cast<int>(row_entry.size()), SQLITE_STATIC));

    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt));

    const bool exists = rc == SQLITE_ROW;

    SQLITE3_CALL(sqlite3_reset(sql_stmt));

    return exists;
}

size_t Database::CountRows(const std::string& table) const {
    const std::string sql = "SELECT COUNT(*) FROM " + table + ";";
    sqlite3_stmt* sql_stmt;

    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt, 0));

    size_t count = 0;
    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt));
    if (rc == SQLITE_ROW) {
        count = static_cast<size_t>(sqlite3_column_int64(sql_stmt, 0));
    }

    SQLITE3_CALL(sqlite3_finalize(sql_stmt));

    return count;
}

size_t Database::SumColumn(const std::string& column, const std::string& table) const {
    const std::string sql = "SELECT SUM(" + column + ") FROM " + table + ";";
    sqlite3_stmt* sql_stmt;

    SQLITE3_CALL(sqlite3_prepare_v2(database_, sql.c_str(), -1, &sql_stmt, 0));

    size_t sum = 0;
    const int rc = SQLITE3_CALL(sqlite3_step(sql_stmt));
    if (rc == SQLITE_ROW) {
        sum = static_cast<size_t>(sqlite3_column_int64(sql_stmt, 0));
    }

    SQLITE3_CALL(sqlite3_finalize(sql_stmt));

    return sum;
}

// Giving an id to a pair of images to uniquely identify it.
image_pair_t Database::ImagePairToPairId(const image_t image_id1, const image_t image_id2) {
    if (SwapImagePair(image_id1, image_id2)) {
        return kMaxNumImages * image_id2 + image_id1;
    } else {
        return kMaxNumImages * image_id1 + image_id2;
    }
}

void Database::PairIdToImagePair(const image_pair_t pair_id, image_t* image_id1, image_t* image_id2) {
    *image_id2 = static_cast<image_t>(pair_id % kMaxNumImages);
    *image_id1 = static_cast<image_t>((pair_id - *image_id2) / kMaxNumImages);
}

bool Database::SwapImagePair(const image_t image_id1, const image_t image_id2) {
    return image_id1 > image_id2;
}


DatabaseCache::DatabaseCache() { }

void DatabaseCache::AddCamera(const class Camera& camera) {
    cameras_.emplace(camera.CameraId(), camera);
}

void DatabaseCache::AddImage(const class Image& image) {
    images_.emplace(image.ImageId(), image);
    scene_graph_.AddImage(image.ImageId(), image.NumPoints2D());
}

void DatabaseCache::Load(const Database& database, const size_t min_num_matches, const bool ignore_watermarks) {
    Timer timer;

    timer.Start();
    std::cout << "Loading cameras..." << std::flush;

    {
        const std::vector<class Camera> cameras = database.ReadAllCameras();
        cameras_.reserve(cameras.size());
        for (const class Camera& camera : cameras) {
            cameras_.emplace(camera.CameraId(), camera);
        }
    }
    std::cout << QString().sprintf(" %lu in %.3fs", cameras_.size(), timer.ElapsedSeconds()).toStdString() << std::endl;

    timer.Restart();
    std::cout << "Loading matches..." << std::flush;

    const std::vector<std::pair<image_pair_t, TwoViewGeometry>> image_pairs = database.ReadAllInlierMatches();

    std::cout << QString().sprintf(
            " %lu in %.3fs", image_pairs.size(), timer.ElapsedSeconds()
    ).toStdString() << std::endl;

    auto UseInlierMatchesCheck = [min_num_matches, ignore_watermarks](const TwoViewGeometry& two_view_geometry) {
        return (static_cast<size_t>(two_view_geometry.inlier_matches.size()) >= min_num_matches) &&
               (!ignore_watermarks || two_view_geometry.config != TwoViewGeometry::WATERMARK);
    };

    timer.Restart();
    std::cout << "Loading images..." << std::flush;

    {
        const std::vector<class Image> images = database.ReadAllImages();

        std::unordered_set<image_t> connected_image_ids;
        connected_image_ids.reserve(images.size());
        for (const auto& image_pair : image_pairs) {
            if (UseInlierMatchesCheck(image_pair.second)) {
                image_t image_id1;
                image_t image_id2;
                Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);
                connected_image_ids.insert(image_id1);
                connected_image_ids.insert(image_id2);
            }
        }

        images_.reserve(connected_image_ids.size());
        for (const class Image& image : images) {
            if (connected_image_ids.count(image.ImageId()) > 0) {
                images_.emplace(image.ImageId(), image);
                const FeatureKeypoints keypoints = database.ReadKeypoints(image.ImageId());
                const std::vector<Eigen::Vector2d> points = FeatureKeypointsToPointsVector(keypoints);
                images_[image.ImageId()].SetPoints2D(points);
                images_[image.ImageId()].SetDescriptors(database.ReadDescriptors(image.ImageId()));
            }
        }

        std::cout << QString().sprintf(
                " %lu in %.3fs (connected %lu)", images.size(), timer.ElapsedSeconds(), connected_image_ids.size()
        ).toStdString() << std::endl;
    }

    timer.Restart();
    std::cout << "Building scene graph..." << std::flush;

    for (const auto& image : images_) {
        scene_graph_.AddImage(image.first, image.second.NumPoints2D());
    }

    size_t num_ignored_image_pairs = 0;
    for (const auto& image_pair : image_pairs) {
        if (UseInlierMatchesCheck(image_pair.second)) {
            image_t image_id1;
            image_t image_id2;
            Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);
            scene_graph_.AddCorrespondences(image_id1, image_id2, image_pair.second.inlier_matches);
        } else {
            num_ignored_image_pairs += 1;
        }
    }

    scene_graph_.Finalize();

    for (auto& image : images_) {
        image.second.SetNumObservations(scene_graph_.NumObservationsForImage(image.first));
        image.second.SetNumCorrespondences(scene_graph_.NumCorrespondencesForImage(image.first));
    }

    std::cout << QString().sprintf(
            " in %.3fs (ignored %lu)", timer.ElapsedSeconds(), num_ignored_image_pairs
    ).toStdString() << std::endl;
}


size_t DatabaseCache::NumCameras() const { return cameras_.size(); }

size_t DatabaseCache::NumImages() const { return images_.size(); }

class Camera& DatabaseCache::Camera(const camera_t camera_id) { return cameras_.at(camera_id); }

const class Camera& DatabaseCache::Camera(const camera_t camera_id) const { return cameras_.at(camera_id); }

class Image& DatabaseCache::Image(const image_t image_id) { return images_.at(image_id); }

const class Image& DatabaseCache::Image(const image_t image_id) const { return images_.at(image_id); }

const std::unordered_map<camera_t, class Camera>& DatabaseCache::Cameras()
const { return cameras_; }

const std::unordered_map<image_t, class Image>& DatabaseCache::Images() const { return images_; }

bool DatabaseCache::ExistsCamera(const camera_t camera_id) const { return cameras_.count(camera_id) > 0; }

bool DatabaseCache::ExistsImage(const image_t image_id) const { return images_.count(image_id) > 0; }

const class SceneGraph& DatabaseCache::SceneGraph() const { return scene_graph_; }
