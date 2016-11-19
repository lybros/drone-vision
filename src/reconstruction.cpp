#include "reconstruction.h"

Reconstruction::Reconstruction()
        : scene_graph_(nullptr), num_added_points3D_(0) {}

std::unordered_set<point3D_t> Reconstruction::Point3DIds() const {
    std::unordered_set<point3D_t> point3D_ids;
    point3D_ids.reserve(points3D_.size());

    for (const auto& point3D : points3D_) {
        point3D_ids.insert(point3D.first);
    }

    return point3D_ids;
}

void Reconstruction::Load(const DatabaseCache& database_cache) {
    scene_graph_ = nullptr;

    cameras_.reserve(database_cache.NumCameras());
    for (const auto& camera : database_cache.Cameras()) {
        if (!ExistsCamera(camera.first)) {
            AddCamera(camera.second);
        }
    }

    images_.reserve(database_cache.NumImages());

    for (const auto& image : database_cache.Images()) {
        if (ExistsImage(image.second.ImageId())) {
            class Image& existing_image = Image(image.second.ImageId());
            existing_image.SetNumObservations(image.second.NumObservations());
            existing_image.SetNumCorrespondences(image.second.NumCorrespondences());
        } else {
            AddImage(image.second);
        }
    }

    for (const auto& image_pair : database_cache.SceneGraph().NumCorrespondencesBetweenImages()) {
        image_pairs_[image_pair.first] = std::make_pair(0, image_pair.second);
    }
}

void Reconstruction::SetUp(const SceneGraph* scene_graph) {
    for (auto& image : images_) {
        image.second.SetUp(Camera(image.second.CameraId()));
    }
    scene_graph_ = scene_graph;

    for (const auto image_id : reg_image_ids_) {
        const class Image& image = Image(image_id);
        for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D(); ++point2D_idx) {
            if (image.Point2D(point2D_idx).HasPoint3D()) {
                const bool kIsContinuedPoint3D = false;
                SetObservationAsTriangulated(image_id, point2D_idx, kIsContinuedPoint3D);
            }
        }
    }
}

void Reconstruction::TearDown() {
    scene_graph_ = nullptr;

    std::unordered_set<camera_t> keep_camera_ids;
    for (auto it = images_.begin(); it != images_.end();) {
        if (it->second.IsRegistered()) {
            keep_camera_ids.insert(it->second.CameraId());
            it->second.TearDown();
            ++it;
        } else {
            it = images_.erase(it);
        }
    }

    for (auto it = cameras_.begin(); it != cameras_.end();) {
        if (keep_camera_ids.count(it->first) == 0) {
            it = cameras_.erase(it);
        } else {
            ++it;
        }
    }

    for (auto& point3D : points3D_) {
        point3D.second.Track().Compress();
    }
}

void Reconstruction::AddCamera(const class Camera& camera) {
    cameras_.emplace(camera.CameraId(), camera);
}

void Reconstruction::AddImage(const class Image& image) {
    images_[image.ImageId()] = image;
}

point3D_t Reconstruction::AddPoint3D(const Eigen::Vector3d& xyz, const Track& track) {
    const point3D_t point3D_id = ++num_added_points3D_;

    class Point_3D& point3D = points3D_[point3D_id];

    point3D.SetXYZ(xyz);
    point3D.SetTrack(track);

    for (const auto& track_el : track.Elements()) {
        class Image& image = Image(track_el.image_id);
        image.SetPoint3DForPoint2D(track_el.point2D_idx, point3D_id);
    }

    const bool kIsContinuedPoint3D = false;

    for (const auto& track_el : track.Elements()) {
        SetObservationAsTriangulated(track_el.image_id, track_el.point2D_idx, kIsContinuedPoint3D);
    }

    return point3D_id;
}

void Reconstruction::AddObservation(const point3D_t point3D_id, const TrackElement& track_el) {
    class Image& image = Image(track_el.image_id);

    image.SetPoint3DForPoint2D(track_el.point2D_idx, point3D_id);

    class Point_3D& point3D = Point3D(point3D_id);
    point3D.Track().AddElement(track_el);

    const bool kIsContinuedPoint3D = true;
    SetObservationAsTriangulated(track_el.image_id, track_el.point2D_idx, kIsContinuedPoint3D);
}

point3D_t Reconstruction::MergePoints3D(const point3D_t point3D_id1, const point3D_t point3D_id2) {
    const class Point_3D& point3D1 = Point3D(point3D_id1);
    const class Point_3D& point3D2 = Point3D(point3D_id2);

    const Eigen::Vector3d merged_xyz =
            (point3D1.Track().Length() * point3D1.XYZ() +
             point3D2.Track().Length() * point3D2.XYZ()) /
            (point3D1.Track().Length() + point3D2.Track().Length());
    const Eigen::Vector3d merged_rgb =
            (point3D1.Track().Length() * point3D1.Color().cast<double>() +
             point3D2.Track().Length() * point3D2.Color().cast<double>()) /
            (point3D1.Track().Length() + point3D2.Track().Length());

    Track merged_track;
    merged_track.Reserve(point3D1.Track().Length() + point3D2.Track().Length());
    merged_track.AddElements(point3D1.Track().Elements());
    merged_track.AddElements(point3D2.Track().Elements());

    DeletePoint3D(point3D_id1);
    DeletePoint3D(point3D_id2);

    const point3D_t merged_point3D_id = AddPoint3D(merged_xyz, merged_track);
    class Point_3D& merged_point3D = Point3D(merged_point3D_id);
    merged_point3D.SetColor(merged_rgb.cast<uint8_t>());

    return merged_point3D_id;
}

void Reconstruction::DeletePoint3D(const point3D_t point3D_id) {
    const class Track& track = Point3D(point3D_id).Track();

    const bool kIsDeletedPoint3D = true;

    for (const auto& track_el : track.Elements()) {
        ResetTriObservations(track_el.image_id, track_el.point2D_idx, kIsDeletedPoint3D);
    }

    for (const auto& track_el : track.Elements()) {
        class Image& image = Image(track_el.image_id);
        image.ResetPoint3DForPoint2D(track_el.point2D_idx);
    }

    points3D_.erase(point3D_id);
}

void Reconstruction::DeleteObservation(const image_t image_id, const point2D_t point2D_idx) {
    class Image& image = Image(image_id);
    const point3D_t point3D_id = image.Point2D(point2D_idx).Point3DId();
    class Point_3D& point3D = Point3D(point3D_id);

    if (point3D.Track().Length() <= 2) {
        DeletePoint3D(point3D_id);
        return;
    }

    point3D.Track().DeleteElement(image_id, point2D_idx);

    const bool kIsDeletedPoint3D = false;
    ResetTriObservations(image_id, point2D_idx, kIsDeletedPoint3D);

    image.ResetPoint3DForPoint2D(point2D_idx);
}

void Reconstruction::RegisterImage(const image_t image_id) {
    class Image& image = Image(image_id);
    if (!image.IsRegistered()) {
        image.SetRegistered(true);
        reg_image_ids_.push_back(image_id);
    }
}

void Reconstruction::DeRegisterImage(const image_t image_id) {
    class Image& image = Image(image_id);

    for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D(); ++point2D_idx) {
        if (image.Point2D(point2D_idx).HasPoint3D()) {
            DeleteObservation(image_id, point2D_idx);
        }
    }

    image.SetRegistered(false);

    reg_image_ids_.erase(std::remove(reg_image_ids_.begin(), reg_image_ids_.end(), image_id), reg_image_ids_.end());
}

void Reconstruction::Normalize(const double extent, const double p0, const double p1, const bool use_images) {
    if (use_images && reg_image_ids_.size() < 2) {
        return;
    }

    std::unordered_map<class Image*, Eigen::Vector3d> proj_centers;

    for (size_t i = 0; i < reg_image_ids_.size(); ++i) {
        class Image& image = Image(reg_image_ids_[i]);
        const Eigen::Vector3d proj_center = image.ProjectionCenter();
        proj_centers[&image] = proj_center;
    }

    std::vector<float> coords_x;
    std::vector<float> coords_y;
    std::vector<float> coords_z;
    if (use_images) {
        coords_x.reserve(proj_centers.size());
        coords_y.reserve(proj_centers.size());
        coords_z.reserve(proj_centers.size());
        for (const auto& proj_center : proj_centers) {
            coords_x.push_back(static_cast<float>(proj_center.second(0)));
            coords_y.push_back(static_cast<float>(proj_center.second(1)));
            coords_z.push_back(static_cast<float>(proj_center.second(2)));
        }
    } else {
        coords_x.reserve(points3D_.size());
        coords_y.reserve(points3D_.size());
        coords_z.reserve(points3D_.size());
        for (const auto& point3D : points3D_) {
            coords_x.push_back(static_cast<float>(point3D.second.X()));
            coords_y.push_back(static_cast<float>(point3D.second.Y()));
            coords_z.push_back(static_cast<float>(point3D.second.Z()));
        }
    }

    std::sort(coords_x.begin(), coords_x.end());
    std::sort(coords_y.begin(), coords_y.end());
    std::sort(coords_z.begin(), coords_z.end());

    const size_t P0 = static_cast<size_t>(
            (coords_x.size() > 3) ? p0 * (coords_x.size() - 1) : 0);
    const size_t P1 = static_cast<size_t>(
            (coords_x.size() > 3) ? p1 * (coords_x.size() - 1) : coords_x.size() - 1);

    const Eigen::Vector3d bbox_min(coords_x[P0], coords_y[P0], coords_z[P0]);
    const Eigen::Vector3d bbox_max(coords_x[P1], coords_y[P1], coords_z[P1]);

    Eigen::Vector3d mean_coord(0, 0, 0);
    for (size_t i = P0; i <= P1; ++i) {
        mean_coord(0) += coords_x[i];
        mean_coord(1) += coords_y[i];
        mean_coord(2) += coords_z[i];
    }
    mean_coord /= P1 - P0 + 1;

    const double old_extent = (bbox_max - bbox_min).norm();
    double scale;
    if (old_extent < std::numeric_limits<double>::epsilon()) {
        scale = 1;
    } else {
        scale = extent / old_extent;
    }

    const Eigen::Vector3d translation = mean_coord;

    for (auto& elem : proj_centers) {
        elem.second -= translation;
        elem.second *= scale;
        const Eigen::Quaterniond quat(elem.first->Qvec(0), elem.first->Qvec(1),
                                      elem.first->Qvec(2), elem.first->Qvec(3));
        elem.first->SetTvec(quat * -elem.second);
    }

    for (auto& point3D : points3D_) {
        point3D.second.XYZ() -= translation;
        point3D.second.XYZ() *= scale;
    }
}

const class Image* Reconstruction::FindImageWithName(
        const std::string& name) const {
    for (const auto& elem : images_) {
        if (elem.second.Name() == name) {
            return &elem.second;
        }
    }
    return nullptr;
}

size_t Reconstruction::FilterPoints3D(
        const double max_reproj_error, const double min_tri_angle,
        const std::unordered_set<point3D_t>& point3D_ids) {
    size_t num_filtered = 0;
    num_filtered +=
            FilterPoints3DWithLargeReprojectionError(max_reproj_error, point3D_ids);
    num_filtered +=
            FilterPoints3DWithSmallTriangulationAngle(min_tri_angle, point3D_ids);
    return num_filtered;
}

size_t Reconstruction::FilterPoints3DInImages(
        const double max_reproj_error, const double min_tri_angle,
        const std::unordered_set<image_t>& image_ids) {
    std::unordered_set<point3D_t> point3D_ids;
    for (const image_t image_id : image_ids) {
        const class Image& image = Image(image_id);
        for (const Point2D& point2D : image.Points2D()) {
            if (point2D.HasPoint3D()) {
                point3D_ids.insert(point2D.Point3DId());
            }
        }
    }
    return FilterPoints3D(max_reproj_error, min_tri_angle, point3D_ids);
}

size_t Reconstruction::FilterAllPoints3D(const double max_reproj_error,
                                         const double min_tri_angle) {
    const std::unordered_set<point3D_t>& point3D_ids = Point3DIds();
    size_t num_filtered = 0;
    num_filtered += FilterPoints3DWithLargeReprojectionError(max_reproj_error, point3D_ids);
    num_filtered += FilterPoints3DWithSmallTriangulationAngle(min_tri_angle, point3D_ids);
    return num_filtered;
}

size_t Reconstruction::FilterObservationsWithNegativeDepth() {
    size_t num_filtered = 0;
    for (const auto image_id : reg_image_ids_) {
        const class Image& image = Image(image_id);
        const Eigen::Matrix3x4d proj_matrix = image.ProjectionMatrix();
        for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D(); ++point2D_idx) {
            const Point2D& point2D = image.Point2D(point2D_idx);
            if (point2D.HasPoint3D()) {
                const class Point_3D& point3D = Point3D(point2D.Point3DId());
                if (!HasPointPositiveDepth(proj_matrix, point3D.XYZ())) {
                    DeleteObservation(image_id, point2D_idx);
                    num_filtered += 1;
                }
            }
        }
    }
    return num_filtered;
}

std::vector<image_t> Reconstruction::FilterImages(
        const double min_focal_length_ratio, const double max_focal_length_ratio,
        const double max_extra_param) {
    std::vector<image_t> filtered_image_ids;
    for (const image_t image_id : RegImageIds()) {
        const class Image& image = Image(image_id);
        const class Camera& camera = Camera(image.CameraId());
        if (image.NumPoints3D() == 0) {
            DeRegisterImage(image_id);
            filtered_image_ids.push_back(image_id);
        } else if (camera.HasBogusParams(min_focal_length_ratio, max_focal_length_ratio, max_extra_param)) {
            filtered_image_ids.push_back(image_id);
        }
    }

    for (const image_t image_id : filtered_image_ids) {
        DeRegisterImage(image_id);
    }

    return filtered_image_ids;
}

size_t Reconstruction::ComputeNumObservations() const {
    size_t num_obs = 0;
    for (const image_t image_id : reg_image_ids_) {
        num_obs += Image(image_id).NumPoints3D();
    }
    return num_obs;
}

double Reconstruction::ComputeMeanTrackLength() const {
    if (points3D_.empty()) {
        return 0.0;
    } else {
        return ComputeNumObservations() / static_cast<double>(points3D_.size());
    }
}

double Reconstruction::ComputeMeanObservationsPerRegImage() const {
    if (reg_image_ids_.empty()) {
        return 0.0;
    } else {
        return ComputeNumObservations() /
               static_cast<double>(reg_image_ids_.size());
    }
}

double Reconstruction::ComputeMeanReprojectionError() const {
    double error_sum = 0.0;
    size_t num_valid_errors = 0;
    for (const auto& point3D : points3D_) {
        if (point3D.second.HasError()) {
            error_sum += point3D.second.Error();
            num_valid_errors += 1;
        }
    }

    if (num_valid_errors == 0) {
        return 0.0;
    } else {
        return error_sum / num_valid_errors;
    }
}

void Reconstruction::ImportPLY(const std::string& path, bool append_to_existing) {
    if (!append_to_existing)
        points3D_.clear();

    std::ifstream file(path.c_str());

    std::string line;

    int X_index = -1;
    int Y_index = -1;
    int Z_index = -1;
    int R_index = -1;
    int G_index = -1;
    int B_index = -1;
    int X_byte_pos = -1;
    int Y_byte_pos = -1;
    int Z_byte_pos = -1;
    int R_byte_pos = -1;
    int G_byte_pos = -1;
    int B_byte_pos = -1;

    bool in_vertex_section = false;
    bool is_binary = false;
    size_t num_bytes_per_line = 0;
    size_t num_vertices = 0;

    int index = 0;
    while (std::getline(file, line)) {
        boost::trim(line);

        if (line.empty()) {
            continue;
        }

        if (line == "end_header") {
            break;
        }

        if (line.size() >= 6 && line.substr(0, 6) == "format") {
            if (line == "format ascii 1.0") {
                is_binary = false;
            } else if (line == "format binary_little_endian 1.0" || line == "format binary_big_endian 1.0") {
                is_binary = true;
            }
        }

        const std::vector<std::string> line_elems = StringSplit(line, " ");

        if (line_elems.size() >= 3 && line_elems[0] == "element") {
            in_vertex_section = false;
            if (line_elems[1] == "vertex") {
                num_vertices = boost::lexical_cast<size_t>(line_elems[2]);
                in_vertex_section = true;
            } else if (boost::lexical_cast<size_t>(line_elems[2]) > 0) {
                std::cerr << "Only vertex elements supported";
            }
        }

        if (!in_vertex_section) {
            continue;
        }

        if (line_elems.size() >= 3 && line_elems[0] == "property") {
            if (line == "property float x") {
                X_index = index;
                X_byte_pos = num_bytes_per_line;
            } else if (line == "property float y") {
                Y_index = index;
                Y_byte_pos = num_bytes_per_line;
            } else if (line == "property float z") {
                Z_index = index;
                Z_byte_pos = num_bytes_per_line;
            } else if (line == "property uchar r" || line == "property uchar red" ||
                       line == "property uchar diffuse_red") {
                R_index = index;
                R_byte_pos = num_bytes_per_line;
            } else if (line == "property uchar g" || line == "property uchar green" ||
                       line == "property uchar diffuse_green") {
                G_index = index;
                G_byte_pos = num_bytes_per_line;
            } else if (line == "property uchar b" || line == "property uchar blue" ||
                       line == "property uchar diffuse_blue") {
                B_index = index;
                B_byte_pos = num_bytes_per_line;
            }

            index += 1;
            if (line_elems[1] == "float") {
                num_bytes_per_line += 4;
            } else if (line_elems[1] == "uchar") {
                num_bytes_per_line += 1;
            } else {
                std::cerr << "Invalid data type: " << line_elems[1];
            }
        }
    }

    if (is_binary) {
        std::vector<char> buffer(num_bytes_per_line);
        for (size_t i = 0; i < num_vertices; ++i) {
            file.read(buffer.data(), num_bytes_per_line);

            Eigen::Vector3d xyz;
            xyz(0) =
                    static_cast<double>(*reinterpret_cast<float*>(&buffer[X_byte_pos]));
            xyz(1) =
                    static_cast<double>(*reinterpret_cast<float*>(&buffer[Y_byte_pos]));
            xyz(2) =
                    static_cast<double>(*reinterpret_cast<float*>(&buffer[Z_byte_pos]));

            Eigen::Vector3i rgb;
            rgb(0) = *reinterpret_cast<uint8_t*>(&buffer[R_byte_pos]);
            rgb(1) = *reinterpret_cast<uint8_t*>(&buffer[G_byte_pos]);
            rgb(2) = *reinterpret_cast<uint8_t*>(&buffer[B_byte_pos]);

            const point3D_t point3D_id = AddPoint3D(xyz, Track());
            Point3D(point3D_id).SetColor(rgb.cast<uint8_t>());
        }
    } else {
        while (std::getline(file, line)) {
            boost::trim(line);
            std::stringstream line_stream(line);

            std::string item;
            std::vector<std::string> items;
            while (!line_stream.eof()) {
                std::getline(line_stream, item, ' ');
                boost::trim(item);
                items.push_back(item);
            }

            Eigen::Vector3d xyz;
            xyz(0) = boost::lexical_cast<double>(items.at(X_index));
            xyz(1) = boost::lexical_cast<double>(items.at(Y_index));
            xyz(2) = boost::lexical_cast<double>(items.at(Z_index));

            Eigen::Vector3i rgb;
            rgb(0) = boost::lexical_cast<int>(items.at(R_index));
            rgb(1) = boost::lexical_cast<int>(items.at(G_index));
            rgb(2) = boost::lexical_cast<int>(items.at(B_index));

            const point3D_t point3D_id = AddPoint3D(xyz, Track());
            Point3D(point3D_id).SetColor(rgb.cast<uint8_t>());
        }
    }

    file.close();
}


void Reconstruction::ExportBundler(const std::string& path, const std::string& list_path) const {
    std::ofstream file;
    file.open(path.c_str(), std::ios::trunc);

    std::ofstream list_file;
    list_file.open(list_path.c_str(), std::ios::trunc);

    file << "# Bundle file v0.3" << std::endl;

    file << reg_image_ids_.size() << " " << points3D_.size() << std::endl;

    std::unordered_map<image_t, size_t> image_id_to_idx_;
    size_t image_idx = 0;

    for (const image_t image_id : reg_image_ids_) {
        const class Image& image = Image(image_id);
        const class Camera& camera = Camera(image.CameraId());

        double f;
        double k1;
        double k2;
        if (camera.ModelId() == PinholeCameraModel::model_id) {
            f = camera.MeanFocalLength();
            k1 = 0.0;
            k2 = 0.0;
        } else if (camera.ModelId() == RadialCameraModel::model_id) {
            f = camera.Params(RadialCameraModel::focal_length_idxs[0]);
            k1 = camera.Params(RadialCameraModel::extra_params_idxs[0]);
            k2 = camera.Params(RadialCameraModel::extra_params_idxs[1]);
        } else {
            throw std::domain_error(
                    "Bundler only supports `SIMPLE_RADIAL` or "
                            "`RADIAL` camera model");
        }

        file << f << " " << k1 << " " << k2 << std::endl;

        const Eigen::Matrix3d R = image.RotationMatrix();
        file << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << std::endl;
        file << -R(1, 0) << " " << -R(1, 1) << " " << -R(1, 2) << std::endl;
        file << -R(2, 0) << " " << -R(2, 1) << " " << -R(2, 2) << std::endl;

        file << image.Tvec(0) << " ";
        file << -image.Tvec(1) << " ";
        file << -image.Tvec(2) << std::endl;

        list_file << image.Name() << std::endl;

        image_id_to_idx_[image_id] = image_idx;
        image_idx += 1;
    }

    for (const auto& point3D : points3D_) {
        file << point3D.second.XYZ()(0) << " ";
        file << point3D.second.XYZ()(1) << " ";
        file << point3D.second.XYZ()(2) << std::endl;

        file << static_cast<int>(point3D.second.Color(0)) << " ";
        file << static_cast<int>(point3D.second.Color(1)) << " ";
        file << static_cast<int>(point3D.second.Color(2)) << std::endl;

        std::ostringstream line;

        line << point3D.second.Track().Length() << " ";

        for (const auto& track_el : point3D.second.Track().Elements()) {
            const class Image& image = Image(track_el.image_id);
            const class Camera& camera = Camera(image.CameraId());

            const Point2D& point2D = image.Point2D(track_el.point2D_idx);

            line << image_id_to_idx_[track_el.image_id] << " ";
            line << track_el.point2D_idx << " ";
            line << point2D.X() - camera.PrincipalPointX() << " ";
            line << camera.PrincipalPointY() - point2D.Y() << " ";
        }

        std::string line_string = line.str();
        line_string = line_string.substr(0, line_string.size() - 1);

        file << line_string << std::endl;
    }

    file.close();
    list_file.close();
}


void Reconstruction::ExportPLY(const std::string& path) const {
    std::ofstream file;
    file.open(path.c_str(), std::ios::trunc);

    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << points3D_.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "end_header" << std::endl;

    for (const auto& point3D : points3D_) {
        file << point3D.second.X() << " ";
        file << point3D.second.Y() << " ";
        file << point3D.second.Z() << " ";
        file << static_cast<int>(point3D.second.Color(0)) << " ";
        file << static_cast<int>(point3D.second.Color(1)) << " ";
        file << static_cast<int>(point3D.second.Color(2)) << std::endl;
    }

    file << std::endl;

    file.close();
}

bool Reconstruction::ExtractColors(const image_t image_id, const std::string& path) {
    const class Image& image = Image(image_id);

    Bitmap bitmap;
    if (!bitmap.Read(EnsureTrailingSlash(path) + image.Name())) {
        return false;
    }

    const Eigen::Vector3ub kBlackColor(0, 0, 0);
    for (const Point2D point2D : image.Points2D()) {
        if (point2D.HasPoint3D()) {
            class Point_3D& point3D = Point3D(point2D.Point3DId());
            if (point3D.Color() == kBlackColor) {
                Eigen::Vector3d color;
                if (bitmap.InterpolateBilinear(point2D.X(), point2D.Y(), &color)) {
                    color.unaryExpr(std::ptr_fun<double, double>(std::round));
                    point3D.SetColor(color.cast<uint8_t>());
                }
            }
        }
    }

    return true;
}

size_t Reconstruction::FilterPoints3DWithSmallTriangulationAngle(
        const double min_tri_angle,
        const std::unordered_set<point3D_t>& point3D_ids) {
    size_t num_filtered = 0;

    const double min_tri_angle_rad = DegToRad(min_tri_angle);

    std::unordered_map<image_t, Eigen::Vector3d> proj_centers;

    for (const auto point3D_id : point3D_ids) {
        if (!ExistsPoint3D(point3D_id)) {
            continue;
        }

        const class Point_3D& point3D = Point3D(point3D_id);

        bool keep_point = false;
        for (size_t i1 = 0; i1 < point3D.Track().Length(); ++i1) {
            const image_t image_id1 = point3D.Track().Element(i1).image_id;

            Eigen::Vector3d proj_center1;
            if (proj_centers.count(image_id1) == 0) {
                const class Image& image1 = Image(image_id1);
                proj_center1 = image1.ProjectionCenter();
                proj_centers.emplace(image_id1, proj_center1);
            } else {
                proj_center1 = proj_centers.at(image_id1);
            }

            for (size_t i2 = 0; i2 < i1; ++i2) {
                const image_t image_id2 = point3D.Track().Element(i2).image_id;
                const Eigen::Vector3d proj_center2 = proj_centers.at(image_id2);

                const double tri_angle = CalculateTriangulationAngle(proj_center1, proj_center2, point3D.XYZ());

                if (tri_angle >= min_tri_angle_rad) {
                    keep_point = true;
                    break;
                }
            }

            if (keep_point) {
                break;
            }
        }

        if (!keep_point) {
            num_filtered += 1;
            DeletePoint3D(point3D_id);
        }
    }

    return num_filtered;
}

size_t Reconstruction::FilterPoints3DWithLargeReprojectionError(
        const double max_reproj_error,
        const std::unordered_set<point3D_t>& point3D_ids) {
    size_t num_filtered = 0;

    std::unordered_map<image_t, Eigen::Matrix3x4d> proj_matrices;

    for (const auto point3D_id : point3D_ids) {
        if (!ExistsPoint3D(point3D_id)) {
            continue;
        }

        class Point_3D& point3D = Point3D(point3D_id);

        if (point3D.Track().Length() < 2) {
            DeletePoint3D(point3D_id);
            continue;
        }

        double reproj_error_sum = 0.0;

        std::vector<TrackElement> track_els_to_delete;

        for (const auto& track_el : point3D.Track().Elements()) {
            const class Image& image = Image(track_el.image_id);

            Eigen::Matrix3x4d proj_matrix;
            if (proj_matrices.count(track_el.image_id) == 0) {
                proj_matrix = image.ProjectionMatrix();
                proj_matrices[track_el.image_id] = proj_matrix;
            } else {
                proj_matrix = proj_matrices[track_el.image_id];
            }

            if (HasPointPositiveDepth(proj_matrix, point3D.XYZ())) {
                const class Camera& camera = Camera(image.CameraId());
                const Point2D& point2D = image.Point2D(track_el.point2D_idx);
                const double reproj_error = CalculateReprojectionError(
                        point2D.XY(), point3D.XYZ(), proj_matrix, camera);
                if (reproj_error > max_reproj_error) {
                    track_els_to_delete.push_back(track_el);
                } else {
                    reproj_error_sum += reproj_error;
                }
            } else {
                track_els_to_delete.push_back(track_el);
            }
        }

        if (track_els_to_delete.size() == point3D.Track().Length() ||
            track_els_to_delete.size() == point3D.Track().Length() - 1) {
            num_filtered += point3D.Track().Length();
            DeletePoint3D(point3D_id);
        } else {
            num_filtered += track_els_to_delete.size();
            for (const auto& track_el : track_els_to_delete) {
                DeleteObservation(track_el.image_id, track_el.point2D_idx);
            }
            point3D.SetError(reproj_error_sum / point3D.Track().Length());
        }
    }

    return num_filtered;
}

void Reconstruction::SetObservationAsTriangulated(
        const image_t image_id, const point2D_t point2D_idx,
        const bool is_continued_point3D) {
    if (scene_graph_ == nullptr) {
        return;
    }

    const class Image& image = Image(image_id);
    const Point2D& point2D = image.Point2D(point2D_idx);
    const std::vector<SceneGraph::Correspondence>& corrs =
            scene_graph_->FindCorrespondences(image_id, point2D_idx);

    for (const auto& corr : corrs) {
        class Image& corr_image = Image(corr.image_id);
        const Point2D& corr_point2D = corr_image.Point2D(corr.point2D_idx);
        corr_image.IncrementCorrespondenceHasPoint3D(corr.point2D_idx);
        if (point2D.Point3DId() == corr_point2D.Point3DId() &&
            (is_continued_point3D || image_id < corr.image_id)) {
            const image_pair_t pair_id =
                    Database::ImagePairToPairId(image_id, corr.image_id);
            image_pairs_[pair_id].first += 1;
        }
    }
}

void Reconstruction::ResetTriObservations(const image_t image_id,
                                          const point2D_t point2D_idx,
                                          const bool is_deleted_point3D) {
    if (scene_graph_ == nullptr) {
        return;
    }

    const class Image& image = Image(image_id);
    const Point2D& point2D = image.Point2D(point2D_idx);
    const std::vector<SceneGraph::Correspondence>& corrs =
            scene_graph_->FindCorrespondences(image_id, point2D_idx);

    for (const auto& corr : corrs) {
        class Image& corr_image = Image(corr.image_id);
        const Point2D& corr_point2D = corr_image.Point2D(corr.point2D_idx);
        corr_image.DecrementCorrespondenceHasPoint3D(corr.point2D_idx);
        if (point2D.Point3DId() == corr_point2D.Point3DId() &&
            (!is_deleted_point3D || image_id < corr.image_id)) {
            const image_pair_t pair_id =
                    Database::ImagePairToPairId(image_id, corr.image_id);
            image_pairs_[pair_id].first -= 1;
        }
    }
}

size_t Reconstruction::NumCameras() const { return cameras_.size(); }

size_t Reconstruction::NumImages() const { return images_.size(); }

size_t Reconstruction::NumRegImages() const { return reg_image_ids_.size(); }

size_t Reconstruction::NumPoints3D() const { return points3D_.size(); }

size_t Reconstruction::NumImagePairs() const { return image_pairs_.size(); }

const class Camera& Reconstruction::Camera(const camera_t camera_id) const {
    return cameras_.at(camera_id);
}

const class Image& Reconstruction::Image(const image_t image_id) const {
    return images_.at(image_id);
}

const class Point_3D& Reconstruction::Point3D(const point3D_t point3D_id) const {
    return points3D_.at(point3D_id);
}

const std::pair<size_t, size_t>& Reconstruction::ImagePair(
        const image_pair_t pair_id) const {
    return image_pairs_.at(pair_id);
}

const std::pair<size_t, size_t>& Reconstruction::ImagePair(
        const image_t image_id1, const image_t image_id2) const {
    const auto pair_id = Database::ImagePairToPairId(image_id1, image_id2);
    return image_pairs_.at(pair_id);
}

class Camera& Reconstruction::Camera(const camera_t camera_id) {
    return cameras_.at(camera_id);
}

class Image& Reconstruction::Image(const image_t image_id) {
    return images_.at(image_id);
}

class Point_3D& Reconstruction::Point3D(const point3D_t point3D_id) {
    return points3D_.at(point3D_id);
}

std::pair<size_t, size_t>& Reconstruction::ImagePair(
        const image_pair_t pair_id) {
    return image_pairs_.at(pair_id);
}

std::pair<size_t, size_t>& Reconstruction::ImagePair(const image_t image_id1,
                                                     const image_t image_id2) {
    const auto pair_id = Database::ImagePairToPairId(image_id1, image_id2);
    return image_pairs_.at(pair_id);
}

const std::unordered_map<camera_t, Camera>& Reconstruction::Cameras() const {
    return cameras_;
}

const std::unordered_map<image_t, Image>& Reconstruction::Images() const {
    return images_;
}

const std::vector<image_t>& Reconstruction::RegImageIds() const {
    return reg_image_ids_;
}

const std::unordered_map<point3D_t, Point_3D>& Reconstruction::Points3D() const {
    return points3D_;
}

const std::unordered_map<image_pair_t, std::pair<size_t, size_t>>&
Reconstruction::ImagePairs() const {
    return image_pairs_;
}

bool Reconstruction::ExistsCamera(const camera_t camera_id) const {
    return cameras_.count(camera_id) > 0;
}

bool Reconstruction::ExistsImage(const image_t image_id) const {
    return images_.count(image_id) > 0;
}

bool Reconstruction::ExistsPoint3D(const point3D_t point3D_id) const {
    return points3D_.count(point3D_id) > 0;
}

bool Reconstruction::ExistsImagePair(const image_pair_t pair_id) const {
    return image_pairs_.count(pair_id) > 0;
}

bool Reconstruction::IsImageRegistered(const image_t image_id) const {
    return Image(image_id).IsRegistered();
}
