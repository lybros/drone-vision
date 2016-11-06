#include "utils.h"


std::string EnsureTrailingSlash(const std::string& str) {
    if (str.length() > 0) {
        if (str.at(str.length() - 1) != '/') {
            return str + "/";
        }
    } else {
        return str + "/";
    }
    return str;
}

bool HasFileExtension(const std::string& file_name, const std::string& ext) {
    std::string ext_lower = ext;
    boost::to_lower(ext_lower);
    if (file_name.size() >= ext_lower.size() &&
        file_name.substr(file_name.size() - ext_lower.size(), ext_lower.size()) ==
        ext_lower) {
        return true;
    }
    return false;
}

void PrintHeading1(const std::string& heading) {
    std::cout << std::endl << std::string(78, '=') << std::endl;
    std::cout << heading << std::endl;
    std::cout << std::string(78, '=') << std::endl << std::endl;
}

void PrintHeading2(const std::string& heading) {
    std::cout << std::endl << heading << std::endl;
    std::cout << std::string(std::min<int>(heading.size(), 78), '-') << std::endl;
}

std::string StringReplace(const std::string& str, const std::string& old_str,
                          const std::string& new_str) {
    if (old_str.empty()) {
        return str;
    }
    size_t position = 0;
    std::string mod_str = str;
    while ((position = mod_str.find(old_str, position)) != std::string::npos) {
        mod_str.replace(position, old_str.size(), new_str);
        position += new_str.size();
    }
    return mod_str;
}

std::vector<std::string> StringSplit(const std::string& str,
                                     const std::string& delim) {
    std::vector<std::string> elems;
    boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
    return elems;
}


bool IsNaN(const float x) { return x != x; }

bool IsNaN(const double x) { return x != x; }

bool IsInf(const float x) { return !IsNaN(x) && IsNaN(x - x); }

bool IsInf(const double x) { return !IsNaN(x) && IsNaN(x - x); }

float DegToRad(const float deg) {
    return deg * 0.0174532925199432954743716805978692718781530857086181640625f;
}

double DegToRad(const double deg) {
    return deg * 0.0174532925199432954743716805978692718781530857086181640625;
}

float RadToDeg(const float rad) {
    return rad * 57.29577951308232286464772187173366546630859375f;
}

double RadToDeg(const double rad) {
    return rad * 57.29577951308232286464772187173366546630859375;
}


size_t NChooseK(const size_t n, const size_t k) {
    if (k == 0) {
        return 1;
    }

    return (n * NChooseK(n - 1, k - 1)) / k;
}


std::vector<double> SolvePolynomial1(const double a, const double b) {
    std::vector<double> roots;
    if (a != 0) {
        roots.resize(1);
        roots[0] = -b / a;
    }
    return roots;
}

std::vector<double> SolvePolynomial2(const double a, const double b,
                                     const double c) {
    std::vector<double> roots;
    if (a == 0) {
        roots = SolvePolynomial1(b, c);
    } else {
        const double d = b * b - 4 * a * c;
        if (d == 0) {
            roots.resize(1);
            roots[0] = -b / (2 * a);
        } else if (d > 0) {
            const double s = std::sqrt(d);
            const double q = -(b + (b > 0 ? s : -s)) / 2;
            roots.resize(2);
            roots[0] = q / a;
            roots[1] = c / q;
        }
    }
    return roots;
}

std::vector<double> SolvePolynomial3(double a, double b, double c, double d) {
    std::vector<double> roots;
    if (a == 0) {
        roots = SolvePolynomial2(b, c, d);
    } else {

        const double inv_a = 1 / a;
        b *= inv_a;
        c *= inv_a;
        d *= inv_a;

        const double p = (3 * c - b * b) / 3;
        const double q = 2 * b * b * b / 27 - b * c / 3 + d;
        const double b3 = b / 3;
        const double p3 = p / 3;
        const double q2 = q / 2;
        const double d = p3 * p3 * p3 + q2 * q2;

        if (d == 0 && p3 == 0) {
            roots.resize(3);
            roots[0] = -b3;
            roots[1] = -b3;
            roots[2] = -b3;
        } else {
            const std::complex<double> u =
                    std::pow(-q / 2 + std::sqrt(std::complex<double>(d)), 1 / 3.0);
            const std::complex<double> v = -p / (3.0 * u);
            const std::complex<double> y0 = u + v;

            if (d > 0) {
                roots.resize(1);
                roots[0] = y0.real() - b3;
            } else {
                const double sqrt3 =
                        1.7320508075688772935274463415058723669428052538103806;
                const std::complex<double> m = -y0 / 2.0;
                const std::complex<double> n =
                        (u - v) / 2.0 * std::complex<double>(0, sqrt3);
                const std::complex<double> y1 = m + n;
                if (d == 0) {
                    roots.resize(2);
                    roots[0] = y0.real() - b3;
                    roots[1] = y1.real() - b3;
                } else {
                    roots.resize(3);
                    const std::complex<double> y2 = m - n;
                    roots[0] = y0.real() - b3;
                    roots[1] = y1.real() - b3;
                    roots[2] = y2.real() - b3;
                }
            }
        }
    }
    return roots;
}

std::vector<std::complex<double>> SolvePolynomialN(
        const std::vector<double>& coeffs, const int max_iter, const double eps) {
    const size_t cn = coeffs.size();
    const size_t n = cn - 1;

    std::vector<std::complex<double>> roots(n);
    std::complex<double> p(1, 0);
    std::complex<double> r(1, 1);

    for (size_t i = 0; i < n; ++i) {
        roots[i] = p;
        p = p * r;
    }

    for (int iter = 0; iter < max_iter; ++iter) {
        double max_diff = 0;
        for (size_t i = 0; i < n; ++i) {
            p = roots[i];
            std::complex<double> nom = coeffs[n];
            std::complex<double> denom = coeffs[n];
            for (size_t j = 0; j < n; ++j) {
                nom = nom * p + coeffs[n - j - 1];
                if (j != i) {
                    denom = denom * (p - roots[j]);
                }
            }
            nom /= denom;
            roots[i] = p - nom;
            max_diff = std::max(max_diff, std::abs(nom.real()));
            max_diff = std::max(max_diff, std::abs(nom.imag()));
        }

        if (max_diff <= eps) {
            break;
        }
    }

    return roots;
}


ThreadPool::ThreadPool(const int num_threads) : stop_(false) {
    int num_effective_threads = num_threads;
    if (num_threads == kMaxNumThreads) {
        num_effective_threads = std::thread::hardware_concurrency();
    }

    if (num_effective_threads <= 0) {
        num_effective_threads = 1;
    }

    for (int i = 0; i < num_effective_threads; ++i) {
        std::function<void(void)> worker = std::bind(&ThreadPool::WorkerFunc, this);
        workers_.emplace_back(worker);
    }
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_) {
            return;
        }
        stop_ = true;
    }

    condition_.notify_all();
    for (auto& worker : workers_) {
        worker.join();
    }
}

void ThreadPool::Stop() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_) {
            return;
        }
        stop_ = true;
    }

    {
        std::queue<std::function<void()>> empty_tasks;
        std::swap(tasks_, empty_tasks);
    }

    condition_.notify_all();
    for (auto& worker : workers_) {
        worker.join();
    }
}

void ThreadPool::WorkerFunc() {
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
            if (stop_ && tasks_.empty()) {
                return;
            }
            task = std::move(tasks_.front());
            tasks_.pop();
        }
        task();
    }
}

int ThreadPool::NumThreads() const { return workers_.size(); }


thread_local std::mt19937* PRNG = nullptr;

void SetPRNGSeed(unsigned seed) {
    if (PRNG != nullptr) {
        delete PRNG;
    }

    if (seed == kRandomPRNGSeed) {
        seed = static_cast<unsigned>(
                std::chrono::system_clock::now().time_since_epoch().count());
    }

    PRNG = new std::mt19937(seed);
}


Timer::Timer() : started_(false), paused_(false) { }

void Timer::Start() {
    if (started_) {
        return;
    }
    started_ = true;
    start_time_ = boost::chrono::high_resolution_clock::now();
}

void Timer::Restart() {
    started_ = false;
    Start();
}

void Timer::Pause() {
    paused_ = true;
    pause_time_ = boost::chrono::high_resolution_clock::now();
}

void Timer::Resume() {
    if (!paused_) {
        return;
    }
    paused_ = false;
    start_time_ += boost::chrono::high_resolution_clock::now() - pause_time_;
}

void Timer::Reset() {
    started_ = false;
    paused_ = false;
}

double Timer::ElapsedMicroSeconds() const {
    if (!started_) {
        return 0.0;
    }
    if (paused_) {
        return boost::chrono::duration_cast<boost::chrono::microseconds>(pause_time_ - start_time_).count();
    } else {
        return boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::high_resolution_clock::now() -
                                                                         start_time_)
                .count();
    }
}

double Timer::ElapsedSeconds() const { return ElapsedMicroSeconds() / 1e6; }

double Timer::ElapsedMinutes() const { return ElapsedSeconds() / 60; }

double Timer::ElapsedHours() const { return ElapsedMinutes() / 60; }

void Timer::PrintSeconds() const {
    std::cout << "Elapsed time: " << std::setiosflags(std::ios::fixed)
    << std::setprecision(5) << ElapsedSeconds() << " [seconds]"
    << std::endl;
}

void Timer::PrintMinutes() const {
    std::cout << "Elapsed time: " << std::setiosflags(std::ios::fixed)
    << std::setprecision(2) << ElapsedMinutes() << " [minutes]"
    << std::endl;
}

void Timer::PrintHours() const {
    std::cout << "Elapsed time: " << std::setiosflags(std::ios::fixed)
    << std::setprecision(2) << ElapsedHours() << " [hours]"
    << std::endl;
}


void RotationMatrixToEulerAngles(const Eigen::Matrix3d& R, double* rx,
                                 double* ry, double* rz) {
    *rx = std::atan2(-R(1, 2), R(2, 2));
    *ry = std::asin(R(0, 2));
    *rz = std::atan2(-R(0, 1), R(0, 0));

    *rx = IsNaN(*rx) ? 0 : *rx;
    *ry = IsNaN(*ry) ? 0 : *ry;
    *rz = IsNaN(*rz) ? 0 : *rz;
}

Eigen::Matrix3d EulerAnglesToRotationMatrix(const double rx, const double ry,
                                            const double rz) {
    const Eigen::Matrix3d Rx =
            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d Ry =
            Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Matrix3d Rz =
            Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return Rz * Ry * Rx;
}

Eigen::Vector4d RotationMatrixToQuaternion(const Eigen::Matrix3d& rot_mat) {
    const Eigen::Quaterniond quat(rot_mat);
    return Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
}

Eigen::Matrix3d QuaternionToRotationMatrix(const Eigen::Vector4d& qvec) {
    const Eigen::Vector4d normalized_qvec = NormalizeQuaternion(qvec);
    const Eigen::Quaterniond quat(normalized_qvec(0), normalized_qvec(1),
                                  normalized_qvec(2), normalized_qvec(3));
    return quat.toRotationMatrix();
}

Eigen::Vector4d NormalizeQuaternion(const Eigen::Vector4d& qvec) {
    const double norm = qvec.norm();
    if (norm == 0) {
        return Eigen::Vector4d(1.0, qvec(1), qvec(2), qvec(3));
    } else {
        const double inv_norm = 1.0 / norm;
        return inv_norm * qvec;
    }
}

Eigen::Vector4d InvertQuaternion(const Eigen::Vector4d& qvec) {
    const Eigen::Vector4d normalized_qvec = NormalizeQuaternion(qvec);
    return Eigen::Vector4d(normalized_qvec(0), -normalized_qvec(1),
                           -normalized_qvec(2), -normalized_qvec(3));
}

Eigen::Vector4d ConcatenateQuaternions(const Eigen::Vector4d& qvec1,
                                       const Eigen::Vector4d& qvec2) {
    const Eigen::Vector4d normalized_qvec1 = NormalizeQuaternion(qvec1);
    const Eigen::Vector4d normalized_qvec2 = NormalizeQuaternion(qvec2);
    const Eigen::Quaterniond quat1(normalized_qvec1(0), normalized_qvec1(1),
                                   normalized_qvec1(2), normalized_qvec1(3));
    const Eigen::Quaterniond quat2(normalized_qvec2(0), normalized_qvec2(1),
                                   normalized_qvec2(2), normalized_qvec2(3));
    const Eigen::Quaterniond cat_quat = quat2 * quat1;
    return Eigen::Vector4d(cat_quat.w(), cat_quat.x(), cat_quat.y(),
                           cat_quat.z());
}

Eigen::Vector3d QuaternionRotatePoint(const Eigen::Vector4d& qvec,
                                      const Eigen::Vector3d& point) {
    const Eigen::Vector4d normalized_qvec = NormalizeQuaternion(qvec);
    const Eigen::Quaterniond quat(normalized_qvec(0), normalized_qvec(1),
                                  normalized_qvec(2), normalized_qvec(3));
    return quat * point;
}

Eigen::Vector3d ProjectionCenterFromMatrix(
        const Eigen::Matrix3x4d& proj_matrix) {
    return -proj_matrix.leftCols<3>().transpose() * proj_matrix.rightCols<1>();
}

Eigen::Vector3d ProjectionCenterFromParameters(const Eigen::Vector4d& qvec,
                                               const Eigen::Vector3d& tvec) {
    const Eigen::Vector4d normalized_qvec = NormalizeQuaternion(qvec);
    const Eigen::Quaterniond quat(normalized_qvec(0), -normalized_qvec(1),
                                  -normalized_qvec(2), -normalized_qvec(3));
    return quat * -tvec;
}

void InterpolatePose(const Eigen::Vector4d& qvec1, const Eigen::Vector3d& tvec1,
                     const Eigen::Vector4d& qvec2, const Eigen::Vector3d& tvec2,
                     const double t, Eigen::Vector4d* qveci,
                     Eigen::Vector3d* tveci) {
    const Eigen::Vector4d normalized_qvec1 = NormalizeQuaternion(qvec1);
    const Eigen::Vector4d normalized_qvec2 = NormalizeQuaternion(qvec2);
    const Eigen::Quaterniond quat1(normalized_qvec1(0), normalized_qvec1(1),
                                   normalized_qvec1(2), normalized_qvec1(3));
    const Eigen::Quaterniond quat2(normalized_qvec2(0), normalized_qvec2(1),
                                   normalized_qvec2(2), normalized_qvec2(3));
    const Eigen::Vector3d tvec12 = tvec2 - tvec1;

    const Eigen::Quaterniond quati = quat1.slerp(t, quat2);

    *qveci = Eigen::Vector4d(quati.w(), quati.x(), quati.y(), quati.z());
    *tveci = tvec1 + tvec12 * t;
}

Eigen::Vector3d CalculateBaseline(const Eigen::Vector4d& qvec1,
                                  const Eigen::Vector3d& tvec1,
                                  const Eigen::Vector4d& qvec2,
                                  const Eigen::Vector3d& tvec2) {
    const Eigen::Vector3d center1 = ProjectionCenterFromParameters(qvec1, tvec1);
    const Eigen::Vector3d center2 = ProjectionCenterFromParameters(qvec2, tvec2);
    return center2 - center1;
}


Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Vector4d& qvec,
                                          const Eigen::Vector3d& tvec) {
    Eigen::Matrix3x4d proj_matrix;
    proj_matrix.leftCols<3>() = QuaternionToRotationMatrix(qvec);
    proj_matrix.rightCols<1>() = tvec;
    return proj_matrix;
}

Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Matrix3d& R,
                                          const Eigen::Vector3d& t) {
    Eigen::Matrix3x4d proj_matrix;
    proj_matrix.leftCols<3>() = R;
    proj_matrix.rightCols<1>() = t;
    return proj_matrix;
}

Eigen::Matrix3x4d InvertProjectionMatrix(const Eigen::Matrix3x4d& proj_matrix) {
    Eigen::Matrix3x4d inv_proj_matrix;
    inv_proj_matrix.leftCols<3>() = proj_matrix.leftCols<3>().transpose();
    inv_proj_matrix.rightCols<1>() = ProjectionCenterFromMatrix(proj_matrix);
    return inv_proj_matrix;
}

double CalculateAngularError(const Eigen::Vector2d& point2D,
                             const Eigen::Vector3d& point3D,
                             const Eigen::Matrix3x4d& proj_matrix) {
    const Eigen::Vector3d ray1 = point2D.homogeneous();
    const Eigen::Vector3d ray2 = proj_matrix * point3D.homogeneous();
    return std::acos(ray1.normalized().transpose() * ray2.normalized());
}

double CalculateDepth(const Eigen::Matrix3x4d& proj_matrix,
                      const Eigen::Vector3d& point3D) {
    const double d = (proj_matrix.row(2) * point3D.homogeneous()).sum();
    return d * proj_matrix.col(2).norm();
}

bool HasPointPositiveDepth(const Eigen::Matrix3x4d& proj_matrix,
                           const Eigen::Vector3d& point3D) {
    return (proj_matrix(2, 0) * point3D(0) + proj_matrix(2, 1) * point3D(1) +
            proj_matrix(2, 2) * point3D(2) + proj_matrix(2, 3)) >
           std::numeric_limits<double>::epsilon();
}


Eigen::Vector3d TriangulatePoint(const Eigen::Matrix3x4d& proj_matrix1,
                                 const Eigen::Matrix3x4d& proj_matrix2,
                                 const Eigen::Vector2d& point1,
                                 const Eigen::Vector2d& point2) {
    Eigen::Matrix4d A;

    A.row(0) = point1(0) * proj_matrix1.row(2) - proj_matrix1.row(0);
    A.row(1) = point1(1) * proj_matrix1.row(2) - proj_matrix1.row(1);
    A.row(2) = point2(0) * proj_matrix2.row(2) - proj_matrix2.row(0);
    A.row(3) = point2(1) * proj_matrix2.row(2) - proj_matrix2.row(1);

    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);

    return svd.matrixV().col(3).hnormalized();
}

std::vector<Eigen::Vector3d> TriangulatePoints(
        const Eigen::Matrix3x4d& proj_matrix1,
        const Eigen::Matrix3x4d& proj_matrix2,
        const std::vector<Eigen::Vector2d>& points1,
        const std::vector<Eigen::Vector2d>& points2) {
    std::vector<Eigen::Vector3d> points3D(points1.size());

    for (size_t i = 0; i < points3D.size(); ++i) {
        points3D[i] =
                TriangulatePoint(proj_matrix1, proj_matrix2, points1[i], points2[i]);
    }

    return points3D;
}

Eigen::Vector3d TriangulateMultiViewPoint(
        const std::vector<Eigen::Matrix3x4d>& proj_matrices,
        const std::vector<Eigen::Vector2d>& points) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    for (size_t i = 0; i < points.size(); i++) {
        const Eigen::Vector3d point = points[i].homogeneous().normalized();
        const Eigen::Matrix3x4d term =
                proj_matrices[i] - point * point.transpose() * proj_matrices[i];
        A += term.transpose() * term;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(A);

    return eigen_solver.eigenvectors().col(0).hnormalized();
}


std::vector<Eigen::Vector3d> TriangulateOptimalPoints(
        const Eigen::Matrix3x4d& proj_matrix1,
        const Eigen::Matrix3x4d& proj_matrix2,
        const std::vector<Eigen::Vector2d>& points1,
        const std::vector<Eigen::Vector2d>& points2) {
    std::vector<Eigen::Vector3d> points3D(points1.size());

    for (size_t i = 0; i < points3D.size(); ++i) {
        points3D[i] =
                TriangulatePoint(proj_matrix1, proj_matrix2, points1[i], points2[i]);
    }

    return points3D;
}

double CalculateTriangulationAngle(const Eigen::Vector3d& proj_center1,
                                   const Eigen::Vector3d& proj_center2,
                                   const Eigen::Vector3d& point3D) {
    const double baseline2 = (proj_center1 - proj_center2).squaredNorm();

    const double ray1 = (point3D - proj_center1).norm();
    const double ray2 = (point3D - proj_center2).norm();

    const double angle = std::abs(
            std::acos((ray1 * ray1 + ray2 * ray2 - baseline2) / (2 * ray1 * ray2)));

    if (IsNaN(angle)) {
        return 0;
    } else {
        return std::min(angle, M_PI - angle);
    }
}

std::vector<double> CalculateTriangulationAngles(
        const Eigen::Matrix3x4d& proj_matrix1,
        const Eigen::Matrix3x4d& proj_matrix2,
        const std::vector<Eigen::Vector3d>& points3D) {
    const Eigen::Vector3d& proj_center1 =
            ProjectionCenterFromMatrix(proj_matrix1);
    const Eigen::Vector3d& proj_center2 =
            ProjectionCenterFromMatrix(proj_matrix2);

    const double baseline2 = (proj_center1 - proj_center2).squaredNorm();

    std::vector<double> angles(points3D.size());

    for (size_t i = 0; i < points3D.size(); ++i) {
        const Eigen::Vector3d& point3D = points3D[i];

        const double ray1 = (point3D - proj_center1).norm();
        const double ray2 = (point3D - proj_center2).norm();

        const double angle = std::abs(
                std::acos((ray1 * ray1 + ray2 * ray2 - baseline2) / (2 * ray1 * ray2)));

        if (IsNaN(angle)) {
            angles[i] = 0;
        } else {
            angles[i] = std::min(angle, M_PI - angle);
        }
    }

    return angles;
}
