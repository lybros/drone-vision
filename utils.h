#ifndef INC_3D_RECONSTRUCTION_UTILS_H
#define INC_3D_RECONSTRUCTION_UTILS_H

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <future>
#include <iomanip>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Core>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/chrono.hpp>
#include <random>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wkeyword-macro"
#endif

// Define `thread_local` cross-platform
#ifndef thread_local
#if __STDC_VERSION__ >= 201112 && !defined __STDC_NO_THREADS__
#define thread_local _Thread_local
#elif defined _WIN32 && (defined _MSC_VER || defined __ICL || \
                         defined __DMC__ || defined __BORLANDC__)
#define thread_local __declspec(thread)
#elif defined __GNUC__ || defined __SUNPRO_C || defined __xlC__
#define thread_local __thread
#else
#error "Cannot define thread_local"
#endif
#endif

#ifdef __clang__
#pragma clang diagnostic pop  // -Wkeyword-macro
#endif

namespace Eigen {

typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;
typedef Eigen::Matrix<uint8_t, 3, 1> Vector3ub;
typedef Eigen::Matrix<uint8_t, 4, 1> Vector4ub;

}

typedef uint32_t camera_t;

typedef uint32_t image_t;

typedef uint64_t image_pair_t;

typedef uint32_t point2D_t;

typedef uint64_t point3D_t;

const camera_t kInvalidCameraId = std::numeric_limits<camera_t>::max();
const image_t kInvalidImageId = std::numeric_limits<image_t>::max();
const image_pair_t kInvalidImagePairId =
    std::numeric_limits<image_pair_t>::max();
const point2D_t kInvalidPoint2DIdx = std::numeric_limits<point2D_t>::max();
const point3D_t kInvalidPoint3DId = std::numeric_limits<point3D_t>::max();

std::string EnsureTrailingSlash(const std::string& str);

bool HasFileExtension(const std::string& file_name, const std::string& ext);

void PrintHeading1(const std::string& heading);

void PrintHeading2(const std::string& heading);

std::string StringReplace(const std::string& str, const std::string& old_str,
                              const std::string& new_str);

std::vector<std::string> StringSplit(const std::string& str,
                                         const std::string& delim);

template <typename T>
bool VectorContains(const std::vector<T>& vector, const T value);

template <typename T>
std::vector<T> CSVToVector(const std::string& csv);

template <typename T>
std::string VectorToCSV(const std::vector<T>& values);

template <typename T>
bool VectorContains(const std::vector<T>& vector, const T value) {
    return std::find_if(vector.begin(), vector.end(), [value](const T element) {
        return element == value;
    }) != vector.end();
}

template <typename T>
std::vector<T> CSVToVector(const std::string& csv) {
    auto elems = StringSplit(csv, ",;");
    std::vector<T> values;
    values.reserve(elems.size());
    for (auto& elem : elems) {
        boost::erase_all(elem, " ");
        if (elem.empty()) {
            continue;
        }
        try {
            values.push_back(boost::lexical_cast<T>(elem));
        } catch (std::exception) {
            return std::vector<T>(0);
        }
    }
    return values;
}

template <typename T>
std::string VectorToCSV(const std::vector<T>& values) {
    std::string string;
    for (const T value : values) {
        string += std::to_string(value) + ", ";
    }
    return string.substr(0, string.length() - 2);
}


#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

template <typename T>
int SignOfNumber(const T val);

bool IsNaN(const float x);
bool IsNaN(const double x);

template <typename Derived>
bool IsNaN(const Eigen::MatrixBase<Derived>& x);

bool IsInf(const float x);
bool IsInf(const double x);

template <typename Derived>
bool IsInf(const Eigen::MatrixBase<Derived>& x);

template <typename T>
T Clip(const T& value, const T& low, const T& high);

float DegToRad(const float deg);
double DegToRad(const double deg);

float RadToDeg(const float rad);
double RadToDeg(const double rad);

template <typename T>
double Median(const std::vector<T>& elems);

template <typename T>
double Mean(const std::vector<T>& elems);

template <typename T>
double Variance(const std::vector<T>& elems);

template <typename T>
double StdDev(const std::vector<T>& elems);

template <typename T>
bool AnyLessThan(std::vector<T> elems, T threshold);

template <typename T>
bool AnyGreaterThan(std::vector<T> elems, T threshold);

template <class Iterator>
bool NextCombination(Iterator first, Iterator middle, Iterator last);

template <typename T>
T Sigmoid(const T x, const T alpha = 1);

template <typename T>
T ScaleSigmoid(T x, const T alpha = 1, const T x0 = 10);

size_t NChooseK(const size_t n, const size_t k);

std::complex<double> EvaluatePolynomial(const std::vector<double>& coeffs,
                                        const std::complex<double> x);

std::vector<double> SolvePolynomial1(const double a, const double b);

std::vector<double> SolvePolynomial2(const double a, const double b,
                                     const double c);

std::vector<double> SolvePolynomial3(double a, double b, double c, double d);

std::vector<std::complex<double>> SolvePolynomialN(
        const std::vector<double>& coeffs, const int max_iter = 100,
        const double eps = 1e-10);

namespace internal {

    template <class Iterator>
    bool NextCombination(Iterator first1, Iterator last1, Iterator first2,
                         Iterator last2) {
        if ((first1 == last1) || (first2 == last2)) {
            return false;
        }
        Iterator m1 = last1;
        Iterator m2 = last2;
        --m2;
        while (--m1 != first1 && *m1 >= *m2) {
        }
        bool result = (m1 == first1) && *first1 >= *m2;
        if (!result) {
            while (first2 != m2 && *m1 >= *first2) {
                ++first2;
            }
            first1 = m1;
            std::iter_swap(first1, first2);
            ++first1;
            ++first2;
        }
        if ((first1 != last1) && (first2 != last2)) {
            m1 = last1;
            m2 = first2;
            while ((m1 != first1) && (m2 != last2)) {
                std::iter_swap(--m1, m2);
                ++m2;
            }
            std::reverse(first1, m1);
            std::reverse(first1, last1);
            std::reverse(m2, last2);
            std::reverse(first2, last2);
        }
        return !result;
    }

}

template <typename T>
int SignOfNumber(const T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename Derived>
bool IsNaN(const Eigen::MatrixBase<Derived>& x) {
    return !(x.array() == x.array()).all();
}

template <typename Derived>
bool IsInf(const Eigen::MatrixBase<Derived>& x) {
    return !((x - x).array() == (x - x).array()).all();
}

template <typename T>
T Clip(const T& value, const T& low, const T& high) {
    return std::max(low, std::min(value, high));
}


template <typename T>
double Median(const std::vector<T>& elems) {
    const size_t mid_idx = elems.size() / 2;

    std::vector<T> ordered_elems = elems;

    std::nth_element(ordered_elems.begin(), ordered_elems.begin() + mid_idx,
                     ordered_elems.end());
    if (elems.size() % 2 == 0) {
        const T mid_element1 = ordered_elems[mid_idx];
        const T mid_element2 = *std::max_element(ordered_elems.begin(),
                                                 ordered_elems.begin() + mid_idx);
        return (mid_element1 + mid_element2) / 2.0;
    } else {
        return ordered_elems[mid_idx];
    }
}

template <typename T>
double Mean(const std::vector<T>& elems) {
    double sum = 0;
    for (const auto el : elems) {
        sum += static_cast<double>(el);
    }
    return sum / elems.size();
}

template <typename T>
double Variance(const std::vector<T>& elems) {
    const double mean = Mean(elems);
    double var = 0;
    for (const auto el : elems) {
        const double diff = el - mean;
        var += diff * diff;
    }
    return var / (elems.size() - 1);
}

template <typename T>
double StdDev(const std::vector<T>& elems) {
    return std::sqrt(Variance(elems));
}

template <typename T>
bool AnyLessThan(std::vector<T> elems, T threshold) {
    for (const auto& el : elems) {
        if (el < threshold) {
            return true;
        }
    }
    return false;
}

template <typename T>
bool AnyGreaterThan(std::vector<T> elems, T threshold) {
    for (const auto& el : elems) {
        if (el > threshold) {
            return true;
        }
    }
    return false;
}

template <class Iterator>
bool NextCombination(Iterator first, Iterator middle, Iterator last) {
    return internal::NextCombination(first, middle, middle, last);
}

template <typename T>
T Sigmoid(const T x, const T alpha) {
    return T(1) / (T(1) + exp(-x * alpha));
}

template <typename T>
T ScaleSigmoid(T x, const T alpha, const T x0) {
    const T t0 = Sigmoid(-x0, alpha);
    const T t1 = Sigmoid(x0, alpha);
    x = (Sigmoid(2 * x0 * x - x0, alpha) - t0) / (t1 - t0);
    return x;
}

class ThreadPool {
public:
    static const int kMaxNumThreads = -1;

    ThreadPool(const int num_threads = kMaxNumThreads);
    ~ThreadPool();

    int NumThreads() const;

    template <class func_t, class... args_t>
    auto AddTask(func_t&& f, args_t&&... args)
            -> std::future<typename std::result_of<func_t(args_t...)>::type>;

    void Stop();

private:
    void WorkerFunc();

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;

    std::mutex mutex_;
    std::condition_variable condition_;
    bool stop_;
};

template <class func_t, class... args_t>
auto ThreadPool::AddTask(func_t&& f, args_t&&... args)
-> std::future<typename std::result_of<func_t(args_t...)>::type> {
    typedef typename std::result_of<func_t(args_t...)>::type return_t;

    auto task = std::make_shared<std::packaged_task<return_t()>>(
            std::bind(std::forward<func_t>(f), std::forward<args_t>(args)...));

    std::future<return_t> result = task->get_future();

    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_) {
            throw std::runtime_error("Cannot add task to stopped thread pool.");
        }
        tasks_.emplace([task]() { (*task)(); });
    }

    condition_.notify_one();

    return result;
}


extern thread_local std::mt19937* PRNG;

static const unsigned kRandomPRNGSeed = std::numeric_limits<unsigned>::max();

void SetPRNGSeed(unsigned seed = kRandomPRNGSeed);
template <typename T>
T RandomInteger(const T min, const T max);
template <typename T>
T RandomReal(const T min, const T max);
template <typename T>
T RandomGaussian(const T mean, const T stddev);
template <typename T>
void Shuffle(const uint32_t num_to_shuffle, std::vector<T>* elems);

template <typename T>
T RandomInteger(const T min, const T max) {
    if (PRNG == nullptr) {
        SetPRNGSeed();
    }

    std::uniform_int_distribution<T> distribution(min, max);

    return distribution(*PRNG);
}

template <typename T>
T RandomReal(const T min, const T max) {
    if (PRNG == nullptr) {
        SetPRNGSeed();
    }

    std::uniform_real_distribution<T> distribution(min, max);

    return distribution(*PRNG);
}

template <typename T>
T RandomGaussian(const T mean, const T stddev) {
    if (PRNG == nullptr) {
        SetPRNGSeed();
    }

    std::normal_distribution<T> distribution(mean, stddev);
    return distribution(*PRNG);
}

template <typename T>
void Shuffle(const uint32_t num_to_shuffle, std::vector<T>* elems) {
    const uint32_t last_idx = static_cast<uint32_t>(elems->size() - 1);
    for (uint32_t i = 0; i < num_to_shuffle; ++i) {
        const auto j = RandomInteger<uint32_t>(i, last_idx);
        std::swap((*elems)[i], (*elems)[j]);
    }
}


class Timer {
public:
    Timer();

    void Start();
    void Restart();
    void Pause();
    void Resume();
    void Reset();

    double ElapsedMicroSeconds() const;
    double ElapsedSeconds() const;
    double ElapsedMinutes() const;
    double ElapsedHours() const;

    void PrintSeconds() const;
    void PrintMinutes() const;
    void PrintHours() const;

private:
    bool started_;
    bool paused_;
    boost::chrono::high_resolution_clock::time_point start_time_;
    boost::chrono::high_resolution_clock::time_point pause_time_;
};



void RotationMatrixToEulerAngles(const Eigen::Matrix3d& R, double* rx,
                                 double* ry, double* rz);
Eigen::Matrix3d EulerAnglesToRotationMatrix(const double rx, const double ry,
                                            const double rz);
Eigen::Vector4d RotationMatrixToQuaternion(const Eigen::Matrix3d& rot_mat);
Eigen::Matrix3d QuaternionToRotationMatrix(const Eigen::Vector4d& qvec);
Eigen::Vector4d NormalizeQuaternion(const Eigen::Vector4d& qvec);
Eigen::Vector4d InvertQuaternion(const Eigen::Vector4d& qvec);
Eigen::Vector4d ConcatenateQuaternions(const Eigen::Vector4d& qvec1,
                                       const Eigen::Vector4d& qvec2);
Eigen::Vector3d QuaternionRotatePoint(const Eigen::Vector4d& qvec,
                                      const Eigen::Vector3d& point);
Eigen::Vector3d ProjectionCenterFromMatrix(
        const Eigen::Matrix3x4d& proj_matrix);
Eigen::Vector3d ProjectionCenterFromParameters(const Eigen::Vector4d& qvec,
                                               const Eigen::Vector3d& tvec);
void InterpolatePose(const Eigen::Vector4d& qvec1, const Eigen::Vector3d& tvec1,
                     const Eigen::Vector4d& qvec2, const Eigen::Vector3d& tvec2,
                     const double t, Eigen::Vector4d* qveci,
                     Eigen::Vector3d* tveci);
Eigen::Vector3d CalculateBaseline(const Eigen::Vector4d& qvec1,
                                  const Eigen::Vector3d& tvec1,
                                  const Eigen::Vector4d& qvec2,
                                  const Eigen::Vector3d& tvec2);

Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Vector4d& qvec,
                                          const Eigen::Vector3d& tvec);

Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Matrix3d& R,
                                          const Eigen::Vector3d& t);

Eigen::Matrix3x4d InvertProjectionMatrix(const Eigen::Matrix3x4d& proj_matrix);

double CalculateAngularError(const Eigen::Vector2d& point2D,
                             const Eigen::Vector3d& point3D,
                             const Eigen::Matrix3x4d& proj_matrix);

double CalculateDepth(const Eigen::Matrix3x4d& proj_matrix,
                      const Eigen::Vector3d& point3D);

bool HasPointPositiveDepth(const Eigen::Matrix3x4d& proj_matrix,
                           const Eigen::Vector3d& point3D);


Eigen::Vector3d TriangulatePoint(const Eigen::Matrix3x4d& proj_matrix1,
                                 const Eigen::Matrix3x4d& proj_matrix2,
                                 const Eigen::Vector2d& point1,
                                 const Eigen::Vector2d& point2);

std::vector<Eigen::Vector3d> TriangulatePoints(
        const Eigen::Matrix3x4d& proj_matrix1,
        const Eigen::Matrix3x4d& proj_matrix2,
        const std::vector<Eigen::Vector2d>& points1,
        const std::vector<Eigen::Vector2d>& points2);

Eigen::Vector3d TriangulateMultiViewPoint(
        const std::vector<Eigen::Matrix3x4d>& proj_matrices,
        const std::vector<Eigen::Vector2d>& points);

std::vector<Eigen::Vector3d> TriangulateOptimalPoints(
        const Eigen::Matrix3x4d& proj_matrix1,
        const Eigen::Matrix3x4d& proj_matrix2,
        const std::vector<Eigen::Vector2d>& points1,
        const std::vector<Eigen::Vector2d>& points2);

double CalculateTriangulationAngle(const Eigen::Vector3d& proj_center1,
                                   const Eigen::Vector3d& proj_center2,
                                   const Eigen::Vector3d& point3D);

std::vector<double> CalculateTriangulationAngles(
        const Eigen::Matrix3x4d& proj_matrix1,
        const Eigen::Matrix3x4d& proj_matrix2,
        const std::vector<Eigen::Vector3d>& points3D);


#endif //INC_3D_RECONSTRUCTION_UTILS_H
