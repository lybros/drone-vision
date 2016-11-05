#include "estimators.h"

void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d>& points,
                                   std::vector<Eigen::Vector2d>* normed_points,
                                   Eigen::Matrix3d* matrix) {
    Eigen::Vector2d centroid(0, 0);
    for (const auto point : points) {
        centroid += point;
    }
    centroid /= points.size();

    double rms_mean_dist = 0;
    for (const auto point : points) {
        rms_mean_dist += (point - centroid).squaredNorm();
    }
    rms_mean_dist = std::sqrt(rms_mean_dist / points.size());

    const double norm_factor = std::sqrt(2.0) / rms_mean_dist;
    *matrix << norm_factor, 0, -norm_factor * centroid(0), 0, norm_factor,
            -norm_factor * centroid(1), 0, 0, 1;

    normed_points->resize(points.size());

    const double M_00 = (*matrix)(0, 0);
    const double M_01 = (*matrix)(0, 1);
    const double M_02 = (*matrix)(0, 2);
    const double M_10 = (*matrix)(1, 0);
    const double M_11 = (*matrix)(1, 1);
    const double M_12 = (*matrix)(1, 2);
    const double M_20 = (*matrix)(2, 0);
    const double M_21 = (*matrix)(2, 1);
    const double M_22 = (*matrix)(2, 2);

    for (size_t i = 0; i < points.size(); ++i) {
        const double p_0 = points[i](0);
        const double p_1 = points[i](1);

        const double np_0 = M_00 * p_0 + M_01 * p_1 + M_02;
        const double np_1 = M_10 * p_0 + M_11 * p_1 + M_12;
        const double np_2 = M_20 * p_0 + M_21 * p_1 + M_22;

        const double inv_np_2 = 1.0 / np_2;
        (*normed_points)[i](0) = np_0 * inv_np_2;
        (*normed_points)[i](1) = np_1 * inv_np_2;
    }
}

void ComputeSquaredSampsonError(const std::vector<Eigen::Vector2d>& points1,
                                const std::vector<Eigen::Vector2d>& points2,
                                const Eigen::Matrix3d& E,
                                std::vector<double>* residuals) {
    residuals->resize(points1.size());

    const double E_00 = E(0, 0);
    const double E_01 = E(0, 1);
    const double E_02 = E(0, 2);
    const double E_10 = E(1, 0);
    const double E_11 = E(1, 1);
    const double E_12 = E(1, 2);
    const double E_20 = E(2, 0);
    const double E_21 = E(2, 1);
    const double E_22 = E(2, 2);

    for (size_t i = 0; i < points1.size(); ++i) {
        const double x1_0 = points1[i](0);
        const double x1_1 = points1[i](1);
        const double x2_0 = points2[i](0);
        const double x2_1 = points2[i](1);

        const double Ex1_0 = E_00 * x1_0 + E_01 * x1_1 + E_02;
        const double Ex1_1 = E_10 * x1_0 + E_11 * x1_1 + E_12;
        const double Ex1_2 = E_20 * x1_0 + E_21 * x1_1 + E_22;

        const double Etx2_0 = E_00 * x2_0 + E_10 * x2_1 + E_20;
        const double Etx2_1 = E_01 * x2_0 + E_11 * x2_1 + E_21;

        const double x2tEx1 = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;

        (*residuals)[i] = x2tEx1 * x2tEx1 / (Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1 +
                                             Etx2_0 * Etx2_0 + Etx2_1 * Etx2_1);
    }
}

void ComputeSquaredReprojectionError(
        const std::vector<Eigen::Vector2d>& points2D,
        const std::vector<Eigen::Vector3d>& points3D,
        const Eigen::Matrix3x4d& proj_matrix, std::vector<double>* residuals) {
    residuals->resize(points2D.size());

    const double P_00 = proj_matrix(0, 0);
    const double P_01 = proj_matrix(0, 1);
    const double P_02 = proj_matrix(0, 2);
    const double P_03 = proj_matrix(0, 3);
    const double P_10 = proj_matrix(1, 0);
    const double P_11 = proj_matrix(1, 1);
    const double P_12 = proj_matrix(1, 2);
    const double P_13 = proj_matrix(1, 3);
    const double P_20 = proj_matrix(2, 0);
    const double P_21 = proj_matrix(2, 1);
    const double P_22 = proj_matrix(2, 2);
    const double P_23 = proj_matrix(2, 3);

    for (size_t i = 0; i < points2D.size(); ++i) {
        const double x_0 = points2D[i](0);
        const double x_1 = points2D[i](1);

        const double X_0 = points3D[i](0);
        const double X_1 = points3D[i](1);
        const double X_2 = points3D[i](2);

        const double px_2 = P_20 * X_0 + P_21 * X_1 + P_22 * X_2 + P_23;

        if (px_2 > std::numeric_limits<double>::epsilon()) {
            const double px_0 = P_00 * X_0 + P_01 * X_1 + P_02 * X_2 + P_03;
            const double px_1 = P_10 * X_0 + P_11 * X_1 + P_12 * X_2 + P_13;

            const double inv_px_2 = 1.0 / px_2;
            const double dx_0 = x_0 - px_0 * inv_px_2;
            const double dx_1 = x_1 - px_1 * inv_px_2;

            (*residuals)[i] = dx_0 * dx_0 + dx_1 * dx_1;
        } else {
            (*residuals)[i] = std::numeric_limits<double>::max();
        }
    }
}


InlierSupportMeasurer::Support InlierSupportMeasurer::Evaluate(
        const std::vector<double>& residuals, const double max_residual) {
    Support support;
    support.num_inliers = 0;
    support.residual_sum = 0;

    for (const auto residual : residuals) {
        if (residual <= max_residual) {
            support.num_inliers += 1;
            support.residual_sum += residual;
        }
    }

    return support;
}

bool InlierSupportMeasurer::Compare(const Support& support1,
                                    const Support& support2) {
    if (support1.num_inliers > support2.num_inliers) {
        return true;
    } else {
        return support1.num_inliers == support2.num_inliers &&
               support1.residual_sum < support2.residual_sum;
    }
}

MEstimatorSupportMeasurer::Support MEstimatorSupportMeasurer::Evaluate(
        const std::vector<double>& residuals, const double max_residual) {
    Support support;
    support.num_inliers = 0;
    support.score = 0;

    for (const auto residual : residuals) {
        if (residual <= max_residual) {
            support.num_inliers += 1;
            support.score += residual;
        } else {
            support.score += max_residual;
        }
    }

    return support;
}

bool MEstimatorSupportMeasurer::Compare(const Support& support1,
                                        const Support& support2) {
    return support1.score < support2.score;
}


RandomSampler::RandomSampler(const size_t num_samples)
        : num_samples_(num_samples) {}

void RandomSampler::Initialize(const size_t total_num_samples) {
    sample_idxs_.resize(total_num_samples);
    std::iota(sample_idxs_.begin(), sample_idxs_.end(), 0);
}

size_t RandomSampler::MaxNumSamples() {
    return std::numeric_limits<size_t>::max();
}

std::vector<size_t> RandomSampler::Sample() {
    Shuffle(static_cast<uint32_t>(num_samples_), &sample_idxs_);

    std::vector<size_t> sampled_idxs(num_samples_);
    for (size_t i = 0; i < num_samples_; ++i) {
        sampled_idxs[i] = sample_idxs_[i];
    }

    return sampled_idxs;
}


CombinationSampler::CombinationSampler(const size_t num_samples)
        : num_samples_(num_samples) {}

void CombinationSampler::Initialize(const size_t total_num_samples) {
    total_sample_idxs_.resize(total_num_samples);
    std::iota(total_sample_idxs_.begin(), total_sample_idxs_.end(), 0);
}

size_t CombinationSampler::MaxNumSamples() {
    return NChooseK(total_sample_idxs_.size(), num_samples_);
}

std::vector<size_t> CombinationSampler::Sample() {
    std::vector<size_t> sampled_idxs(num_samples_);
    for (size_t i = 0; i < num_samples_; ++i) {
        sampled_idxs[i] = total_sample_idxs_[i];
    }

    if (!NextCombination(total_sample_idxs_.begin(),
                         total_sample_idxs_.begin() + num_samples_,
                         total_sample_idxs_.end())) {
        std::iota(total_sample_idxs_.begin(), total_sample_idxs_.end(), 0);
    }

    return sampled_idxs;
}


namespace {
    Eigen::Vector3d LiftImagePoint(const Eigen::Vector2d& point) {
        return point.homogeneous() / std::sqrt(point.squaredNorm() + 1);
    }
}

std::vector<P3PEstimator::M_t> P3PEstimator::Estimate(
        const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
    Eigen::Matrix3d points3D_world;
    points3D_world.col(0) = points3D[0];
    points3D_world.col(1) = points3D[1];
    points3D_world.col(2) = points3D[2];

    const Eigen::Vector3d u = LiftImagePoint(points2D[0]);
    const Eigen::Vector3d v = LiftImagePoint(points2D[1]);
    const Eigen::Vector3d w = LiftImagePoint(points2D[2]);

    const double cos_uv = u.transpose() * v;
    const double cos_uw = u.transpose() * w;
    const double cos_vw = v.transpose() * w;

    const double dist_AB_2 = (points3D[0] - points3D[1]).squaredNorm();
    const double dist_AC_2 = (points3D[0] - points3D[2]).squaredNorm();
    const double dist_BC_2 = (points3D[1] - points3D[2]).squaredNorm();

    const double dist_AB = std::sqrt(dist_AB_2);

    const double a = dist_BC_2 / dist_AB_2;
    const double b = dist_AC_2 / dist_AB_2;

    const double a2 = a * a;
    const double b2 = b * b;
    const double p = 2 * cos_vw;
    const double q = 2 * cos_uw;
    const double r = 2 * cos_uv;
    const double p2 = p * p;
    const double p3 = p2 * p;
    const double q2 = q * q;
    const double r2 = r * r;
    const double r3 = r2 * r;
    const double r4 = r3 * r;
    const double r5 = r4 * r;

    std::vector<double> coeffs_a(5);
    coeffs_a[4] = -2 * b + b2 + a2 + 1 + a * b * (2 - r2) - 2 * a;
    coeffs_a[3] = -2 * q * a2 - r * p * b2 + 4 * q * a + (2 * q + p * r) * b +
                  (r2 * q - 2 * q + r * p) * a * b - 2 * q;
    coeffs_a[2] = (2 + q2) * a2 + (p2 + r2 - 2) * b2 - (4 + 2 * q2) * a -
                  (p * q * r + p2) * b - (p * q * r + r2) * a * b + q2 + 2;
    coeffs_a[1] = -2 * q * a2 - r * p * b2 + 4 * q * a +
                  (p * r + q * p2 - 2 * q) * b + (r * p + 2 * q) * a * b -
                  2 * q;
    coeffs_a[0] = a2 + b2 - 2 * a + (2 - p2) * b - 2 * a * b + 1;

    const std::vector<std::complex<double>> roots_a = SolvePolynomialN(coeffs_a);

    std::vector<M_t> models;

    const double kEps = 1e-10;

    for (const auto root_a : roots_a) {
        const double x = root_a.real();

        if (root_a.imag() > kEps || x < 0) {
            continue;
        }

        const double x2 = x * x;
        const double x3 = x2 * x;

        const double _b1 =
                (p2 - p * q * r + r2) * a + (p2 - r2) * b - p2 + p * q * r - r2;
        const double b1 = b * _b1 * _b1;
        const double b0 =
                ((1 - a - b) * x2 + (a - 1) * q * x - a + b + 1) *
                (r3 * (a2 + b2 - 2 * a - 2 * b + (2 - r2) * a * b + 1) * x3 +
                 r2 * (p + p * a2 - 2 * r * q * a * b + 2 * r * q * b - 2 * r * q -
                       2 * p * a - 2 * p * b + p * r2 * b + 4 * r * q * a +
                       q * r3 * a * b - 2 * r * q * a2 + 2 * p * a * b + p * b2 -
                       r2 * p * b2) *
                 x2 +
                 (r5 * (b2 - a * b) - r4 * p * q * b +
                  r3 * (q2 - 4 * a - 2 * q2 * a + q2 * a2 + 2 * a2 - 2 * b2 + 2) +
                  r2 * (4 * p * q * a - 2 * p * q * a * b + 2 * p * q * b - 2 * p * q -
                        2 * p * q * a2) +
                  r * (p2 * b2 - 2 * p2 * b + 2 * p2 * a * b - 2 * p2 * a + p2 +
                       p2 * a2)) *
                 x +
                 (2 * p * r2 - 2 * r3 * q + p3 - 2 * p2 * q * r + p * q2 * r2) * a2 +
                 (p3 - 2 * p * r2) * b2 +
                 (4 * q * r3 - 4 * p * r2 - 2 * p3 + 4 * p2 * q * r - 2 * p * q2 * r2) *
                 a +
                 (-2 * q * r3 + p * r4 + 2 * p2 * q * r - 2 * p3) * b +
                 (2 * p3 + 2 * q * r3 - 2 * p2 * q * r) * a * b + p * q2 * r2 -
                 2 * p2 * q * r + 2 * p * r2 + p3 - 2 * r3 * q);

        const double y = b0 / b1;
        const double y2 = y * y;

        const double nu = x2 + y2 - 2 * x * y * cos_uv;

        const double dist_PC = dist_AB / std::sqrt(nu);
        const double dist_PB = y * dist_PC;
        const double dist_PA = x * dist_PC;

        Eigen::Matrix3d points3D_camera;
        points3D_camera.col(0) = u * dist_PA;  // A'
        points3D_camera.col(1) = v * dist_PB;  // B'
        points3D_camera.col(2) = w * dist_PC;  // C'

        const Eigen::Matrix4d matrix =
                Eigen::umeyama(points3D_world, points3D_camera, false);

        models.push_back(matrix.topLeftCorner<3, 4>());
    }

    return models;
}

void P3PEstimator::Residuals(const std::vector<X_t>& points2D,
                             const std::vector<Y_t>& points3D,
                             const M_t& proj_matrix,
                             std::vector<double>* residuals) {
    residuals->resize(points2D.size());

    const double P_00 = proj_matrix(0, 0);
    const double P_01 = proj_matrix(0, 1);
    const double P_02 = proj_matrix(0, 2);
    const double P_03 = proj_matrix(0, 3);
    const double P_10 = proj_matrix(1, 0);
    const double P_11 = proj_matrix(1, 1);
    const double P_12 = proj_matrix(1, 2);
    const double P_13 = proj_matrix(1, 3);
    const double P_20 = proj_matrix(2, 0);
    const double P_21 = proj_matrix(2, 1);
    const double P_22 = proj_matrix(2, 2);
    const double P_23 = proj_matrix(2, 3);

    for (size_t i = 0; i < points2D.size(); ++i) {
        const double x_0 = points2D[i](0);
        const double x_1 = points2D[i](1);

        const double X_0 = points3D[i](0);
        const double X_1 = points3D[i](1);
        const double X_2 = points3D[i](2);

        const double px_2 = P_20 * X_0 + P_21 * X_1 + P_22 * X_2 + P_23;

        if (px_2 > std::numeric_limits<double>::epsilon()) {
            const double px_0 = P_00 * X_0 + P_01 * X_1 + P_02 * X_2 + P_03;
            const double px_1 = P_10 * X_0 + P_11 * X_1 + P_12 * X_2 + P_13;

            const double inv_px_2 = 1.0 / px_2;
            const double dx_0 = x_0 - px_0 * inv_px_2;
            const double dx_1 = x_1 - px_1 * inv_px_2;

            (*residuals)[i] = dx_0 * dx_0 + dx_1 * dx_1;
        } else {
            (*residuals)[i] = std::numeric_limits<double>::max();
        }
    }
}


std::vector<EPnPEstimator::M_t> EPnPEstimator::Estimate(
        const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
    EPnPEstimator epnp;
    M_t proj_matrix;
    if (!epnp.ComputePose(points2D, points3D, &proj_matrix)) {
        return {};
    }

    return {proj_matrix};
}

void EPnPEstimator::Residuals(const std::vector<X_t>& points2D,
                              const std::vector<Y_t>& points3D,
                              const M_t& proj_matrix,
                              std::vector<double>* residuals) {
    ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

bool EPnPEstimator::ComputePose(const std::vector<Eigen::Vector2d>& points2D,
                                const std::vector<Eigen::Vector3d>& points3D,
                                Eigen::Matrix3x4d* proj_matrix) {
    points2D_ = points2D;
    points3D_ = points3D;

    ChooseControlPoints();

    if (!ComputeBarycentricCoordinates()) {
        return false;
    }

    const Eigen::Matrix<double, Eigen::Dynamic, 12> M = ComputeM();
    const Eigen::Matrix<double, 12, 12> MtM = M.transpose() * M;

    Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(
            MtM, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::Matrix<double, 12, 12> Ut = svd.matrixU().transpose();

    const Eigen::Matrix<double, 6, 10> L6x10 = ComputeL6x10(Ut);
    const Eigen::Matrix<double, 6, 1> rho = ComputeRho();

    Eigen::Vector4d betas[4];
    std::array<double, 4> reproj_errors;
    std::array<Eigen::Matrix3d, 4> Rs;
    std::array<Eigen::Vector3d, 4> ts;

    FindBetasApprox1(L6x10, rho, &betas[1]);
    RunGaussNewton(L6x10, rho, &betas[1]);
    reproj_errors[1] = ComputeRT(Ut, betas[1], &Rs[1], &ts[1]);

    FindBetasApprox2(L6x10, rho, &betas[2]);
    RunGaussNewton(L6x10, rho, &betas[2]);
    reproj_errors[2] = ComputeRT(Ut, betas[2], &Rs[2], &ts[2]);

    FindBetasApprox3(L6x10, rho, &betas[3]);
    RunGaussNewton(L6x10, rho, &betas[3]);
    reproj_errors[3] = ComputeRT(Ut, betas[3], &Rs[3], &ts[3]);

    int best_idx = 1;
    if (reproj_errors[2] < reproj_errors[1]) {
        best_idx = 2;
    }
    if (reproj_errors[3] < reproj_errors[best_idx]) {
        best_idx = 3;
    }

    proj_matrix->leftCols<3>() = Rs[best_idx];
    proj_matrix->rightCols<1>() = ts[best_idx];

    return true;
}

void EPnPEstimator::ChooseControlPoints() {
    cws_[0].setZero();
    for (size_t i = 0; i < points3D_.size(); ++i) {
        cws_[0] += points3D_[i];
    }
    cws_[0] /= points3D_.size();

    Eigen::Matrix<double, Eigen::Dynamic, 3> PW0(points3D_.size(), 3);
    for (size_t i = 0; i < points3D_.size(); ++i) {
        PW0.row(i) = points3D_[i] - cws_[0];
    }

    const Eigen::Matrix3d PW0tPW0 = PW0.transpose() * PW0;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            PW0tPW0, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::Vector3d D = svd.singularValues();
    const Eigen::Matrix3d Ut = svd.matrixU().transpose();

    for (int i = 1; i < 4; ++i) {
        const double k = std::sqrt(D(i - 1) / points3D_.size());
        cws_[i] = cws_[0] + k * Ut.row(i - 1).transpose();
    }
}

bool EPnPEstimator::ComputeBarycentricCoordinates() {
    Eigen::Matrix3d CC;
    for (int i = 0; i < 3; ++i) {
        for (int j = 1; j < 4; ++j) {
            CC(i, j - 1) = cws_[j][i] - cws_[0][i];
        }
    }

    if (CC.colPivHouseholderQr().rank() < 3) {
        return false;
    }

    const Eigen::Matrix3d CC_inv = CC.inverse();

    alphas_.resize(points2D_.size());
    for (size_t i = 0; i < points3D_.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            alphas_[i][1 + j] = CC_inv(j, 0) * (points3D_[i][0] - cws_[0][0]) +
                                CC_inv(j, 1) * (points3D_[i][1] - cws_[0][1]) +
                                CC_inv(j, 2) * (points3D_[i][2] - cws_[0][2]);
        }
        alphas_[i][0] = 1.0 - alphas_[i][1] - alphas_[i][2] - alphas_[i][3];
    }

    return true;
}

Eigen::Matrix<double, Eigen::Dynamic, 12> EPnPEstimator::ComputeM() {
    Eigen::Matrix<double, Eigen::Dynamic, 12> M(2 * points2D_.size(), 12);
    for (size_t i = 0; i < points3D_.size(); ++i) {
        for (size_t j = 0; j < 4; ++j) {
            M(2 * i, 3 * j) = alphas_[i][j];
            M(2 * i, 3 * j + 1) = 0.0;
            M(2 * i, 3 * j + 2) = -alphas_[i][j] * points2D_[i].x();

            M(2 * i + 1, 3 * j) = 0.0;
            M(2 * i + 1, 3 * j + 1) = alphas_[i][j];
            M(2 * i + 1, 3 * j + 2) = -alphas_[i][j] * points2D_[i].y();
        }
    }
    return M;
}

Eigen::Matrix<double, 6, 10> EPnPEstimator::ComputeL6x10(
        const Eigen::Matrix<double, 12, 12>& Ut) {
    Eigen::Matrix<double, 6, 10> L6x10;

    std::array<std::array<Eigen::Vector3d, 6>, 4> dv;
    for (int i = 0; i < 4; ++i) {
        int a = 0, b = 1;
        for (int j = 0; j < 6; ++j) {
            dv[i][j][0] = Ut(11 - i, 3 * a) - Ut(11 - i, 3 * b);
            dv[i][j][1] = Ut(11 - i, 3 * a + 1) - Ut(11 - i, 3 * b + 1);
            dv[i][j][2] = Ut(11 - i, 3 * a + 2) - Ut(11 - i, 3 * b + 2);

            b += 1;
            if (b > 3) {
                a += 1;
                b = a + 1;
            }
        }
    }

    for (int i = 0; i < 6; ++i) {
        L6x10(i, 0) = dv[0][i].transpose() * dv[0][i];
        L6x10(i, 1) = 2.0 * dv[0][i].transpose() * dv[1][i];
        L6x10(i, 2) = dv[1][i].transpose() * dv[1][i];
        L6x10(i, 3) = 2.0 * dv[0][i].transpose() * dv[2][i];
        L6x10(i, 4) = 2.0 * dv[1][i].transpose() * dv[2][i];
        L6x10(i, 5) = dv[2][i].transpose() * dv[2][i];
        L6x10(i, 6) = 2.0 * dv[0][i].transpose() * dv[3][i];
        L6x10(i, 7) = 2.0 * dv[1][i].transpose() * dv[3][i];
        L6x10(i, 8) = 2.0 * dv[2][i].transpose() * dv[3][i];
        L6x10(i, 9) = dv[3][i].transpose() * dv[3][i];
    }

    return L6x10;
}

Eigen::Matrix<double, 6, 1> EPnPEstimator::ComputeRho() {
    Eigen::Matrix<double, 6, 1> rho;
    rho[0] = (cws_[0] - cws_[1]).squaredNorm();
    rho[1] = (cws_[0] - cws_[2]).squaredNorm();
    rho[2] = (cws_[0] - cws_[3]).squaredNorm();
    rho[3] = (cws_[1] - cws_[2]).squaredNorm();
    rho[4] = (cws_[1] - cws_[3]).squaredNorm();
    rho[5] = (cws_[2] - cws_[3]).squaredNorm();
    return rho;
}


void EPnPEstimator::FindBetasApprox1(const Eigen::Matrix<double, 6, 10>& L6x10,
                                     const Eigen::Matrix<double, 6, 1>& rho,
                                     Eigen::Vector4d* betas) {
    Eigen::Matrix<double, 6, 4> L_6x4;
    for (int i = 0; i < 6; ++i) {
        L_6x4(i, 0) = L6x10(i, 0);
        L_6x4(i, 1) = L6x10(i, 1);
        L_6x4(i, 2) = L6x10(i, 3);
        L_6x4(i, 3) = L6x10(i, 6);
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 4>> svd(
            L_6x4, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix<double, 6, 1> Rho_temp = rho;
    const Eigen::Matrix<double, 4, 1> b4 = svd.solve(Rho_temp);

    if (b4[0] < 0) {
        (*betas)[0] = std::sqrt(-b4[0]);
        (*betas)[1] = -b4[1] / (*betas)[0];
        (*betas)[2] = -b4[2] / (*betas)[0];
        (*betas)[3] = -b4[3] / (*betas)[0];
    } else {
        (*betas)[0] = std::sqrt(b4[0]);
        (*betas)[1] = b4[1] / (*betas)[0];
        (*betas)[2] = b4[2] / (*betas)[0];
        (*betas)[3] = b4[3] / (*betas)[0];
    }
}

void EPnPEstimator::FindBetasApprox2(const Eigen::Matrix<double, 6, 10>& L6x10,
                                     const Eigen::Matrix<double, 6, 1>& rho,
                                     Eigen::Vector4d* betas) {
    Eigen::Matrix<double, 6, 3> L_6x3(6, 3);

    for (int i = 0; i < 6; ++i) {
        L_6x3(i, 0) = L6x10(i, 0);
        L_6x3(i, 1) = L6x10(i, 1);
        L_6x3(i, 2) = L6x10(i, 2);
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 3>> svd(
            L_6x3, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix<double, 6, 1> Rho_temp = rho;
    const Eigen::Matrix<double, 3, 1> b3 = svd.solve(Rho_temp);

    if (b3[0] < 0) {
        (*betas)[0] = std::sqrt(-b3[0]);
        (*betas)[1] = (b3[2] < 0) ? std::sqrt(-b3[2]) : 0.0;
    } else {
        (*betas)[0] = std::sqrt(b3[0]);
        (*betas)[1] = (b3[2] > 0) ? std::sqrt(b3[2]) : 0.0;
    }

    if (b3[1] < 0) {
        (*betas)[0] = -(*betas)[0];
    }

    (*betas)[2] = 0.0;
    (*betas)[3] = 0.0;
}

void EPnPEstimator::FindBetasApprox3(const Eigen::Matrix<double, 6, 10>& L6x10,
                                     const Eigen::Matrix<double, 6, 1>& rho,
                                     Eigen::Vector4d* betas) {
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 5>> svd(
            L6x10.leftCols<5>(), Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix<double, 6, 1> Rho_temp = rho;
    const Eigen::Matrix<double, 5, 1> b5 = svd.solve(Rho_temp);

    if (b5[0] < 0) {
        (*betas)[0] = std::sqrt(-b5[0]);
        (*betas)[1] = (b5[2] < 0) ? std::sqrt(-b5[2]) : 0.0;
    } else {
        (*betas)[0] = std::sqrt(b5[0]);
        (*betas)[1] = (b5[2] > 0) ? std::sqrt(b5[2]) : 0.0;
    }
    if (b5[1] < 0) {
        (*betas)[0] = -(*betas)[0];
    }
    (*betas)[2] = b5[3] / (*betas)[0];
    (*betas)[3] = 0.0;
}

void EPnPEstimator::RunGaussNewton(const Eigen::Matrix<double, 6, 10>& L6x10,
                                   const Eigen::Matrix<double, 6, 1>& rho,
                                   Eigen::Vector4d* betas) {
    Eigen::Matrix<double, 6, 4> A;
    Eigen::Matrix<double, 6, 1> b;

    const int kNumIterations = 5;
    for (int k = 0; k < kNumIterations; ++k) {
        for (int i = 0; i < 6; ++i) {
            A(i, 0) = 2 * L6x10(i, 0) * (*betas)[0] + L6x10(i, 1) * (*betas)[1] +
                      L6x10(i, 3) * (*betas)[2] + L6x10(i, 6) * (*betas)[3];
            A(i, 1) = L6x10(i, 1) * (*betas)[0] + 2 * L6x10(i, 2) * (*betas)[1] +
                      L6x10(i, 4) * (*betas)[2] + L6x10(i, 7) * (*betas)[3];
            A(i, 2) = L6x10(i, 3) * (*betas)[0] + L6x10(i, 4) * (*betas)[1] +
                      2 * L6x10(i, 5) * (*betas)[2] + L6x10(i, 8) * (*betas)[3];
            A(i, 3) = L6x10(i, 6) * (*betas)[0] + L6x10(i, 7) * (*betas)[1] +
                      L6x10(i, 8) * (*betas)[2] + 2 * L6x10(i, 9) * (*betas)[3];

            b(i) = rho[i] - (L6x10(i, 0) * (*betas)[0] * (*betas)[0] +
                             L6x10(i, 1) * (*betas)[0] * (*betas)[1] +
                             L6x10(i, 2) * (*betas)[1] * (*betas)[1] +
                             L6x10(i, 3) * (*betas)[0] * (*betas)[2] +
                             L6x10(i, 4) * (*betas)[1] * (*betas)[2] +
                             L6x10(i, 5) * (*betas)[2] * (*betas)[2] +
                             L6x10(i, 6) * (*betas)[0] * (*betas)[3] +
                             L6x10(i, 7) * (*betas)[1] * (*betas)[3] +
                             L6x10(i, 8) * (*betas)[2] * (*betas)[3] +
                             L6x10(i, 9) * (*betas)[3] * (*betas)[3]);
        }

        const Eigen::Vector4d x = A.colPivHouseholderQr().solve(b);

        (*betas) += x;
    }
}

double EPnPEstimator::ComputeRT(const Eigen::Matrix<double, 12, 12>& Ut,
                                const Eigen::Vector4d& betas,
                                Eigen::Matrix3d* R, Eigen::Vector3d* t) {
    ComputeCcs(betas, Ut);
    ComputePcs();

    SolveForSign();

    EstimateRT(R, t);

    return ComputeTotalReprojectionError(*R, *t);
}

void EPnPEstimator::ComputeCcs(const Eigen::Vector4d& betas,
                               const Eigen::Matrix<double, 12, 12>& Ut) {
    for (int i = 0; i < 4; ++i) {
        ccs_[i][0] = ccs_[i][1] = ccs_[i][2] = 0.0;
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 3; ++k) {
                ccs_[j][k] += betas[i] * Ut(11 - i, 3 * j + k);
            }
        }
    }
}

void EPnPEstimator::ComputePcs() {
    pcs_.resize(points2D_.size());
    for (size_t i = 0; i < points3D_.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            pcs_[i][j] = alphas_[i][0] * ccs_[0][j] + alphas_[i][1] * ccs_[1][j] +
                         alphas_[i][2] * ccs_[2][j] + alphas_[i][3] * ccs_[3][j];
        }
    }
}

void EPnPEstimator::SolveForSign() {
    if (pcs_[0][2] < 0.0 || pcs_[0][2] > 0.0) {
        for (int i = 0; i < 4; ++i) {
            ccs_[i] = -ccs_[i];
        }
        for (size_t i = 0; i < points3D_.size(); ++i) {
            pcs_[i] = -pcs_[i];
        }
    }
}

void EPnPEstimator::EstimateRT(Eigen::Matrix3d* R, Eigen::Vector3d* t) {
    Eigen::Vector3d pc0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d pw0 = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < points3D_.size(); ++i) {
        pc0 += pcs_[i];
        pw0 += points3D_[i];
    }
    pc0 /= points3D_.size();
    pw0 /= points3D_.size();

    Eigen::Matrix3d abt = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < points3D_.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            abt(j, 0) += (pcs_[i][j] - pc0[j]) * (points3D_[i][0] - pw0[0]);
            abt(j, 1) += (pcs_[i][j] - pc0[j]) * (points3D_[i][1] - pw0[1]);
            abt(j, 2) += (pcs_[i][j] - pc0[j]) * (points3D_[i][2] - pw0[2]);
        }
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            abt, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::Matrix3d abt_U = svd.matrixU();
    const Eigen::Matrix3d abt_V = svd.matrixV();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            (*R)(i, j) = abt_U.row(i) * abt_V.row(j).transpose();
        }
    }

    if (R->determinant() < 0) {
        Eigen::Matrix3d Abt_v_prime = abt_V;
        Abt_v_prime.col(2) = -abt_V.col(2);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                (*R)(i, j) = abt_U.row(i) * Abt_v_prime.row(j).transpose();
            }
        }
    }

    *t = pc0 - *R * pw0;
}

double EPnPEstimator::ComputeTotalReprojectionError(const Eigen::Matrix3d& R,
                                                    const Eigen::Vector3d& t) {
    Eigen::Matrix3x4d proj_matrix;
    proj_matrix.leftCols<3>() = R;
    proj_matrix.rightCols<1>() = t;

    std::vector<double> residuals;
    ComputeSquaredReprojectionError(points2D_, points3D_, proj_matrix,
                                    &residuals);

    double reproj_error = 0.0;
    for (const double residual : residuals) {
        reproj_error += std::sqrt(residual);
    }

    return reproj_error;
}


std::vector<EssentialMatrixFivePointEstimator::M_t>
EssentialMatrixFivePointEstimator::Estimate(const std::vector<X_t>& points1,
                                            const std::vector<Y_t>& points2) {
    Eigen::Matrix<double, Eigen::Dynamic, 9> Q(points1.size(), 9);
    for (size_t i = 0; i < points1.size(); ++i) {
        const double x1_0 = points1[i](0);
        const double x1_1 = points1[i](1);
        const double x2_0 = points2[i](0);
        const double x2_1 = points2[i](1);
        Q(i, 0) = x1_0 * x2_0;
        Q(i, 1) = x1_1 * x2_0;
        Q(i, 2) = x2_0;
        Q(i, 3) = x1_0 * x2_1;
        Q(i, 4) = x1_1 * x2_1;
        Q(i, 5) = x2_1;
        Q(i, 6) = x1_0;
        Q(i, 7) = x1_1;
        Q(i, 8) = 1;
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(
            Q, Eigen::ComputeFullV);
    Eigen::Matrix<double, 4, 9, Eigen::RowMajor> E =
            svd.matrixV().block<9, 4>(0, 5).transpose();

    Eigen::Matrix<double, 10, 20, Eigen::ColMajor> A;
#include "polygons.h"
    Eigen::Matrix<double, 10, 10> AA =
            A.block<10, 10>(0, 0).partialPivLu().solve(A.block<10, 10>(0, 10));

    Eigen::Matrix<double, 13, 3> B;
    Eigen::Matrix<double, 1, 13> B_row1, B_row2;
    B_row1(0, 0) = 0;
    B_row1(0, 4) = 0;
    B_row1(0, 8) = 0;
    B_row2(0, 3) = 0;
    B_row2(0, 7) = 0;
    B_row2(0, 12) = 0;
    for (size_t i = 0; i < 3; ++i) {
        B_row1.block<1, 3>(0, 1) = AA.block<1, 3>(i * 2 + 4, 0);
        B_row1.block<1, 3>(0, 5) = AA.block<1, 3>(i * 2 + 4, 3);
        B_row1.block<1, 4>(0, 9) = AA.block<1, 4>(i * 2 + 4, 6);
        B_row2.block<1, 3>(0, 0) = AA.block<1, 3>(i * 2 + 5, 0);
        B_row2.block<1, 3>(0, 4) = AA.block<1, 3>(i * 2 + 5, 3);
        B_row2.block<1, 4>(0, 8) = AA.block<1, 4>(i * 2 + 5, 6);
        B.col(i) = B_row1 - B_row2;
    }

    std::vector<double> coeffs(11);
#include "coefficients.h"

    std::vector<std::complex<double>> roots = SolvePolynomialN(coeffs);

    std::vector<M_t> models;

    const double kEps = 1e-10;

    for (size_t i = 0; i < roots.size(); ++i) {
        if (std::abs(roots[i].imag()) > kEps) {
            continue;
        }

        const double z1 = roots[i].real();
        const double z2 = z1 * z1;
        const double z3 = z2 * z1;
        const double z4 = z3 * z1;

        Eigen::Matrix3d Bz;
        for (size_t j = 0; j < 3; ++j) {
            const double* br = b + j * 13;
            Bz(j, 0) = br[0] * z3 + br[1] * z2 + br[2] * z1 + br[3];
            Bz(j, 1) = br[4] * z3 + br[5] * z2 + br[6] * z1 + br[7];
            Bz(j, 2) = br[8] * z4 + br[9] * z3 + br[10] * z2 + br[11] * z1 + br[12];
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(Bz, Eigen::ComputeFullV);
        const Eigen::Vector3d X = svd.matrixV().block<3, 1>(0, 2);

        if (std::abs(X(2)) < kEps) {
            continue;
        }

        Eigen::MatrixXd essential_vec = E.row(0) * (X(0) / X(2)) +
                                        E.row(1) * (X(1) / X(2)) + E.row(2) * z1 +
                                        E.row(3);

        const double inv_norm = 1.0 / essential_vec.norm();
        essential_vec *= inv_norm;

        essential_vec.resize(3, 3);
        const Eigen::Matrix3d essential_matrix = essential_vec.transpose();

        models.push_back(essential_matrix);
    }

    return models;
}

void EssentialMatrixFivePointEstimator::Residuals(
        const std::vector<X_t>& points1, const std::vector<Y_t>& points2,
        const M_t& E, std::vector<double>* residuals) {
    ComputeSquaredSampsonError(points1, points2, E, residuals);
}

std::vector<EssentialMatrixEightPointEstimator::M_t>
EssentialMatrixEightPointEstimator::Estimate(const std::vector<X_t>& points1,
                                             const std::vector<Y_t>& points2) {
    std::vector<X_t> normed_points1;
    std::vector<Y_t> normed_points2;
    Eigen::Matrix3d points1_norm_matrix;
    Eigen::Matrix3d points2_norm_matrix;
    CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
    CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

    Eigen::Matrix<double, Eigen::Dynamic, 9> cmatrix(points1.size(), 9);
    for (size_t i = 0; i < points1.size(); ++i) {
        cmatrix.block<1, 3>(i, 0) = normed_points1[i].homogeneous();
        cmatrix.block<1, 3>(i, 0) *= normed_points2[i].x();
        cmatrix.block<1, 3>(i, 3) = normed_points1[i].homogeneous();
        cmatrix.block<1, 3>(i, 3) *= normed_points2[i].y();
        cmatrix.block<1, 3>(i, 6) = normed_points1[i].homogeneous();
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> cmatrix_svd(
            cmatrix, Eigen::ComputeFullV);
    const Eigen::VectorXd ematrix_nullspace = cmatrix_svd.matrixV().col(8);
    const Eigen::Map<const Eigen::Matrix3d> ematrix_t(ematrix_nullspace.data());

    Eigen::JacobiSVD<Eigen::Matrix3d> ematrix_svd(
            ematrix_t.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d singular_values = ematrix_svd.singularValues();
    singular_values(0) = (singular_values(0) + singular_values(1)) / 2.0;
    singular_values(1) = singular_values(0);
    singular_values(2) = 0.0;
    const Eigen::Matrix3d E = ematrix_svd.matrixU() *
                              singular_values.asDiagonal() *
                              ematrix_svd.matrixV().transpose();

    const std::vector<M_t> models = {points2_norm_matrix.transpose() * E *
                                     points1_norm_matrix};
    return models;
}

void EssentialMatrixEightPointEstimator::Residuals(
        const std::vector<X_t>& points1, const std::vector<Y_t>& points2,
        const M_t& E, std::vector<double>* residuals) {
    ComputeSquaredSampsonError(points1, points2, E, residuals);
}


std::vector<FundamentalMatrixSevenPointEstimator::M_t>
FundamentalMatrixSevenPointEstimator::Estimate(
        const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
    Eigen::Matrix<double, 7, 9> A;
    for (size_t i = 0; i < 7; ++i) {
        const double x0 = points1[i](0);
        const double y0 = points1[i](1);
        const double x1 = points2[i](0);
        const double y1 = points2[i](1);
        A(i, 0) = x1 * x0;
        A(i, 1) = x1 * y0;
        A(i, 2) = x1;
        A(i, 3) = y1 * x0;
        A(i, 4) = y1 * y0;
        A(i, 5) = y1;
        A(i, 6) = x0;
        A(i, 7) = y0;
        A(i, 8) = 1;
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, 7, 9>> svd(A, Eigen::ComputeFullV);
    const Eigen::Matrix<double, 9, 9> f = svd.matrixV();
    Eigen::Matrix<double, 1, 9> f1 = f.col(7);
    Eigen::Matrix<double, 1, 9> f2 = f.col(8);

    f1 -= f2;

    const double t0 = f1(4) * f1(8) - f1(5) * f1(7);
    const double t1 = f1(3) * f1(8) - f1(5) * f1(6);
    const double t2 = f1(3) * f1(7) - f1(4) * f1(6);

    const double c0 = f1(0) * t0 - f1(1) * t1 + f1(2) * t2;

    const double c1 = f2(0) * t0 - f2(1) * t1 + f2(2) * t2 -
                      f2(3) * (f1(1) * f1(8) - f1(2) * f1(7)) +
                      f2(4) * (f1(0) * f1(8) - f1(2) * f1(6)) -
                      f2(5) * (f1(0) * f1(7) - f1(1) * f1(6)) +
                      f2(6) * (f1(1) * f1(5) - f1(2) * f1(4)) -
                      f2(7) * (f1(0) * f1(5) - f1(2) * f1(3)) +
                      f2(8) * (f1(0) * f1(4) - f1(1) * f1(3));

    const double t3 = f2(4) * f2(8) - f2(5) * f2(7);
    const double t4 = f2(3) * f2(8) - f2(5) * f2(6);
    const double t5 = f2(3) * f2(7) - f2(4) * f2(6);

    const double c2 = f1(0) * t3 - f1(1) * t4 + f1(2) * t5 -
                      f1(3) * (f2(1) * f2(8) - f2(2) * f2(7)) +
                      f1(4) * (f2(0) * f2(8) - f2(2) * f2(6)) -
                      f1(5) * (f2(0) * f2(7) - f2(1) * f2(6)) +
                      f1(6) * (f2(1) * f2(5) - f2(2) * f2(4)) -
                      f1(7) * (f2(0) * f2(5) - f2(2) * f2(3)) +
                      f1(8) * (f2(0) * f2(4) - f2(1) * f2(3));

    const double c3 = f2(0) * t3 - f2(1) * t4 + f2(2) * t5;

    const std::vector<double> roots = SolvePolynomial3(c0, c1, c2, c3);

    std::vector<M_t> models(roots.size());

    const double kEps = 1e-10;

    for (size_t i = 0; i < roots.size(); ++i) {
        const double lambda = roots[i];
        const double mu = 1;

        Eigen::MatrixXd F = lambda * f1 + mu * f2;

        F.resize(3, 3);

        if (std::abs(F(2, 2)) > kEps) {
            F /= F(2, 2);
        } else {
            F(2, 2) = 0;
        }

        models[i] = F.transpose();
    }

    return models;
}

void FundamentalMatrixSevenPointEstimator::Residuals(
        const std::vector<X_t>& points1, const std::vector<Y_t>& points2,
        const M_t& F, std::vector<double>* residuals) {
    ComputeSquaredSampsonError(points1, points2, F, residuals);
}

std::vector<FundamentalMatrixEightPointEstimator::M_t>
FundamentalMatrixEightPointEstimator::Estimate(
        const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
    std::vector<X_t> normed_points1;
    std::vector<Y_t> normed_points2;
    Eigen::Matrix3d points1_norm_matrix;
    Eigen::Matrix3d points2_norm_matrix;
    CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
    CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

    Eigen::Matrix<double, Eigen::Dynamic, 9> cmatrix(points1.size(), 9);
    for (size_t i = 0; i < points1.size(); ++i) {
        cmatrix.block<1, 3>(i, 0) = normed_points1[i].homogeneous();
        cmatrix.block<1, 3>(i, 0) *= normed_points2[i].x();
        cmatrix.block<1, 3>(i, 3) = normed_points1[i].homogeneous();
        cmatrix.block<1, 3>(i, 3) *= normed_points2[i].y();
        cmatrix.block<1, 3>(i, 6) = normed_points1[i].homogeneous();
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> cmatrix_svd(
            cmatrix, Eigen::ComputeFullV);
    const Eigen::VectorXd cmatrix_nullspace = cmatrix_svd.matrixV().col(8);
    const Eigen::Map<const Eigen::Matrix3d> ematrix_t(cmatrix_nullspace.data());

    Eigen::JacobiSVD<Eigen::Matrix3d> fmatrix_svd(
            ematrix_t.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d singular_values = fmatrix_svd.singularValues();
    singular_values(2) = 0.0;
    const Eigen::Matrix3d F = fmatrix_svd.matrixU() *
                              singular_values.asDiagonal() *
                              fmatrix_svd.matrixV().transpose();

    const std::vector<M_t> models = {points2_norm_matrix.transpose() * F *
                                     points1_norm_matrix};
    return models;
}

void FundamentalMatrixEightPointEstimator::Residuals(
        const std::vector<X_t>& points1, const std::vector<Y_t>& points2,
        const M_t& E, std::vector<double>* residuals) {
    ComputeSquaredSampsonError(points1, points2, E, residuals);
}


std::vector<HomographyMatrixEstimator::M_t> HomographyMatrixEstimator::Estimate(
        const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
    const size_t N = points1.size();

    std::vector<X_t> normed_points1;
    std::vector<Y_t> normed_points2;
    Eigen::Matrix3d points1_norm_matrix;
    Eigen::Matrix3d points2_norm_matrix;
    CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
    CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

    Eigen::Matrix<double, Eigen::Dynamic, 9> A = Eigen::MatrixXd::Zero(2 * N, 9);

    for (size_t i = 0, j = N; i < points1.size(); ++i, ++j) {
        const double s_0 = normed_points1[i](0);
        const double s_1 = normed_points1[i](1);
        const double d_0 = normed_points2[i](0);
        const double d_1 = normed_points2[i](1);

        A(i, 0) = -s_0;
        A(i, 1) = -s_1;
        A(i, 2) = -1;
        A(i, 6) = s_0 * d_0;
        A(i, 7) = s_1 * d_0;
        A(i, 8) = d_0;

        A(j, 3) = -s_0;
        A(j, 4) = -s_1;
        A(j, 5) = -1;
        A(j, 6) = s_0 * d_1;
        A(j, 7) = s_1 * d_1;
        A(j, 8) = d_1;
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(
            A, Eigen::ComputeFullV);

    const Eigen::VectorXd nullspace = svd.matrixV().col(8);
    Eigen::Map<const Eigen::Matrix3d> H_t(nullspace.data());

    const std::vector<M_t> models = {points2_norm_matrix.inverse() *
                                     H_t.transpose() * points1_norm_matrix};
    return models;
}

void HomographyMatrixEstimator::Residuals(const std::vector<X_t>& points1,
                                          const std::vector<Y_t>& points2,
                                          const M_t& H,
                                          std::vector<double>* residuals) {
    residuals->resize(points1.size());

    const double H_00 = H(0, 0);
    const double H_01 = H(0, 1);
    const double H_02 = H(0, 2);
    const double H_10 = H(1, 0);
    const double H_11 = H(1, 1);
    const double H_12 = H(1, 2);
    const double H_20 = H(2, 0);
    const double H_21 = H(2, 1);
    const double H_22 = H(2, 2);

    for (size_t i = 0; i < points1.size(); ++i) {
        const double s_0 = points1[i](0);
        const double s_1 = points1[i](1);
        const double d_0 = points2[i](0);
        const double d_1 = points2[i](1);

        const double pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
        const double pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
        const double pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

        const double inv_pd_2 = 1.0 / pd_2;
        const double dd_0 = d_0 - pd_0 * inv_pd_2;
        const double dd_1 = d_1 - pd_1 * inv_pd_2;

        (*residuals)[i] = dd_0 * dd_0 + dd_1 * dd_1;
    }
}


void TriangulationEstimator::SetMinTriAngle(const double min_tri_angle) {
    min_tri_angle_ = min_tri_angle;
}

void TriangulationEstimator::SetResidualType(const ResidualType residual_type) {
    residual_type_ = residual_type;
}

std::vector<TriangulationEstimator::M_t>
TriangulationEstimator::Estimate(const std::vector<X_t> &point_data,
                                 const std::vector<Y_t> &pose_data) const {

    if (point_data.size() == 2) {

        const M_t xyz = TriangulatePoint(
                pose_data[0].proj_matrix, pose_data[1].proj_matrix,
                point_data[0].point_normalized, point_data[1].point_normalized);

        if (HasPointPositiveDepth(pose_data[0].proj_matrix, xyz) &&
            HasPointPositiveDepth(pose_data[1].proj_matrix, xyz) &&
            CalculateTriangulationAngle(pose_data[0].proj_center,
                                        pose_data[1].proj_center,
                                        xyz) >= min_tri_angle_) {
            return std::vector<M_t>{xyz};
        }
    } else {

        std::vector<Eigen::Matrix3x4d> proj_matrices;
        proj_matrices.reserve(point_data.size());
        std::vector<Eigen::Vector2d> points;
        points.reserve(point_data.size());
        for (size_t i = 0; i < point_data.size(); ++i) {
            proj_matrices.push_back(pose_data[i].proj_matrix);
            points.push_back(point_data[i].point_normalized);
        }

        const M_t xyz = TriangulateMultiViewPoint(proj_matrices, points);

        for (const auto &pose : pose_data) {
            if (!HasPointPositiveDepth(pose.proj_matrix, xyz)) {
                return std::vector<M_t>();
            }
        }

        for (size_t i = 0; i < pose_data.size(); ++i) {
            for (size_t j = 0; j < i; ++j) {
                const double tri_angle = CalculateTriangulationAngle(
                        pose_data[i].proj_center, pose_data[j].proj_center, xyz);
                if (tri_angle >= min_tri_angle_) {
                    return std::vector<M_t>{xyz};
                }
            }
        }
    }

    return std::vector<M_t>();
}

void TriangulationEstimator::Residuals(const std::vector<X_t> &point_data,
                                       const std::vector<Y_t> &pose_data,
                                       const M_t &xyz,
                                       std::vector<double> *residuals) const {
    residuals->resize(point_data.size());

    for (size_t i = 0; i < point_data.size(); ++i) {
        if (HasPointPositiveDepth(pose_data[i].proj_matrix, xyz)) {
            if (residual_type_ == ResidualType::REPROJECTION_ERROR) {
                (*residuals)[i] = CalculateReprojectionError(point_data[i].point, xyz,
                                                             pose_data[i].proj_matrix,
                                                             *pose_data[i].camera);
            } else if (residual_type_ == ResidualType::ANGULAR_ERROR) {
                (*residuals)[i] = CalculateAngularError(point_data[i].point_normalized,
                                                        xyz, pose_data[i].proj_matrix);
            }
        } else {
            (*residuals)[i] = std::numeric_limits<double>::max();
        }
    }
}

bool EstimateTriangulation(
        const EstimateTriangulationOptions &options,
        const std::vector<TriangulationEstimator::PointData> &point_data,
        const std::vector<TriangulationEstimator::PoseData> &pose_data,
        std::vector<bool> *inlier_mask, Eigen::Vector3d *xyz) {
    options.Check();

    LORANSAC<TriangulationEstimator, TriangulationEstimator,
            InlierSupportMeasurer, CombinationSampler>
            ransac(options.ransac_options);
    ransac.estimator.SetMinTriAngle(options.min_tri_angle);
    ransac.estimator.SetResidualType(options.residual_type);
    const auto report = ransac.Estimate(point_data, pose_data);
    if (!report.success) {
        return false;
    }

    *inlier_mask = report.inlier_mask;
    *xyz = report.model;

    return report.success;
}
