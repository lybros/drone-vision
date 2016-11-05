#include "projection.h"

bool CheckCheirality(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                     const std::vector<Eigen::Vector2d>& points1,
                     const std::vector<Eigen::Vector2d>& points2,
                     std::vector<Eigen::Vector3d>* points3D) {
    const Eigen::Matrix3x4d proj_matrix1 = Eigen::Matrix3x4d::Identity();
    const Eigen::Matrix3x4d proj_matrix2 = ComposeProjectionMatrix(R, t);
    const double kMinDepth = std::numeric_limits<double>::epsilon();
    const double max_depth = 1000.0f * (R.transpose() * t).norm();
    points3D->clear();
    for (size_t i = 0; i < points1.size(); ++i) {
        const Eigen::Vector3d point3D =
                TriangulatePoint(proj_matrix1, proj_matrix2, points1[i], points2[i]);
        const double depth1 = CalculateDepth(proj_matrix1, point3D);
        if (depth1 > kMinDepth && depth1 < max_depth) {
            const double depth2 = CalculateDepth(proj_matrix2, point3D);
            if (depth2 > kMinDepth && depth2 < max_depth) {
                points3D->push_back(point3D);
            }
        }
    }
    return !points3D->empty();
}

Eigen::Vector2d ProjectPointToImage(const Eigen::Vector3d& point3D,
                                    const Eigen::Matrix3x4d& proj_matrix,
                                    const Camera& camera) {
    const Eigen::Vector3d world_point = proj_matrix * point3D.homogeneous();
    return camera.WorldToImage(world_point.hnormalized());
}

double CalculateReprojectionError(const Eigen::Vector2d& point2D,
                                  const Eigen::Vector3d& point3D,
                                  const Eigen::Matrix3x4d& proj_matrix,
                                  const Camera& camera) {
    const auto image_point = ProjectPointToImage(point3D, proj_matrix, camera);
    return (image_point - point2D).norm();
}

double CalculateAngularError(const Eigen::Vector2d& point2D,
                             const Eigen::Vector3d& point3D,
                             const Eigen::Matrix3x4d& proj_matrix,
                             const Camera& camera) {
    return CalculateAngularError(camera.ImageToWorld(point2D), point3D,
                                 proj_matrix);
}


Eigen::Vector3d TriangulateOptimalPoint(const Eigen::Matrix3x4d& proj_matrix1,
                                        const Eigen::Matrix3x4d& proj_matrix2,
                                        const Eigen::Vector2d& point1,
                                        const Eigen::Vector2d& point2) {
    const Eigen::Matrix3d E =
            EssentialMatrixFromAbsolutePoses(proj_matrix1, proj_matrix2);

    Eigen::Vector2d optimal_point1;
    Eigen::Vector2d optimal_point2;
    FindOptimalImageObservations(E, point1, point2, &optimal_point1,
                                 &optimal_point2);

    return TriangulatePoint(proj_matrix1, proj_matrix2, optimal_point1,
                            optimal_point2);
}


void DecomposeEssentialMatrix(const Eigen::Matrix3d& E, Eigen::Matrix3d* R1,
                              Eigen::Matrix3d* R2, Eigen::Vector3d* t) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV().transpose();

    if (U.determinant() < 0) {
        U *= -1;
    }
    if (V.determinant() < 0) {
        V *= -1;
    }

    Eigen::Matrix3d W;
    W << 0, 1, 0, -1, 0, 0, 0, 0, 1;

    *R1 = U * W * V;
    *R2 = U * W.transpose() * V;
    *t = U.col(2).normalized();
}

void PoseFromEssentialMatrix(const Eigen::Matrix3d& E,
                             const std::vector<Eigen::Vector2d>& points1,
                             const std::vector<Eigen::Vector2d>& points2,
                             Eigen::Matrix3d* R, Eigen::Vector3d* t,
                             std::vector<Eigen::Vector3d>* points3D) {
    Eigen::Matrix3d R1;
    Eigen::Matrix3d R2;
    DecomposeEssentialMatrix(E, &R1, &R2, t);

    const std::array<Eigen::Matrix3d, 4> R_cmbs{{R1, R2, R1, R2}};
    const std::array<Eigen::Vector3d, 4> t_cmbs{{*t, *t, -*t, -*t}};

    points3D->clear();
    for (size_t i = 0; i < R_cmbs.size(); ++i) {
        std::vector<Eigen::Vector3d> points3D_cmb;
        CheckCheirality(R_cmbs[i], t_cmbs[i], points1, points2, &points3D_cmb);
        if (points3D_cmb.size() >= points3D->size()) {
            *R = R_cmbs[i];
            *t = t_cmbs[i];
            *points3D = points3D_cmb;
        }
    }
}

Eigen::Matrix3d EssentialMatrixFromPose(const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& t) {
    assert(t.norm() - 1.0 < std::numeric_limits<double>::epsilon());
    Eigen::Matrix3d t_x;
    t_x << 0, -t(2), t(1), t(2), 0, -t(0), -t(1), t(0), 0;
    return t_x * R;
}

Eigen::Matrix3d EssentialMatrixFromAbsolutePoses(
        const Eigen::Matrix3x4d& proj_matrix1,
        const Eigen::Matrix3x4d& proj_matrix2) {
    const Eigen::Matrix3d R1 = proj_matrix1.leftCols<3>();
    const Eigen::Matrix3d R2 = proj_matrix2.leftCols<3>();
    const Eigen::Vector3d t1 = proj_matrix1.rightCols<1>();
    const Eigen::Vector3d t2 = proj_matrix2.rightCols<1>();

    const Eigen::Matrix3d R = R1.transpose() * R2;
    const Eigen::Vector3d t = (R1.transpose() * (t2 - t1)).normalized();

    return EssentialMatrixFromPose(R, t);
}

void FindOptimalImageObservations(const Eigen::Matrix3d& E,
                                  const Eigen::Vector2d& point1,
                                  const Eigen::Vector2d& point2,
                                  Eigen::Vector2d* optimal_point1,
                                  Eigen::Vector2d* optimal_point2) {
    const Eigen::Vector3d& point1h = point1.homogeneous();
    const Eigen::Vector3d& point2h = point2.homogeneous();

    Eigen::Matrix<double, 2, 3> S;
    S << 1, 0, 0, 0, 1, 0;

    Eigen::Vector2d n1 = S * E * point2h;
    Eigen::Vector2d n2 = S * E.transpose() * point1h;

    const Eigen::Matrix2d E_tilde = E.block<2, 2>(0, 0);

    const double a = n1.transpose() * E_tilde * n2;
    const double b = (n1.squaredNorm() + n2.squaredNorm()) / 2.0;
    const double c = point1h.transpose() * E * point2h;
    const double d = sqrt(b * b - a * c);
    double lambda = c / (b + d);

    n1 -= E_tilde * lambda * n1;
    n2 -= E_tilde.transpose() * lambda * n2;

    lambda *= (2.0 * d) / (n1.squaredNorm() + n2.squaredNorm());

    *optimal_point1 = (point1h - S.transpose() * lambda * n1).hnormalized();
    *optimal_point2 = (point2h - S.transpose() * lambda * n2).hnormalized();
}

Eigen::Vector3d EpipoleFromEssentialMatrix(const Eigen::Matrix3d& E,
                                           const bool left_image) {
    Eigen::Vector3d e;
    if (left_image) {
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullV);
        e = svd.matrixV().block<3, 1>(0, 2);
    } else {
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(E.transpose(), Eigen::ComputeFullV);
        e = svd.matrixV().block<3, 1>(0, 2);
    }
    return e;
}

Eigen::Matrix3d InvertEssentialMatrix(const Eigen::Matrix3d& E) {
    return E.transpose();
}


namespace {
    double ComputeOppositeOfMinor(const Eigen::Matrix3d& matrix, const size_t row,
                                  const size_t col) {
        const size_t col1 = col == 0 ? 1 : 0;
        const size_t col2 = col == 2 ? 1 : 2;
        const size_t row1 = row == 0 ? 1 : 0;
        const size_t row2 = row == 2 ? 1 : 2;
        return (matrix(row1, col2) * matrix(row2, col1) -
                matrix(row1, col1) * matrix(row2, col2));
    }

    Eigen::Matrix3d ComputeHomographyRotation(const Eigen::Matrix3d& hmatrix_norm,
                                              const Eigen::Vector3d& tstar,
                                              const Eigen::Vector3d& n,
                                              const double v) {
        return hmatrix_norm *
               (Eigen::Matrix3d::Identity() - (2.0 / v) * tstar * n.transpose());
    }

}

void DecomposeHomographyMatrix(const Eigen::Matrix3d& H,
                               const Eigen::Matrix3d& K1,
                               const Eigen::Matrix3d& K2,
                               std::vector<Eigen::Matrix3d>* R,
                               std::vector<Eigen::Vector3d>* t,
                               std::vector<Eigen::Vector3d>* n) {
    Eigen::Matrix3d hmatrix_norm = K2.inverse() * H * K1;

    Eigen::JacobiSVD<Eigen::Matrix3d> hmatrix_norm_svd(hmatrix_norm);
    hmatrix_norm.array() /= hmatrix_norm_svd.singularValues()[1];

    const Eigen::Matrix3d S =
            hmatrix_norm.transpose() * hmatrix_norm - Eigen::Matrix3d::Identity();

    const double kMinInfinityNorm = 1e-3;
    if (S.lpNorm<Eigen::Infinity>() < kMinInfinityNorm) {
        *R = {hmatrix_norm};
        *t = {Eigen::Vector3d::Zero()};
        *n = {Eigen::Vector3d::Zero()};
        return;
    }

    const double M00 = ComputeOppositeOfMinor(S, 0, 0);
    const double M11 = ComputeOppositeOfMinor(S, 1, 1);
    const double M22 = ComputeOppositeOfMinor(S, 2, 2);

    const double rtM00 = std::sqrt(M00);
    const double rtM11 = std::sqrt(M11);
    const double rtM22 = std::sqrt(M22);

    const double M01 = ComputeOppositeOfMinor(S, 0, 1);
    const double M12 = ComputeOppositeOfMinor(S, 1, 2);
    const double M02 = ComputeOppositeOfMinor(S, 0, 2);

    const int e12 = SignOfNumber(M12);
    const int e02 = SignOfNumber(M02);
    const int e01 = SignOfNumber(M01);

    const double nS00 = std::abs(S(0, 0));
    const double nS11 = std::abs(S(1, 1));
    const double nS22 = std::abs(S(2, 2));

    const std::array<double, 3> nS{{nS00, nS11, nS22}};
    const size_t idx =
            std::distance(nS.begin(), std::max_element(nS.begin(), nS.end()));

    Eigen::Vector3d np1;
    Eigen::Vector3d np2;
    if (idx == 0) {
        np1[0] = S(0, 0);
        np2[0] = S(0, 0);
        np1[1] = S(0, 1) + rtM22;
        np2[1] = S(0, 1) - rtM22;
        np1[2] = S(0, 2) + e12 * rtM11;
        np2[2] = S(0, 2) - e12 * rtM11;
    } else if (idx == 1) {
        np1[0] = S(0, 1) + rtM22;
        np2[0] = S(0, 1) - rtM22;
        np1[1] = S(1, 1);
        np2[1] = S(1, 1);
        np1[2] = S(1, 2) - e02 * rtM00;
        np2[2] = S(1, 2) + e02 * rtM00;
    } else if (idx == 2) {
        np1[0] = S(0, 2) + e01 * rtM11;
        np2[0] = S(0, 2) - e01 * rtM11;
        np1[1] = S(1, 2) + rtM00;
        np2[1] = S(1, 2) - rtM00;
        np1[2] = S(2, 2);
        np2[2] = S(2, 2);
    }

    const double traceS = S.trace();
    const double v = 2.0 * std::sqrt(1.0 + traceS - M00 - M11 - M22);

    const double ESii = SignOfNumber(S(idx, idx));
    const double r_2 = 2 + traceS + v;
    const double nt_2 = 2 + traceS - v;

    const double r = std::sqrt(r_2);
    const double n_t = std::sqrt(nt_2);

    const Eigen::Vector3d n1 = np1.normalized();
    const Eigen::Vector3d n2 = np2.normalized();

    const double half_nt = 0.5 * n_t;
    const double esii_t_r = ESii * r;

    const Eigen::Vector3d t1_star = half_nt * (esii_t_r * n2 - n_t * n1);
    const Eigen::Vector3d t2_star = half_nt * (esii_t_r * n1 - n_t * n2);

    const Eigen::Matrix3d R1 =
            ComputeHomographyRotation(hmatrix_norm, t1_star, n1, v);
    const Eigen::Vector3d t1 = R1 * t1_star;

    const Eigen::Matrix3d R2 =
            ComputeHomographyRotation(hmatrix_norm, t2_star, n2, v);
    const Eigen::Vector3d t2 = R2 * t2_star;

    *R = {R1, R1, R2, R2};
    *t = {t1, -t1, t2, -t2};
    *n = {-n1, n1, -n2, n2};
}

void PoseFromHomographyMatrix(const Eigen::Matrix3d& H,
                              const Eigen::Matrix3d& K1,
                              const Eigen::Matrix3d& K2,
                              const std::vector<Eigen::Vector2d>& points1,
                              const std::vector<Eigen::Vector2d>& points2,
                              Eigen::Matrix3d* R, Eigen::Vector3d* t,
                              Eigen::Vector3d* n,
                              std::vector<Eigen::Vector3d>* points3D) {
    std::vector<Eigen::Matrix3d> R_cmbs;
    std::vector<Eigen::Vector3d> t_cmbs;
    std::vector<Eigen::Vector3d> n_cmbs;
    DecomposeHomographyMatrix(H, K1, K2, &R_cmbs, &t_cmbs, &n_cmbs);

    points3D->clear();
    for (size_t i = 0; i < R_cmbs.size(); ++i) {
        std::vector<Eigen::Vector3d> points3D_cmb;
        CheckCheirality(R_cmbs[i], t_cmbs[i], points1, points2, &points3D_cmb);
        if (points3D_cmb.size() >= points3D->size()) {
            *R = R_cmbs[i];
            *t = t_cmbs[i];
            *n = n_cmbs[i];
            *points3D = points3D_cmb;
        }
    }
}

Eigen::Matrix3d HomographyMatrixFromPose(const Eigen::Matrix3d& K1,
                                         const Eigen::Matrix3d& K2,
                                         const Eigen::Matrix3d& R,
                                         const Eigen::Vector3d& t,
                                         const Eigen::Vector3d& n,
                                         const double d) {
    return K2 * (R - t * n.normalized().transpose() / d) * K1.inverse();
}
