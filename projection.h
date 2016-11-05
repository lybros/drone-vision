#ifndef INC_3D_RECONSTRUCTION_PROJECTION_H
#define INC_3D_RECONSTRUCTION_PROJECTION_H

#include "entities.h"

#include <Eigen/Core>

#include <ceres/solver.h>

bool CheckCheirality(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                     const std::vector<Eigen::Vector2d>& points1,
                     const std::vector<Eigen::Vector2d>& points2,
                     std::vector<Eigen::Vector3d>* points3D);


Eigen::Vector2d ProjectPointToImage(const Eigen::Vector3d& point3D,
                                    const Eigen::Matrix3x4d& proj_matrix,
                                    const Camera& camera);

double CalculateReprojectionError(const Eigen::Vector2d& point2D,
                                  const Eigen::Vector3d& point3D,
                                  const Eigen::Matrix3x4d& proj_matrix,
                                  const Camera& camera);

double CalculateAngularError(const Eigen::Vector2d& point2D,
                             const Eigen::Vector3d& point3D,
                             const Eigen::Matrix3x4d& proj_matrix,
                             const Camera& camera);

Eigen::Vector3d TriangulateOptimalPoint(const Eigen::Matrix3x4d& proj_matrix1,
                                        const Eigen::Matrix3x4d& proj_matrix2,
                                        const Eigen::Vector2d& point1,
                                        const Eigen::Vector2d& point2);

void DecomposeEssentialMatrix(const Eigen::Matrix3d& E, Eigen::Matrix3d* R1,
                              Eigen::Matrix3d* R2, Eigen::Vector3d* t);

void PoseFromEssentialMatrix(const Eigen::Matrix3d& E,
                             const std::vector<Eigen::Vector2d>& points1,
                             const std::vector<Eigen::Vector2d>& points2,
                             Eigen::Matrix3d* R, Eigen::Vector3d* t,
                             std::vector<Eigen::Vector3d>* points3D);

Eigen::Matrix3d EssentialMatrixFromPose(const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& t);

Eigen::Matrix3d EssentialMatrixFromAbsolutePoses(
        const Eigen::Matrix3x4d& proj_matrix1,
        const Eigen::Matrix3x4d& proj_matrix2);

void FindOptimalImageObservations(const Eigen::Matrix3d& E,
                                  const Eigen::Vector2d& point1,
                                  const Eigen::Vector2d& point2,
                                  Eigen::Vector2d* optimal_point1,
                                  Eigen::Vector2d* optimal_point2);

Eigen::Vector3d EpipoleFromEssentialMatrix(const Eigen::Matrix3d& E,
                                           const bool left_image);

Eigen::Matrix3d InvertEssentialMatrix(const Eigen::Matrix3d& matrix);


void DecomposeHomographyMatrix(const Eigen::Matrix3d& H,
                               const Eigen::Matrix3d& K1,
                               const Eigen::Matrix3d& K2,
                               std::vector<Eigen::Matrix3d>* R,
                               std::vector<Eigen::Vector3d>* t,
                               std::vector<Eigen::Vector3d>* n);

void PoseFromHomographyMatrix(const Eigen::Matrix3d& H,
                              const Eigen::Matrix3d& K1,
                              const Eigen::Matrix3d& K2,
                              const std::vector<Eigen::Vector2d>& points1,
                              const std::vector<Eigen::Vector2d>& points2,
                              Eigen::Matrix3d* R, Eigen::Vector3d* t,
                              Eigen::Vector3d* n,
                              std::vector<Eigen::Vector3d>* points3D);

Eigen::Matrix3d HomographyMatrixFromPose(const Eigen::Matrix3d& K1,
                                         const Eigen::Matrix3d& K2,
                                         const Eigen::Matrix3d& R,
                                         const Eigen::Vector3d& t,
                                         const Eigen::Vector3d& n,
                                         const double d);


#endif //INC_3D_RECONSTRUCTION_PROJECTION_H
