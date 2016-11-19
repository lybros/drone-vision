#ifndef INC_3D_RECONSTRUCTION_DENSIFY_H
#define INC_3D_RECONSTRUCTION_DENSIFY_H

#include "reconstruction.h"

#include <QtCore/QThread>
#include <QtCore/QMutex>
#include <QtCore/QProcess>


class ImageDensifier : public QThread {
public:
    ImageDensifier(const Reconstruction& reconstruction, const std::string& image_path, const std::string& output_path,
                   const std::string& binary_path);

    void Stop();

    bool IsSuccessfull();

    bool IsRunning();

    std::vector<std::string> ResultFiles() const;

protected:
    void run();

    void WaitForProcess(QProcess* child, const QString& program, const QStringList& params);

    QMutex mutex_;

    bool stop_;

    std::string image_path_;
    std::string output_path_;
    std::string binary_path_;
    std::vector<std::string> results_;
    bool successfull_;
    const Reconstruction& reconstruction_;
};

Camera UndistortCamera(const Camera& camera);

void UndistortImage(const Bitmap& distorted_image,
                    const Camera& distorted_camera, Bitmap* undistorted_image,
                    Camera* undistorted_camera);

#endif //INC_3D_RECONSTRUCTION_DENSIFY_H
