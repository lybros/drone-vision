#ifndef INC_3D_RECONSTRUCTION_SURFACE_RECONSTRUCT_H
#define INC_3D_RECONSTRUCTION_SURFACE_RECONSTRUCT_H

#include "utils.h"

#include <string>
#include <fstream>

#include <QtCore/QThread>
#include <QtCore/QProcess>
#include <QtCore/QMutex>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

class SurfaceReconstructer : public QThread {
public:
    SurfaceReconstructer(const std::string& input_path, const std::string& binary_path);
    void Stop();
    bool IsSuccessfull();
    std::string ResultFile() const;

protected:
    void run();

    void WaitForProcess(QProcess *child, const QString &program, const QStringList &params);

    QMutex mutex_;

    bool stop_;

    std::string input_path_;
    std::string binary_path_;
    std::string result_;
    bool successfull_;
};

#endif //INC_3D_RECONSTRUCTION_SURFACE_RECONSTRUCT_H
