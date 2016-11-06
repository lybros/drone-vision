#include "surface_reconstruct.h"

SurfaceReconstructer::SurfaceReconstructer(const std::string& input_path, const std::string& binary_path)
        : stop_(false),
          input_path_(input_path),
          binary_path_(binary_path),
          result_(),
          successfull_(true) { }

void SurfaceReconstructer::Stop() {
    QMutexLocker locker(&mutex_);
    stop_ = true;
}

bool SurfaceReconstructer::IsSuccessfull() {
    QMutexLocker locker(&mutex_);
    return successfull_;
}

std::string SurfaceReconstructer::ResultFile() const {
    return result_;
}

void SurfaceReconstructer::run() {
    std::ofstream input;
    boost::filesystem::path input_path(input_path_);
    input.open((input_path / boost::filesystem::path("input.txt")).string().c_str());
    for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(input_path), {})) {
        size_t pos = entry.path().string().rfind(".");
        if (pos != std::string::npos && entry.path().string().substr(pos) == ".pset") {
            std::ifstream current_input;
            current_input.open(entry.path().string().c_str());
            std::string line;
            while (std::getline(current_input, line))
                input << line << std::endl;
            current_input.close();
        }
    }
    input.close();
    size_t pos = binary_path_.rfind("/");
    QString program = QString::fromStdString(binary_path_.substr(0, pos) + "/"
                                             + "poisson_triangulation/poisson_triangulation");
    QStringList arguments;
    arguments << "--in" << (input_path / boost::filesystem::path("input.txt")).string().c_str()
    << "--out" << (input_path / boost::filesystem::path("output")).string().c_str() << "--verbose";
    QProcess* reconstruction_process = new QProcess;
    std::cout << "Running " << program.toUtf8().constData() << " with mode poisson" << std::endl;
    WaitForProcess(reconstruction_process, program, arguments);
}

void SurfaceReconstructer::WaitForProcess(QProcess* child, const QString& program, const QStringList& params) {
    child->setProcessChannelMode(QProcess::ForwardedChannels);
    child->start(program, params);
    while (true) {
        {
            QMutexLocker locker(&mutex_);
            if (stop_) {
                std::cout << "Stopping surface reconstructing" << std::endl;
                child->terminate();
                break;
            }
        }
        if (child->waitForFinished(5000)) {
            if (child->exitCode() == 0) {
                QMutexLocker locker(&mutex_);
            }
            else {
                successfull_ = false;
                std::cerr << "Process exited with exit code " << child->exitCode() << std::endl;
            }
            break;
        }
    }
}
