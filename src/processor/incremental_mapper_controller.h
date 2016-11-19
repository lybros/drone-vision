#ifndef INC_3D_RECONSTRUCTION_CONTROLLERS_H
#define INC_3D_RECONSTRUCTION_CONTROLLERS_H

#include "../options.h"

#include <memory>
#include <iostream>

#include <boost/format.hpp>

#include <QtCore/QThread>
#include <QtWidgets/QAction>
#include <QtCore/QMutex>
#include <QtCore/QWaitCondition>

class IncrementalMapperController : public QThread {
public:
    IncrementalMapperController(const OptionManager& options);

    IncrementalMapperController(const OptionManager& options, class Reconstruction* initial_model);

    void run();

    void Stop();

    void Pause();

    void Resume();

    bool IsRunning();

    bool IsStarted();

    bool IsPaused();

    bool IsFinished();

    size_t NumModels() const;

    const std::vector<std::unique_ptr<Reconstruction>>& Models() const;

    const Reconstruction& Model(const size_t idx) const;

    Reconstruction& Model(const size_t idx);

    size_t AddModel();

    QAction* action_render;
    QAction* action_render_now;
    QAction* action_finish;

private:
    void Render();

    void RenderNow();

    void Finish();

    bool terminate_;
    bool pause_;
    bool running_;
    bool started_;
    bool finished_;

    QMutex control_mutex_;
    QWaitCondition pause_condition_;

    const OptionManager options_;

    std::vector<std::unique_ptr<class Reconstruction>> models_;
};

#endif //INC_3D_RECONSTRUCTION_CONTROLLERS_H
