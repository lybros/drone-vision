#ifndef INC_3D_RECONSTRUCTION_MODEL_MANAGER_WIDGET_H
#define INC_3D_RECONSTRUCTION_MODEL_MANAGER_WIDGET_H

#include "../reconstruction.h"

#include <QtWidgets>

class ModelManagerWidget : public QComboBox {
public:
    const static size_t kNewestModelIdx;

    ModelManagerWidget(QWidget* parent);

    size_t ModelIdx() const;

    void SetModelIdx(const size_t idx);

    void UpdateModels(const std::vector<std::unique_ptr<Reconstruction>>& models);

private:
    std::vector<size_t> model_idxs_;
};

#endif //INC_3D_RECONSTRUCTION_MODEL_MANAGER_WIDGET_H
