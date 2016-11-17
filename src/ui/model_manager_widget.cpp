#include "model_manager_widget.h"

const size_t ModelManagerWidget::kNewestModelIdx =
        std::numeric_limits<size_t>::max();

ModelManagerWidget::ModelManagerWidget(QWidget* parent) : QComboBox(parent) {
    QFont font;
    font.setPointSize(10);
    setFont(font);
}

size_t ModelManagerWidget::ModelIdx() const {
    if (model_idxs_.empty()) {
        return kNewestModelIdx;
    } else {
        return model_idxs_[currentIndex()];
    }
}

void ModelManagerWidget::SetModelIdx(const size_t idx) {
    for (size_t i = 0; i < model_idxs_.size(); ++i) {
        if (model_idxs_[i] == idx) {
            blockSignals(true);
            setCurrentIndex(i);
            blockSignals(false);
        }
    }
}

void ModelManagerWidget::UpdateModels(
        const std::vector<std::unique_ptr<Reconstruction>>& models) {
    if (view()->isVisible()) {
        return;
    }

    blockSignals(true);

    const int prev_idx = currentIndex();
    const size_t prev_num_models = model_idxs_.size();

    clear();
    model_idxs_.clear();

    addItem("Newest model");
    model_idxs_.push_back(ModelManagerWidget::kNewestModelIdx);

    int max_width = 0;
    QFontMetrics font_metrics(view()->font());

    for (size_t i = 0; i < models.size(); ++i) {
        const QString item = QString().sprintf(
                "Model %d (%d images, %d points)", static_cast<int>(i + 1),
                static_cast<int>(models[i]->NumRegImages()),
                static_cast<int>(models[i]->NumPoints3D()));
        max_width = std::max(max_width, font_metrics.width(item));
        addItem(item);
        model_idxs_.push_back(i);
    }

    view()->setMinimumWidth(max_width);

    if (prev_num_models <= 0 || models.size() == 0) {
        setCurrentIndex(0);
    } else {
        setCurrentIndex(prev_idx);
    }
    blockSignals(false);
}

