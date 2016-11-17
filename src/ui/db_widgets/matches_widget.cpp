#include "matches_widget.h"

MatchesTab::MatchesTab(QWidget* parent, OptionManager* options,
                       Database* database)
        : QWidget(parent),
          options_(options),
          database_(database),
          matches_viewer_(new MatchesImageViewerWidget(parent)) { }

void MatchesTab::Clear() {
    table_widget_->clearContents();
    matches_.clear();
    configs_.clear();
    sorted_matches_idxs_.clear();
}

void MatchesTab::InitializeTable(const QStringList& table_header) {
    QGridLayout* grid = new QGridLayout(this);

    info_label_ = new QLabel(this);
    grid->addWidget(info_label_, 0, 0);

    QPushButton* show_button = new QPushButton(tr("Show matches"), this);
    connect(show_button, &QPushButton::released, this, &MatchesTab::ShowMatches);
    grid->addWidget(show_button, 0, 1, Qt::AlignRight);

    table_widget_ = new QTableWidget(this);
    table_widget_->setColumnCount(table_header.size());
    table_widget_->setHorizontalHeaderLabels(table_header);

    table_widget_->setShowGrid(true);
    table_widget_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_widget_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_widget_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_widget_->horizontalHeader()->setStretchLastSection(true);
    table_widget_->verticalHeader()->setVisible(false);
    table_widget_->verticalHeader()->setDefaultSectionSize(20);

    grid->addWidget(table_widget_, 1, 0, 1, 2);
}

void MatchesTab::ShowMatches() {
    QItemSelectionModel* select = table_widget_->selectionModel();

    if (!select->hasSelection()) {
        QMessageBox::critical(this, "", tr("No image pair selected."));
        return;
    }

    if (select->selectedRows().size() > 1) {
        QMessageBox::critical(this, "", tr("Only one image pair may be selected."));
        return;
    }

    const size_t idx =
            sorted_matches_idxs_[select->selectedRows().begin()->row()];
    const auto& selection = matches_[idx];
    const std::string path1 =
            EnsureTrailingSlash(*options_->image_path) + image_->Name();
    const std::string path2 =
            EnsureTrailingSlash(*options_->image_path) + selection.first->Name();
    const auto keypoints1 = database_->ReadKeypoints(image_->ImageId());
    const auto keypoints2 = database_->ReadKeypoints(selection.first->ImageId());

    matches_viewer_->Show(path1, path2, keypoints1, keypoints2, selection.second);

    matches_viewer_->setWindowTitle(QString::fromStdString(
            "Matches for image pair " + std::to_string(image_->ImageId()) + " - " +
            std::to_string(selection.first->ImageId())));

    matches_viewer_->show();
    matches_viewer_->raise();
}

void MatchesTab::FillTable() {
    sorted_matches_idxs_.resize(matches_.size());
    std::iota(sorted_matches_idxs_.begin(), sorted_matches_idxs_.end(), 0);

    std::sort(sorted_matches_idxs_.begin(), sorted_matches_idxs_.end(),
              [&](const size_t idx1, const size_t idx2) {
                  return matches_[idx1].second.size() >
                         matches_[idx2].second.size();
              });

    QString info;
    info += QString("Matched images: ") + QString::number(matches_.size());
    info_label_->setText(info);

    table_widget_->clearContents();
    table_widget_->setRowCount(matches_.size());

    for (size_t i = 0; i < sorted_matches_idxs_.size(); ++i) {
        const size_t idx = sorted_matches_idxs_[i];

        QTableWidgetItem* image_id_item =
                new QTableWidgetItem(QString::number(matches_[idx].first->ImageId()));
        table_widget_->setItem(i, 0, image_id_item);

        QTableWidgetItem* num_matches_item =
                new QTableWidgetItem(QString::number(matches_[idx].second.size()));
        table_widget_->setItem(i, 1, num_matches_item);

        // config for inlier matches tab
        if (table_widget_->columnCount() == 3) {
            QTableWidgetItem* config_item =
                    new QTableWidgetItem(QString::number(configs_[idx]));
            table_widget_->setItem(i, 2, config_item);
        }
    }

    table_widget_->resizeColumnsToContents();
}

RawMatchesTab::RawMatchesTab(QWidget* parent, OptionManager* options,
                             Database* database)
        : MatchesTab(parent, options, database) {
    QStringList table_header;
    table_header << "image_id"
                 << "num_matches";
    InitializeTable(table_header);
}

void RawMatchesTab::Update(const std::vector<Image>& images,
                           const image_t image_id) {
    matches_.clear();

    for (const auto& image : images) {
        if (image.ImageId() == image_id) {
            image_ = &image;
            continue;
        }

        if (database_->ExistsMatches(image_id, image.ImageId())) {
            const auto matches = database_->ReadMatches(image_id, image.ImageId());

            if (matches.size() > 0) {
                matches_.emplace_back(&image, matches);
            }
        }
    }

    FillTable();
}

InlierMatchesTab::InlierMatchesTab(QWidget* parent, OptionManager* options,
                                   Database* database)
        : MatchesTab(parent, options, database) {
    QStringList table_header;
    table_header << "image_id"
                 << "num_matches"
                 << "config";
    InitializeTable(table_header);
}

void InlierMatchesTab::Update(const std::vector<Image>& images,
                              const image_t image_id) {
    matches_.clear();
    configs_.clear();

    for (const auto& image : images) {
        if (image.ImageId() == image_id) {
            image_ = &image;
            continue;
        }

        if (database_->ExistsInlierMatches(image_id, image.ImageId())) {
            const auto two_view_geometry =
                    database_->ReadInlierMatches(image_id, image.ImageId());

            if (two_view_geometry.inlier_matches.size() > 0) {
                matches_.emplace_back(&image, two_view_geometry.inlier_matches);
                configs_.push_back(two_view_geometry.config);
            }
        }
    }

    FillTable();
}

MatchesWidget::MatchesWidget(QWidget* parent, OptionManager* options,
                             Database* database)
        : parent_(parent), options_(options) {
    setWindowFlags(Qt::Window);
    resize(parent->size().width() - 20, parent->size().height() - 20);

    QGridLayout* grid = new QGridLayout(this);

    tab_widget_ = new QTabWidget(this);

    raw_matches_tab_ = new RawMatchesTab(this, options_, database);
    tab_widget_->addTab(raw_matches_tab_, tr("Raw matches"));

    inlier_matches_tab_ = new InlierMatchesTab(this, options_, database);
    tab_widget_->addTab(inlier_matches_tab_, tr("Inlier matches"));

    grid->addWidget(tab_widget_, 0, 0);

    QPushButton* close_button = new QPushButton(tr("Close"), this);
    connect(close_button, &QPushButton::released, this, &MatchesWidget::close);
    grid->addWidget(close_button, 1, 0, Qt::AlignRight);
}

void MatchesWidget::ShowMatches(const std::vector<Image>& images,
                                const image_t image_id) {
    parent_->setDisabled(true);

    setWindowTitle(
            QString::fromStdString("Matches for image " + std::to_string(image_id)));

    raw_matches_tab_->Update(images, image_id);
    inlier_matches_tab_->Update(images, image_id);
}

void MatchesWidget::closeEvent(QCloseEvent* event) {
    raw_matches_tab_->Clear();
    inlier_matches_tab_->Clear();
    parent_->setEnabled(true);
}

