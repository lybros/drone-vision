#ifndef INC_3D_RECONSTRUCTION_MATCHES_WIDGET_H
#define INC_3D_RECONSTRUCTION_MATCHES_WIDGET_H

#include "database_management_widget.h"

#include <QtWidgets>

class MatchesImageViewerWidget;

class MatchesTab : public QWidget {
public:
    MatchesTab() { }

    MatchesTab(QWidget* parent, OptionManager* options, Database* database);

    void Clear();

protected:
    void InitializeTable(const QStringList& table_header);

    void ShowMatches();

    void FillTable();

    OptionManager* options_;
    Database* database_;

    const Image* image_;
    std::vector<std::pair<const Image*, FeatureMatches>> matches_;
    std::vector<int> configs_;
    std::vector<size_t> sorted_matches_idxs_;

    QTableWidget* table_widget_;
    QLabel* info_label_;
    MatchesImageViewerWidget* matches_viewer_;
};

class RawMatchesTab : public MatchesTab {
public:
    RawMatchesTab(QWidget* parent, OptionManager* options, Database* database);

    void Update(const std::vector<Image>& images, const image_t image_id);
};

class InlierMatchesTab : public MatchesTab {
public:
    InlierMatchesTab(QWidget* parent, OptionManager* options, Database* database);

    void Update(const std::vector<Image>& images, const image_t image_id);
};

class MatchesWidget : public QWidget {
public:
    MatchesWidget(QWidget* parent, OptionManager* options, Database* database);

    void ShowMatches(const std::vector<Image>& images, const image_t image_id);

private:
    void closeEvent(QCloseEvent* event);

    QWidget* parent_;

    OptionManager* options_;

    QTabWidget* tab_widget_;
    RawMatchesTab* raw_matches_tab_;
    InlierMatchesTab* inlier_matches_tab_;
};

#endif //INC_3D_RECONSTRUCTION_MATCHES_WIDGET_H
