#include "ui/main_window.h"
#include "../3rdparty/pmvs/pmvs/option.h"
#include "../3rdparty/pmvs/pmvs/findMatch.h"
#include "../3rdparty/pmvs/cmvs/bundle.h"

int main(int argc, char** argv) {
    Q_INIT_RESOURCE(resources);

    QApplication app(argc, argv);
    std::string binary_path = QCoreApplication::applicationFilePath().toUtf8().constData();

    std::string mode = "app";
    if (argc > 1) {
        mode = argv[1];
    }

    if (argc == 1 || mode == "app") {
        MainWindow main_window(binary_path);
        main_window.show();
        return app.exec();
    } else if (mode == "experiment") {
        if (argc != 7) {
            std::cerr << "Cannot run experiment mode with number of arguments != 7, sorry." << std::endl;
        }
        std::string use_drone_data = argv[2];
        std::string use_qvec_tvec_estimations = argv[3];
        std::string refine_focal_length = argv[4];
        std::string refine_principal_point = argv[5];
        std::string refine_extra_params = argv[6];

        MainWindow main_window(binary_path);
        main_window.SetExperimentalFlags(use_drone_data == "1",
                                         use_qvec_tvec_estimations == "1",
                                         refine_focal_length == "1",
                                         refine_principal_point == "1",
                                         refine_extra_params == "1");
        main_window.show();
        return app.exec();
    } else if (mode == "pmvs") {
        std::cerr << "Get arguments " << argv[2] << " " << argv[3] << std::endl;
        PMVS3::Soption option;
        option.init(argv[2], argv[3]);

        PMVS3::CfindMatch findMatch;
        findMatch.init(option);
        findMatch.run();

        char buffer[1024];
        sprintf(buffer, "%smodels/%s", argv[2], argv[3]);
        findMatch.write(buffer, true, true, true);
        return 0;
    } else if (mode == "cmvs") {
        int maximage = 100;
        if (argc >= 4)
            maximage = atoi(argv[3]);

        int CPU = 4;
        if (argc >= 5)
            CPU = atoi(argv[4]);

        const float scoreRatioThreshold = 0.7f;
        const float coverageThreshold = 0.7f;

        const int iNumForScore = 4;
        const int pnumThreshold = 0;
        CMVS::Cbundle bundle;
        bundle.run(argv[2], maximage, iNumForScore,
                   scoreRatioThreshold, coverageThreshold,
                   pnumThreshold, CPU);
        return 0;
    }
    return 0;
}
