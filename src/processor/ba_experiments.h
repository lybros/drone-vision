//
// Class to run experiments on Bundle Adjustment problem.
//

#ifndef INC_3D_RECONSTRUCTION_BA_EXPERIMENTAL_H
#define INC_3D_RECONSTRUCTION_BA_EXPERIMENTAL_H

#include "../storage.h"
#include "../options.h"

class BAExperiments {
public:
    BAExperiments(const OptionManager& options);

    void RunSimpleCeresImplementation();

private:
    const OptionManager options_;
};

#endif //INC_3D_RECONSTRUCTION_BA_EXPERIMENTAL_H
