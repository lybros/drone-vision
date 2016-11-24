//
// Class to run experiments on Bundle Adjustment problem.
//
// Primary statements:
// 1. To use the data from our database.
// 2. Assuming all photos were made by the same camera, so all inner parameters are the same.
//     Rotation & translation are not.
// 3. Ways of estimating 3d points: * the same approach with IncrementalMapper.
//                                  * using GPS data either retrieved from the photo itself or from other file.
// 4. Ways of estimating camera params: * the same approach with IncrementalMapper.
//                                      * using magic file which knows rotation etc. (from drone detectors).
// 5. To check if it's possible to make CERES use GPU and if that makes any sense.

#ifndef INC_3D_RECONSTRUCTION_BA_EXPERIMENTAL_H
#define INC_3D_RECONSTRUCTION_BA_EXPERIMENTAL_H

#include <iostream>

#include "../storage.h"
#include "../options.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

class BAExperiments {
public:
    BAExperiments(const OptionManager& options);

    void RunSimpleCeresImplementation();

private:
    void LoadDatabaseCache();
    void BuildCeresProblem(ceres::Problem&);

    const OptionManager options_;
    DatabaseCache* database_cache_;
};

#endif //INC_3D_RECONSTRUCTION_BA_EXPERIMENTAL_H
