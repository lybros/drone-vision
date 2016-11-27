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

// Let's define the format of the file, which contains additional data from drone.
// Depending on which sensors and detectors do we have on the drone, data set can vary.
//
// .drone_data0 - this extension keeps the following data:
// * Assuming we have one physical camera.
//
// <focal_length> <second_camera_distortion> <fourth_camera_distortion>
// <num_images>
// <image_name_0> <3 camera rotation params> <3 camera translation params> <lat, lon, alt>
// ...
// <image_name_num_images> ...

#ifndef INC_3D_RECONSTRUCTION_BA_EXPERIMENTAL_H
#define INC_3D_RECONSTRUCTION_BA_EXPERIMENTAL_H

#include <iostream>
#include <stdio.h>
#include <unordered_map>

#include "../storage.h"
#include "../options.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

class DroneData {
public:
    DroneData(const OptionManager& options);

    struct ImageData {
        std::string image_name;
        image_t image_db_id;
        double* rotation_params;    // 3 params.
        double* translation_params; // 3 params.
        double* gps_data;           // lat, lon, alt.
    };

    bool Read();
    void Print();

    // Returns a number of images read matched.
    int MatchWithDatabase(DatabaseCache*);

    std::vector<double> camera_inner_params;    // focal length, 2 distortions.
    int num_images;

    std::vector<ImageData> images;

private:

    const OptionManager options_;
    // Name of the file, which is stored in images_path and contains all additional data from the drone.
    const std::string DRONE_DATA_FILE = "drone_data.txt";
};

class BAExperiments {
public:
    BAExperiments(const OptionManager& options);

    void RunSimpleCeresImplementation();

private:
    void LoadDatabaseCache();
    void BuildCeresProblem(ceres::Problem&);

    const OptionManager options_;
    DatabaseCache* database_cache_;
    DroneData* drone_data_;
    double** approx_3d;
};

#endif //INC_3D_RECONSTRUCTION_BA_EXPERIMENTAL_H
