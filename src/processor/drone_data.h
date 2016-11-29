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

#ifndef INC_3D_RECONSTRUCTION_DRONE_DATA_H
#define INC_3D_RECONSTRUCTION_DRONE_DATA_H

#include <stdio.h>
#include <unordered_map>

#include "../storage.h"
#include "../options.h"

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

    std::vector<double> camera_inner_params;    // f, cx, cy, k1, k2 (for RADIAL).
    int num_images;

    std::vector<ImageData> images;

private:

    const OptionManager options_;
    // Name of the file, which is stored in images_path and contains all additional data from the drone.
    const std::string DRONE_DATA_FILE = "drone_data.txt";
};

#endif //INC_3D_RECONSTRUCTION_DRONE_DATA_H
