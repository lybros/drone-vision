// Let's define the format of the file, which contains additional data from drone.
// Depending on which sensors and detectors do we have on the drone, data set can vary.
//
// drone_data.txt - this file keeps the following data:
// * Assuming we have one physical camera.
// * Every image stores quaternion which defines camera rotation and translation vector.
//
// <focal_length> <cx> <cy> <second_camera_distortion> <fourth_camera_distortion>
// <num_images>
// <image_name_0> <4d quaternion vector> <3d translation vector (lat, lon, alt)>
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
        Eigen::Vector4d qvec;
        Eigen::Vector3d tvec;
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
