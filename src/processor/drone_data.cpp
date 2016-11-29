#include "drone_data.h"

DroneData::DroneData(const OptionManager& options) : options_(options) {}

bool DroneData::Read() {
    std::string filename = EnsureTrailingSlash(*options_.image_path) + DroneData::DRONE_DATA_FILE;
    std::ifstream file(filename);

    // Reading inner camera parameters.
    std::string camera_inner_params;
    std::getline(file, camera_inner_params);
    this->camera_inner_params = std::vector<double>(3);
    std::sscanf(camera_inner_params.c_str(), "%lf %lf %lf", &this->camera_inner_params[0],
                &this->camera_inner_params[1], &this->camera_inner_params[2]);

    // Reading number of images in a set.
    std::string image_number_line;
    std::getline(file, image_number_line);
    std::sscanf(image_number_line.c_str(), "%d", &num_images);
    images = std::vector<ImageData>(num_images);

    // Reading every image.
    for (int i = 0; i < num_images; i++) {
        std::string image_line;
        std::getline(file, image_line);
        images[i].rotation_params = new double[3];
        images[i].translation_params = new double[3];
        images[i].gps_data = new double[3];
        char name[100];
        std::sscanf(image_line.c_str(), "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                    name,
                    &images[i].rotation_params[0], &images[i].rotation_params[1], &images[i].rotation_params[2],
                    &images[i].translation_params[0], &images[i].translation_params[1],
                    &images[i].translation_params[2],
                    &images[i].gps_data[0], &images[i].gps_data[1], &images[i].gps_data[2]);
        images[i].image_name = std::string(name);
    }

    file.close();

    return true;
}

int DroneData::MatchWithDatabase(DatabaseCache* database_cache) {
    std::unordered_map<image_t, Image> db_images = database_cache->Images();
    int matched = 0;
    for (int i = 0; i < images.size(); i++) {
        ImageData& image = images[i];
        for (auto db_image : db_images) {
            if (db_image.second.Name() == image.image_name) {
                image.image_db_id = db_image.first;
                matched++;
                break;
            }
        }
    }
    return matched;
}

void DroneData::Print() {
    PrintHeading1("PRINTING DRONE DATA");

    std::cout << "Camera params: (" << camera_inner_params.size() << ")" << std::endl;
    for (auto param : camera_inner_params) {
        std::cout << param << " ";
    }
    std::cout << std::endl;

    std::cout << "Image params: (" << images.size() << ")" << std::endl;
    for (auto image : images) {
        std::cout << "  Image: " << image.image_name << std::endl;
        std::cout << "    in database as: " << image.image_db_id << std::endl;
        std::cout << "    Rotation params: " << image.rotation_params[0] << " "
                  << image.rotation_params[1] << " "
                  << image.rotation_params[2] << std::endl;
        std::cout << "    Translation params: " << image.translation_params[0] << " "
                  << image.translation_params[1] << " "
                  << image.translation_params[2] << std::endl;
        std::cout << "    GPS data: " << image.gps_data[0] << " "
                  << image.gps_data[1] << " "
                  << image.gps_data[2] << std::endl;
    }

    PrintHeading1("END OF PRINT");
}