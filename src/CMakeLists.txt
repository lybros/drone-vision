add_subdirectory(entities)
add_subdirectory(processor)
add_subdirectory(ui)

add_library(3d_reconstruction_src
        main.cpp
        options.h options.cpp
        features.h features.cpp
        utils.h utils.cpp
        projection.h projection.cpp
        estimators.h estimators.cpp
        optimization.h optimization.cpp
        reconstruction.h reconstruction.cpp
        storage.h storage.cpp
        geometry.h geometry.cpp
        refinement.h refinement.cpp
        densify.h densify.cpp
        surface_reconstruct.h surface_reconstruct.cpp)

target_link_libraries(3d_reconstruction_src
        entities processor ui)
