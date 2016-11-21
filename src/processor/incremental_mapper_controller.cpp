#include "incremental_mapper_controller.h"

namespace {
    size_t TriangulateImage(const MapperOptions& options, const Image& image, IncrementalMapper* mapper) {
        std::cout << "  => Continued observations: " << image.NumPoints3D() << std::endl;
        const size_t num_tris = mapper->TriangulateImage(options.TriangulationOptions(), image.ImageId());
        std::cout << "  => Added observations: " << num_tris << std::endl;
        return num_tris;
    }

    size_t CompleteAndMergeTracks(const MapperOptions& options, IncrementalMapper* mapper) {
        const size_t num_completed_observations = mapper->CompleteTracks(options.TriangulationOptions());
        std::cout << "  => Merged observations: " << num_completed_observations << std::endl;
        const size_t num_merged_observations = mapper->MergeTracks(options.TriangulationOptions());
        std::cout << "  => Completed observations: " << num_merged_observations << std::endl;
        return num_completed_observations + num_merged_observations;
    }

    size_t FilterPoints(const MapperOptions& options, IncrementalMapper* mapper) {
        const size_t num_filtered_observations = mapper->FilterPoints(options.IncrementalMapperOptions());
        std::cout << "  => Filtered observations: " << num_filtered_observations << std::endl;
        return num_filtered_observations;
    }

    size_t FilterImages(const MapperOptions& options, IncrementalMapper* mapper) {
        const size_t num_filtered_images = mapper->FilterImages(options.IncrementalMapperOptions());
        std::cout << "  => Filtered images: " << num_filtered_images << std::endl;
        return num_filtered_images;
    }

    void AdjustGlobalBundle(const MapperOptions& options,
                            const Reconstruction& reconstruction,
                            IncrementalMapper* mapper) {
        BundleAdjuster::Options custom_options = options.GlobalBundleAdjustmentOptions();

        const size_t num_reg_images = reconstruction.NumRegImages();

        const size_t kMinNumRegImages = 10;
        // Setting custom options if we have not many images registered (so we've just started our reconstruction).
        if (num_reg_images < kMinNumRegImages) {
            custom_options.solver_options.function_tolerance /= 10;
            custom_options.solver_options.gradient_tolerance /= 10;
            custom_options.solver_options.parameter_tolerance /= 10;
            custom_options.solver_options.max_num_iterations *= 2;
            custom_options.solver_options.max_linear_solver_iterations = 200;
        }

        PrintHeading1("Global bundle adjustment");
        // AdjustGlobalBundle() sets up a problem which is then solved using ceres.
        mapper->AdjustGlobalBundle(custom_options);
    }

    void IterativeLocalRefinement(const MapperOptions& options,
                                  const image_t image_id,
                                  IncrementalMapper* mapper) {
        auto ba_options = options.LocalBundleAdjustmentOptions();
        for (int i = 0; i < options.ba_local_max_refinements; ++i) {
            const auto report = mapper->AdjustLocalBundle(
                    options.IncrementalMapperOptions(), ba_options,
                    options.TriangulationOptions(), image_id);
            std::cout << "  => Merged observations: " << report.num_merged_observations << std::endl;
            std::cout << "  => Completed observations: " << report.num_completed_observations << std::endl;
            std::cout << "  => Filtered observations: " << report.num_filtered_observations << std::endl;
            const double changed = (report.num_merged_observations + report.num_completed_observations +
                                    report.num_filtered_observations) /
                                   static_cast<double>(report.num_adjusted_observations);
            std::cout << "  => Changed observations: " << changed << std::endl;
            if (changed < options.ba_local_max_refinement_change) {
                break;
            }
            ba_options.loss_function_type = BundleAdjuster::Options::LossFunctionType::TRIVIAL;
        }
    }

    void IterativeGlobalRefinement(const MapperOptions& options,
                                   const Reconstruction& reconstruction,
                                   IncrementalMapper* mapper) {
        PrintHeading1("Retriangulation");
        CompleteAndMergeTracks(options, mapper);
        std::cout << "  => Retriangulated observations: "
                  << mapper->Retriangulate(options.TriangulationOptions())
                  << std::endl;

        for (int i = 0; i < options.ba_global_max_refinements; ++i) {
            const size_t num_observations = reconstruction.ComputeNumObservations();
            size_t num_changed_observations = 0;
            AdjustGlobalBundle(options, reconstruction, mapper);
            num_changed_observations += CompleteAndMergeTracks(options, mapper);
            num_changed_observations += FilterPoints(options, mapper);
            const double changed = static_cast<double>(num_changed_observations) / num_observations;
            std::cout << "  => Changed observations: " << changed << std::endl;
            if (changed < options.ba_global_max_refinement_change) {
                break;
            }
        }

        FilterImages(options, mapper);
    }

    void ExtractColors(const std::string& image_path, const image_t image_id, Reconstruction* reconstruction) {
        if (!reconstruction->ExtractColors(image_id, image_path)) {
            std::cout << QString().sprintf(
                    "WARNING: Could not read image %s at path %s.",
                    reconstruction->Image(image_id).Name().c_str(), image_path.c_str()
            ).toStdString() << std::endl;
        }
    }

}

IncrementalMapperController::IncrementalMapperController(const OptionManager& options)
        : action_render(nullptr),
          action_render_now(nullptr),
          action_finish(nullptr),
          terminate_(false),
          pause_(false),
          running_(false),
          started_(false),
          finished_(false),
          options_(options) {}

IncrementalMapperController::IncrementalMapperController(
        const OptionManager& options, class Reconstruction* initial_model)
        : IncrementalMapperController(options) {
    models_.emplace_back(initial_model);
}

void IncrementalMapperController::Stop() {
    {
        QMutexLocker control_locker(&control_mutex_);
        terminate_ = true;
        running_ = false;
        finished_ = true;
    }
    Resume();
}

void IncrementalMapperController::Pause() {
    QMutexLocker control_locker(&control_mutex_);
    if (pause_) {
        return;
    }
    pause_ = true;
    running_ = false;
}

void IncrementalMapperController::Resume() {
    QMutexLocker control_locker(&control_mutex_);
    if (!pause_) {
        return;
    }
    pause_ = false;
    running_ = true;
    pause_condition_.wakeAll();
}

bool IncrementalMapperController::IsRunning() {
    QMutexLocker control_locker(&control_mutex_);
    return running_;
}

bool IncrementalMapperController::IsStarted() {
    QMutexLocker control_locker(&control_mutex_);
    return started_;
}

bool IncrementalMapperController::IsPaused() {
    QMutexLocker control_locker(&control_mutex_);
    return pause_;
}

bool IncrementalMapperController::IsFinished() { return finished_; }

size_t IncrementalMapperController::AddModel() {
    const size_t model_idx = models_.size();
    models_.emplace_back(new class Reconstruction());
    return model_idx;
}

void IncrementalMapperController::Render() {
    {
        QMutexLocker control_locker(&control_mutex_);
        if (terminate_) {
            return;
        }
    }

    if (action_render != nullptr) {
        action_render->trigger();
    }
}

void IncrementalMapperController::RenderNow() {
    {
        QMutexLocker control_locker(&control_mutex_);
        if (terminate_) {
            return;
        }
    }

    if (action_render_now != nullptr) {
        action_render_now->trigger();
    }
}

void IncrementalMapperController::Finish() {
    {
        QMutexLocker control_locker(&control_mutex_);
        running_ = false;
        finished_ = true;
        if (terminate_) {
            return;
        }
    }

    if (action_finish != nullptr) {
        action_finish->trigger();
    }
}

// Here is the process of building a cloud of points starts.
void IncrementalMapperController::run() {
    if (IsRunning()) {
        exit(0);
    }

    {
        QMutexLocker control_locker(&control_mutex_);
        terminate_ = false;
        pause_ = false;
        running_ = true;
        started_ = true;
        finished_ = false;
    }

    const MapperOptions& mapper_options = *options_.mapper_options;

    Timer total_timer;
    total_timer.Start();

    PrintHeading1("Loading database");

    DatabaseCache database_cache;

    {
        Database database;
        database.Open(*options_.database_path);
        Timer timer;
        timer.Start();
        const size_t min_num_matches = static_cast<size_t>(mapper_options.min_num_matches);
        database_cache.Load(database, min_num_matches, mapper_options.ignore_watermarks);
        std::cout << std::endl;
        timer.PrintMinutes();
    }

    std::cout << std::endl;

    // Creating an IncrementalMapper object, which is linked with our cached database.
    IncrementalMapper mapper(&database_cache);

    // Checking if we already have any models.
    const bool initial_model_given = !models_.empty();

    // mapper_options.init_num_trials is set to 200 by default in MapperOptions::Reset().
    // It's not clear due to how long this loop is, but everything happens inside the loop.
    // Everything we have from the "outer world" is mapper initialized with database cache and options_.
    for (int num_trials = 0; num_trials < mapper_options.init_num_trials; ++num_trials) {
        {
            QMutexLocker control_locker(&control_mutex_);
            if (pause_ && !terminate_) {
                total_timer.Pause();
                pause_condition_.wait(&control_mutex_);
                total_timer.Resume();
            } else if (terminate_) {
                break;
            }
        }

        // Creating new model if we don't have any or that's not the first trial.
        // The model will be added to models_ vector.
        // Every model is an object of Reconstruction class.
        if (!initial_model_given || num_trials > 0) {
            AddModel();
        }

        // Choosing the model from the model_ vector.
        const size_t model_idx = initial_model_given ? 0 : NumModels() - 1;
        Reconstruction& reconstruction = Model(model_idx);
        // BeginReconstruction seems to be only initializing before a real reconstruction.
        // That's the only place where BeginReconstruction is called.
        mapper.BeginReconstruction(&reconstruction);

        // In case we don't have any registered images yet.
        if (reconstruction.NumRegImages() == 0) {
            image_t image_id1, image_id2;

            // mapper_options.init_image_id1 and mapper_options.init_image_id2 are -1 by default.
            image_id1 = static_cast<image_t>(mapper_options.init_image_id1);
            image_id2 = static_cast<image_t>(mapper_options.init_image_id2);

            if (mapper_options.init_image_id1 == -1 || mapper_options.init_image_id2 == -1) {
                // Default values for init_image_ids means we're at the very beginning of the reconstruction.
                // Searching for initial pair of images to proceed.
                const bool find_init_success = mapper.FindInitialImagePair(
                        mapper_options.IncrementalMapperOptions(), &image_id1, &image_id2);

                // In case we haven't found any good pairs to begin the reconstruction - it ends.
                // So just switching everything off.
                if (!find_init_success) {
                    std::cerr << "  => Could not find good initial pair." << std::endl;
                    const bool kDiscardReconstruction = true;
                    mapper.EndReconstruction(kDiscardReconstruction);
                    models_.pop_back();
                    break;
                }
            }

            // Congrats! We have an initializing pair to proceed!
            PrintHeading1("Initializing with images #" + std::to_string(image_id1) +
                          " and #" + std::to_string(image_id2));

            // Whatever that means - if registering initial pair fails - we are stopping the process as well.
            // But in practice this function never returns 'false' (or rarely or I'm missing something).
            // So the point is that function found a track and a 3d point using two images we have and added
            // it into reconstruction.
            const bool reg_init_success = mapper.RegisterInitialImagePair(
                    mapper_options.IncrementalMapperOptions(), image_id1, image_id2);
            if (!reg_init_success) {
                std::cout << "  => Initialization failed." << std::endl;
                break;
            }

            // Finally something about Bundle Adjustment. So here we're setting up a problem using the registered images,
            // than ceres solves it. Report is printed. At the very end reconstruction is normalized.
            AdjustGlobalBundle(mapper_options, reconstruction, &mapper);

            // Filtering out 3d points with large reprojection error and small triangulation error.
            // Result is then printed as "Filtered observations:".
            FilterPoints(mapper_options, &mapper);

            // Filtering out and deregistering images using some mapper options.
            // Result is then printed as "Filtered imaged".
            FilterImages(mapper_options, &mapper);

            if (reconstruction.NumRegImages() == 0 || reconstruction.NumPoints3D() == 0) {
                // Initial values haven't given us much - removing a model and trying again.
                const bool kDiscardReconstruction = true;
                mapper.EndReconstruction(kDiscardReconstruction);
                models_.pop_back();
                continue;
            }

            if (mapper_options.extract_colors) {
                // Making points colourful by retrieving colors from images themselves.
                ExtractColors(*options_.image_path, image_id1, &reconstruction);
            }
        }

        // We have something to render, that's awesome!
        // In fact, calling RenderNow() means calling MainWindow::RenderNow(). Then it chooses a model to render, etc.
        // Let's just assume it renders it fine.
        RenderNow();

        size_t prev_num_reg_images = reconstruction.NumRegImages();
        size_t prev_num_points = reconstruction.NumPoints3D();
        int num_global_bas = 1;

        bool reg_next_success = true;

        // Iteratively registering new images till we can do that successfully.
        while (reg_next_success) {
            {
                QMutexLocker control_locker(&control_mutex_);
                if (pause_) {
                    total_timer.Pause();
                    pause_condition_.wait(&control_mutex_);
                    total_timer.Resume();
                }
                if (terminate_) {
                    break;
                }
            }

            reg_next_success = false;

            // And now we're trying to do something pretty similar to what we were doing in the half of the method.
            // The difference is between choosing 1st and 2nd images and adding more and more images to what we
            // already have.

            // next_images is a list of images, ordered by rank, which are candidated to be appended to the model.
            const std::vector<image_t> next_images = mapper.FindNextImages(mapper_options.IncrementalMapperOptions());

            if (next_images.empty()) {
                // No images can be added to the reconstruction.
                break;
            }

            for (size_t reg_trial = 0; reg_trial < next_images.size(); ++reg_trial) {
                // Trying to add every image from the list we have.
                const image_t next_image_id = next_images[reg_trial];
                const Image& next_image = reconstruction.Image(next_image_id);

                PrintHeading1("Processing image #" + std::to_string(next_image_id) +
                              " (" + std::to_string(reconstruction.NumRegImages() + 1) + ")");

                std::cout << "  => Image sees " << next_image.NumVisiblePoints3D()
                          << " / " << next_image.NumObservations() << " points." << std::endl;

                // RegusterNextImage() is really huge. And compilicated. Lot's of things are happening inside.
                // But for now it would be enough to know that inside we have a lot of checks if an image
                // can be added to the reconstructions and HOW that's possible; there're estimations which also
                // tells in the end, if image is good enough for appending.
                // If image passes all the obstructions - it's added to the reconstruction.
                reg_next_success = mapper.RegisterNextImage(
                        mapper_options.IncrementalMapperOptions(), next_image_id);

                if (reg_next_success) {
                    // So image is successfully added. That's not the end. We have to suffer more.

                    // Greeting IncrementalTriangulator!
                    // As for now let's assume it makes the world a better place.
                    TriangulateImage(mapper_options, next_image, &mapper);

                    // I thought we're close to the end - but here we're bundle-adjusting again! What a surprise!
                    // Some observations are going to be merged, some are filtered out.
                    // So here's we're making a model better. Makes sense to understand how.
                    IterativeLocalRefinement(mapper_options, next_image_id, &mapper);

                    if (reconstruction.NumRegImages() >= mapper_options.ba_global_images_ratio * prev_num_reg_images ||
                        reconstruction.NumRegImages() >= mapper_options.ba_global_images_freq + prev_num_reg_images ||
                        reconstruction.NumPoints3D() >= mapper_options.ba_global_points_ratio * prev_num_points ||
                        reconstruction.NumPoints3D() >= mapper_options.ba_global_points_freq + prev_num_points) {
                        // Not the easiest condition. So if it's true we're doing some magic with tracks (merging
                        // some of them, completing them) and running bundle adjustment again. Just in case, I guess.
                        // Because we can.
                        IterativeGlobalRefinement(mapper_options, reconstruction, &mapper);
                        prev_num_points = reconstruction.NumPoints3D();
                        prev_num_reg_images = reconstruction.NumRegImages();
                        num_global_bas += 1;
                    }

                    // Again: extracting colors for new image added from the image data.
                    // (will fail if the path to images is broken).
                    if (mapper_options.extract_colors) {
                        ExtractColors(*options_.image_path, next_image_id, &reconstruction);
                    }

                    // Take a look on MainWindow::Render().
                    Render();

                    break;
                } else {
                    // Trying with another image if the one we've used failed.
                    std::cout << "  => Could not register, trying another image." << std::endl;

                    const size_t kMinNumInitialRegTrials = 30;
                    if (reg_trial >= kMinNumInitialRegTrials &&
                        reconstruction.NumRegImages() < static_cast<size_t>(mapper_options.min_model_size)) {
                        break;
                    }
                }
            }

            // mapper_options.max_model_overlap is 20 by default. Does that mean 20 is max number of images which
            // can be added to the reconstruction? Depends on what are the "Shared Registered Images".
            const size_t max_model_overlap = static_cast<size_t>(mapper_options.max_model_overlap);
            if (mapper.NumSharedRegImages() >= max_model_overlap) {
                break;
            }
        }

        // The first non-conditional end of reconstruction.
        {
            QMutexLocker control_locker(&control_mutex_);
            if (terminate_) {
                const bool kDiscardReconstruction = false;
                mapper.EndReconstruction(kDiscardReconstruction);
                break;
            }
        }

        // Not sure why this could happen, seems like we're doing that in case. Well, why not.
        // Global refining the model.
        if (reconstruction.NumRegImages() >= 2 &&
            reconstruction.NumRegImages() != prev_num_reg_images &&
            reconstruction.NumPoints3D() != prev_num_points) {
            IterativeGlobalRefinement(mapper_options, reconstruction, &mapper);
        }

        const size_t min_model_size =
                std::min(database_cache.NumImages(), static_cast<size_t>(mapper_options.min_model_size));
        if ((mapper_options.multiple_models && reconstruction.NumRegImages() < min_model_size) ||
            reconstruction.NumRegImages() == 0) {
            const bool kDiscardReconstruction = true;
            mapper.EndReconstruction(kDiscardReconstruction);
            models_.pop_back();
        } else {
            const bool kDiscardReconstruction = false;
            mapper.EndReconstruction(kDiscardReconstruction);
            RenderNow();
        }

        const size_t max_num_models = static_cast<size_t>(mapper_options.max_num_models);
        if (initial_model_given || !mapper_options.multiple_models ||
            models_.size() >= max_num_models ||
            mapper.NumTotalRegImages() >= database_cache.NumImages() - 1) {
            break;
        }
    }

    std::cout << std::endl;

    total_timer.PrintMinutes();

    RenderNow();
    Finish();

    exit(0);
}

size_t IncrementalMapperController::NumModels() const { return models_.size(); }

const std::vector<std::unique_ptr<Reconstruction>>& IncrementalMapperController::Models() const { return models_; }

const Reconstruction& IncrementalMapperController::Model(const size_t idx) const { return *models_.at(idx); }

Reconstruction& IncrementalMapperController::Model(const size_t idx) { return *models_.at(idx); }
