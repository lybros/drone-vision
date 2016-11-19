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
        if (num_reg_images < kMinNumRegImages) {
            custom_options.solver_options.function_tolerance /= 10;
            custom_options.solver_options.gradient_tolerance /= 10;
            custom_options.solver_options.parameter_tolerance /= 10;
            custom_options.solver_options.max_num_iterations *= 2;
            custom_options.solver_options.max_linear_solver_iterations = 200;
        }

        PrintHeading1("Global bundle adjustment");
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

    void ExtractColors(const std::string& image_path, const image_t image_id,
                       Reconstruction* reconstruction) {
        if (!reconstruction->ExtractColors(image_id, image_path)) {
            std::cout << boost::format("WARNING: Could not read image %s at path %s.") %
                         reconstruction->Image(image_id).Name() % image_path
                      << std::endl;
        }
    }

}

IncrementalMapperController::IncrementalMapperController(
        const OptionManager& options)
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

    IncrementalMapper mapper(&database_cache);

    const bool initial_model_given = !models_.empty();

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

        if (!initial_model_given || num_trials > 0) {
            AddModel();
        }

        const size_t model_idx = initial_model_given ? 0 : NumModels() - 1;
        Reconstruction& reconstruction = Model(model_idx);
        mapper.BeginReconstruction(&reconstruction);

        if (reconstruction.NumRegImages() == 0) {
            image_t image_id1, image_id2;

            image_id1 = static_cast<image_t>(mapper_options.init_image_id1);
            image_id2 = static_cast<image_t>(mapper_options.init_image_id2);

            if (mapper_options.init_image_id1 == -1 ||
                mapper_options.init_image_id2 == -1) {
                const bool find_init_success = mapper.FindInitialImagePair(
                        mapper_options.IncrementalMapperOptions(), &image_id1, &image_id2);

                if (!find_init_success) {
                    std::cerr << "  => Could not find good initial pair." << std::endl;
                    const bool kDiscardReconstruction = true;
                    mapper.EndReconstruction(kDiscardReconstruction);
                    models_.pop_back();
                    break;
                }
            }

            PrintHeading1("Initializing with images #" + std::to_string(image_id1) +
                          " and #" + std::to_string(image_id2));
            const bool reg_init_success = mapper.RegisterInitialImagePair(
                    mapper_options.IncrementalMapperOptions(), image_id1, image_id2);

            if (!reg_init_success) {
                std::cout << "  => Initialization failed." << std::endl;
                break;
            }

            AdjustGlobalBundle(mapper_options, reconstruction, &mapper);
            FilterPoints(mapper_options, &mapper);
            FilterImages(mapper_options, &mapper);

            if (reconstruction.NumRegImages() == 0 ||
                reconstruction.NumPoints3D() == 0) {
                const bool kDiscardReconstruction = true;
                mapper.EndReconstruction(kDiscardReconstruction);
                models_.pop_back();
                continue;
            }

            if (mapper_options.extract_colors) {
                ExtractColors(*options_.image_path, image_id1, &reconstruction);
            }
        }

        RenderNow();

        size_t prev_num_reg_images = reconstruction.NumRegImages();
        size_t prev_num_points = reconstruction.NumPoints3D();
        int num_global_bas = 1;

        bool reg_next_success = true;

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

            const std::vector<image_t> next_images =
                    mapper.FindNextImages(mapper_options.IncrementalMapperOptions());

            if (next_images.empty()) {
                break;
            }

            for (size_t reg_trial = 0; reg_trial < next_images.size(); ++reg_trial) {
                const image_t next_image_id = next_images[reg_trial];
                const Image& next_image = reconstruction.Image(next_image_id);

                PrintHeading1("Processing image #" + std::to_string(next_image_id) +
                              " (" + std::to_string(reconstruction.NumRegImages() + 1) +
                              ")");

                std::cout << "  => Image sees " << next_image.NumVisiblePoints3D()
                          << " / " << next_image.NumObservations() << " points."
                          << std::endl;

                reg_next_success = mapper.RegisterNextImage(
                        mapper_options.IncrementalMapperOptions(), next_image_id);

                if (reg_next_success) {
                    TriangulateImage(mapper_options, next_image, &mapper);
                    IterativeLocalRefinement(mapper_options, next_image_id, &mapper);

                    if (reconstruction.NumRegImages() >=
                        mapper_options.ba_global_images_ratio * prev_num_reg_images ||
                        reconstruction.NumRegImages() >=
                        mapper_options.ba_global_images_freq + prev_num_reg_images ||
                        reconstruction.NumPoints3D() >=
                        mapper_options.ba_global_points_ratio * prev_num_points ||
                        reconstruction.NumPoints3D() >=
                        mapper_options.ba_global_points_freq + prev_num_points) {
                        IterativeGlobalRefinement(mapper_options, reconstruction, &mapper);
                        prev_num_points = reconstruction.NumPoints3D();
                        prev_num_reg_images = reconstruction.NumRegImages();
                        num_global_bas += 1;
                    }

                    if (mapper_options.extract_colors) {
                        ExtractColors(*options_.image_path, next_image_id, &reconstruction);
                    }

                    Render();

                    break;
                } else {
                    std::cout << "  => Could not register, trying another image." << std::endl;

                    const size_t kMinNumInitialRegTrials = 30;
                    if (reg_trial >= kMinNumInitialRegTrials &&
                        reconstruction.NumRegImages() <
                        static_cast<size_t>(mapper_options.min_model_size)) {
                        break;
                    }
                }
            }

            const size_t max_model_overlap = static_cast<size_t>(mapper_options.max_model_overlap);
            if (mapper.NumSharedRegImages() >= max_model_overlap) {
                break;
            }
        }

        {
            QMutexLocker control_locker(&control_mutex_);
            if (terminate_) {
                const bool kDiscardReconstruction = false;
                mapper.EndReconstruction(kDiscardReconstruction);
                break;
            }
        }

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

size_t IncrementalMapperController::NumModels() const {
    return models_.size();
}

const std::vector<std::unique_ptr<Reconstruction>>& IncrementalMapperController::Models() const {
    return models_;
}

const Reconstruction& IncrementalMapperController::Model(const size_t idx) const {
    return *models_.at(idx);
}

Reconstruction& IncrementalMapperController::Model(const size_t idx) {
    return *models_.at(idx);
}
