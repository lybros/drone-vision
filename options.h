#ifndef INC_3D_RECONSTRUCTION_OPTIONS_H
#define INC_3D_RECONSTRUCTION_OPTIONS_H

#include "features.h"
#include "processor.h"

#include <string>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

struct BaseOptions {
  virtual void Reset() = 0;
  virtual bool Check() = 0;
};

struct ExtractionOptions : public BaseOptions {
  ExtractionOptions();

  void Reset() override;
  bool Check() override;

  FeatureExtractor::Options Options() const;

  std::string camera_model;
  bool single_camera;
  std::string camera_params;
  double default_focal_length_factor;

  SIFTOptions sift_options;
};

struct MatchOptions : public BaseOptions {
  MatchOptions();

  void Reset() override;
  bool Check() override;

  FeatureMatcher::Options Options() const;

  int num_threads;
  int gpu_index;
  double max_ratio;
  double max_distance;
  bool cross_check;
  int max_num_matches;
  double max_error;
  double confidence;
  int max_num_trials;
  double min_inlier_ratio;
  int min_num_inliers;
  bool multiple_models;
  bool guided_matching;
};

struct ExhaustiveMatchOptions : public BaseOptions {
  ExhaustiveMatchOptions();

  void Reset() override;
  bool Check() override;

  ExhaustiveFeatureMatcher::ExhaustiveOptions Options() const;

  int block_size;
  bool preemptive;
  int preemptive_num_features;
  int preemptive_min_num_matches;
};

struct BundleAdjustmentOptions : public BaseOptions {
  BundleAdjustmentOptions();

  void Reset() override;
  bool Check() override;

  virtual BundleAdjuster::Options Options() const;

  int max_num_iterations;
  int max_linear_solver_iterations;

  double function_tolerance;
  double gradient_tolerance;
  double parameter_tolerance;

  int max_num_consecutive_invalid_steps;
  int max_consecutive_nonmonotonic_steps;
  bool use_nonmonotonic_steps;

  bool minimizer_progress_to_stdout;

  int min_observations_per_image;

  double loss_function_scale;

  bool refine_focal_length;
  bool refine_principal_point;
  bool refine_extra_params;

  bool print_summary;
};

struct TriangulationOptions : public BaseOptions {
  TriangulationOptions();

  void Reset() override;
  bool Check() override;

  IncrementalTriangulator::Options Options() const;

  int max_transitivity;
  double create_max_angle_error;
  double continue_max_angle_error;
  double merge_max_reproj_error;
  double re_max_angle_error;
  double re_min_ratio;
  int re_max_trials;
  double complete_max_reproj_error;
  int complete_max_transitivity;
  double min_angle;
  bool ignore_two_view_tracks;
};

struct IncrementalMapperOptions : public BaseOptions {
  IncrementalMapperOptions();

  void Reset() override;
  bool Check() override;

  IncrementalMapper::Options Options() const;

  int init_min_num_inliers;
  double init_max_error;
  double init_max_forward_motion;
  double init_min_tri_angle;

  double abs_pose_max_error;
  int abs_pose_min_num_inliers;
  double abs_pose_min_inlier_ratio;

  double filter_max_reproj_error;
  double filter_min_tri_angle;

  int max_reg_trials;
};

struct MapperOptions : public BaseOptions {
  MapperOptions();

  void Reset() override;
  bool Check() override;

  IncrementalMapper::Options IncrementalMapperOptions() const;
  IncrementalTriangulator::Options TriangulationOptions() const;
  BundleAdjuster::Options LocalBundleAdjustmentOptions() const;
  BundleAdjuster::Options GlobalBundleAdjustmentOptions() const;

  int min_num_matches;
  bool ignore_watermarks;

  bool multiple_models;
  int max_num_models;
  int max_model_overlap;
  int min_model_size;

  int init_image_id1;
  int init_image_id2;
  int init_num_trials;

  bool extract_colors;

  int num_threads;

  double min_focal_length_ratio;
  double max_focal_length_ratio;
  double max_extra_param;

  bool ba_refine_focal_length;
  bool ba_refine_principal_point;
  bool ba_refine_extra_params;

  int ba_local_num_images;
  int ba_local_max_num_iterations;

  double ba_global_images_ratio;
  double ba_global_points_ratio;
  int ba_global_images_freq;
  int ba_global_points_freq;
  int ba_global_max_num_iterations;

  int ba_global_max_refinements;
  double ba_global_max_refinement_change;
  int ba_local_max_refinements;
  double ba_local_max_refinement_change;

  struct IncrementalMapperOptions incremental_mapper;
  struct TriangulationOptions triangulation;
};

struct RenderOptions : public BaseOptions {
  RenderOptions();

  void Reset() override;
  bool Check() override;

  int min_track_len;
  double max_error;
  int refresh_rate;
  bool adapt_refresh_rate;
  bool image_connections;
};

class OptionManager {
 public:
  OptionManager();

  void AddAllOptions();
  void AddDatabaseOptions();
  void AddImageOptions();
  void AddExtractionOptions();
  void AddMatchOptions();
  void AddExhaustiveMatchOptions();
  void AddBundleAdjustmentOptions();
  void AddMapperOptions();
  void AddRenderOptions();

  void Reset();

  bool Check();

  std::shared_ptr<boost::program_options::options_description> desc;

  std::shared_ptr<std::string> project_path;
  std::shared_ptr<std::string> database_path;
  std::shared_ptr<std::string> image_path;

  std::shared_ptr<ExtractionOptions> extraction_options;
  std::shared_ptr<MatchOptions> match_options;
  std::shared_ptr<ExhaustiveMatchOptions> exhaustive_match_options;
  std::shared_ptr<BundleAdjustmentOptions> ba_options;
  std::shared_ptr<MapperOptions> mapper_options;
  std::shared_ptr<RenderOptions> render_options;

 private:
  template <typename T>
  void RegisterOption(const std::string& name, const T* option);

  std::vector<std::pair<std::string, const bool*>> options_bool_;
  std::vector<std::pair<std::string, const int*>> options_int_;
  std::vector<std::pair<std::string, const double*>> options_double_;
  std::vector<std::pair<std::string, const std::string*>> options_string_;

  bool added_database_options_;
  bool added_image_options_;
  bool added_extraction_options_;
  bool added_match_options_;
  bool added_exhaustive_match_options_;
  bool added_ba_options_;
  bool added_mapper_options_;
  bool added_render_options_;
};

template <typename T>
void OptionManager::RegisterOption(const std::string& name, const T* option) {
  if (std::is_same<T, bool>::value) {
    options_bool_.emplace_back(name, reinterpret_cast<const bool*>(option));
  } else if (std::is_same<T, int>::value) {
    options_int_.emplace_back(name, reinterpret_cast<const int*>(option));
  } else if (std::is_same<T, double>::value) {
    options_double_.emplace_back(name, reinterpret_cast<const double*>(option));
  } else if (std::is_same<T, std::string>::value) {
    options_string_.emplace_back(name,
                                 reinterpret_cast<const std::string*>(option));
  }
}


#endif //INC_3D_RECONSTRUCTION_OPTIONS_H
