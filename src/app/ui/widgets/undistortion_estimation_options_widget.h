#pragma once

#include "calibmar/core/report.h"
#include "ui/widgets/camera_model_selector_widget.h"
#include "ui/widgets/housing_selector_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class UndistortionEstimationOptionsWidget : public QWidget {
   public:
    struct Options {
      CameraModelType camera_model;
      std::optional<std::vector<double>> camera_parameters;
      std::optional<std::pair<HousingInterfaceType, std::vector<double>>> housing_options;
      std::string image_path;
      std::optional<double> projection_distance;
      std::optional<double> virtual_d0;
      std::string output_map;
    };

    UndistortionEstimationOptionsWidget(QWidget* parent = nullptr);

    bool Validate();

    Options GetOptions();
    void SetOptions(Options options);

   private:
    CameraModelSelectorWidget* camera_model_selector_;
    HousingSelectorWidget* housing_type_selector_;
    QLineEdit* projection_distance_edit_;
    QLineEdit* virtual_d0_edit_;
    QLineEdit* output_map_edit_;

    std::string image_path_;
    std::optional<double> projection_distance_;
    std::optional<double> virtual_d0_;
    std::string output_map_;

    std::optional<std::pair<HousingInterfaceType, std::vector<double>>> housing_calibration_;
    std::optional<std::vector<double>> camera_parameters_;
  };
}