#pragma once

#include "ui/widgets/undistortion_estimation_options_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

    
  class UndistortionEstimationDialog : public QDialog {
   public:
    struct Options {
      CameraModelType camera_model;
      std::optional<std::pair<HousingInterfaceType, std::vector<double>>> housing_calibration;
      std::optional<std::vector<double>> camera_parameters;
      std::string image_path;
      std::optional<double> projection_distance;
      std::optional<double> virtual_d0;
      std::string output_map;
    };

    UndistortionEstimationDialog(QWidget* parent = nullptr);

    void SetOptions(Options options);
    Options GetOptions();

   private:
    bool Validate();
    void ImportParameters();

    QLineEdit* image_edit_;
    std::string image_path_;
    UndistortionEstimationOptionsWidget* undistortion_estimation_options_widget_;
  };
}
