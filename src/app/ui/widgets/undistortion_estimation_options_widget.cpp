#include "undistortion_estimation_options_widget.h"
#include "ui/utils/parse_params.h"
#include <colmap/util/misc.h>

namespace calibmar {
  UndistortionEstimationOptionsWidget::UndistortionEstimationOptionsWidget(QWidget* parent) : QWidget(parent) {
    // camera model
    camera_model_selector_ = new CameraModelSelectorWidget(this);

    // housing type
    housing_type_selector_ = new HousingSelectorWidget(this);

    // undistortion options
    QGroupBox* undistortion_groupbox = new QGroupBox("Undistortion Options", this);
    QVBoxLayout* layout_groupbox = new QVBoxLayout(undistortion_groupbox);
    projection_distance_edit_ = new QLineEdit(undistortion_groupbox);
    projection_distance_edit_->setPlaceholderText("Projection Distance");

    virtual_d0_edit_ = new QLineEdit(undistortion_groupbox);
    virtual_d0_edit_->setPlaceholderText("Virtual d0");
    layout_groupbox->addWidget(projection_distance_edit_);
    layout_groupbox->addWidget(virtual_d0_edit_);

    QGroupBox* output_groupbox = new QGroupBox("Output Map Path", this);
    output_map_edit_ = new QLineEdit(output_groupbox);
    QPushButton* select_directory_button = new QPushButton(output_groupbox);
    select_directory_button->setText("Browse");
    connect(select_directory_button, &QPushButton::released, this, [this]() {
      QString output_path = QFileDialog::getExistingDirectory(this, "Select output directory for undistortion map");
      this->output_map_ = output_path.toStdString();
      this->output_map_edit_->setText(output_path);
    });
    QHBoxLayout* horizontal_layout_directory = new QHBoxLayout(output_groupbox);
    horizontal_layout_directory->addWidget(output_map_edit_);
    horizontal_layout_directory->addWidget(select_directory_button);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(camera_model_selector_);
    layout->addWidget(housing_type_selector_);
    layout->addWidget(undistortion_groupbox);
    layout->addWidget(output_groupbox);
    layout->setSizeConstraint(QLayout::SetMinimumSize);
    layout->setContentsMargins(0, 0, 0, 0);
  }

  bool UndistortionEstimationOptionsWidget::Validate() {
    std::optional<std::pair<HousingInterfaceType, std::string>> housing = housing_type_selector_->HousingOptions();
    if (housing.has_value()) {
      std::vector<double> params;
      if (!TryParseParams(params, housing.value().second)) {
        QMessageBox::information(this, "Validation Error", "Invalid housing parameter format.");
        return false;
      }
      else if (params.size() != HousingInterface::HousingInterfaces().at(housing.value().first).num_params) {
        QMessageBox::information(this, "Validation Error", "Housing parameters dont match housing type.");
        return false;
      }
      else if (housing->first == HousingInterfaceType::DoubleLayerPlanarRefractive &&
               abs(Eigen::Map<Eigen::Vector3d>(params.data()).norm() - 1) > 1e-6) {
        QMessageBox::information(this, "Validation Error", "Interface normal must be normalized to unit length!");
        return false;
      }
      else {
        // validated, take parameters
        housing_calibration_ = {housing.value().first, params};
      }
    }
    else {
      QMessageBox::information(this, "Validation Error", "Housing model required for undistortion estimation!");
      return false;
    }

    std::string message;
    if (!camera_model_selector_->Validate(message)) {
      QMessageBox::information(this, "Validation Error", QString::fromStdString(message));
      return false;
    }
    else {
      // validated
      camera_parameters_ = camera_model_selector_->InitialCameraParameters();
    }

    if (!camera_parameters_.has_value()) {
      QMessageBox::information(this, "Validation Error", "Undistortion estimation requires the camera intrinsic parameters.");
      return false;
    }

    if (output_map_.empty()) {
      QMessageBox::information(this, "Validation Error", "Output map path is required.");
      return false;
    }

    std::string prj_dist_str = projection_distance_edit_->text().toStdString();
    if (prj_dist_str.empty()) {
      QMessageBox::information(this, "Validation Error", "Projection distance is required.");
      return false;
    }

    bool is_double = true;
    try { projection_distance_ = std::stod(prj_dist_str); }
    catch (const std::invalid_argument&) { is_double = false; }

    if (!is_double) {
      QMessageBox::information(this, "Validation Error", "Projection distance must be a valid number.");
      return false;
    }
    
    std::string virtual_d0_str = virtual_d0_edit_->text().toStdString();
    if (housing->first == HousingInterfaceType::DoubleLayerPlanarRefractive && virtual_d0_str.empty()) {
      QMessageBox::information(this, "Validation Error", "Virtual d0 is required when using Flat Port.");
      return false;
    }

    try { virtual_d0_ = std::stod(virtual_d0_str); }
    catch (const std::invalid_argument&) { is_double = false; }

    if (!is_double) {
      QMessageBox::information(this, "Validation Error", "Virtual d0 must be a valid number.");
      return false;
    }

    return true;
  }

  UndistortionEstimationOptionsWidget::Options UndistortionEstimationOptionsWidget::GetOptions() {
    Options options;
    options.camera_model = camera_model_selector_->CameraModel();
    options.housing_options = housing_calibration_;
    options.camera_parameters = camera_parameters_;
    options.image_path = image_path_;
    options.projection_distance = projection_distance_;
    options.virtual_d0 = virtual_d0_;
    options.output_map = output_map_;
    return options;
  }

  void UndistortionEstimationOptionsWidget::SetOptions(Options options) {
    camera_model_selector_->SetCameraModel(options.camera_model);
    camera_model_selector_->SetInitialCameraParameters(options.camera_parameters);
    image_path_ = options.image_path;
    if (options.housing_options.has_value()) {
      housing_type_selector_->SetHousingOptions(
          std::make_pair(options.housing_options->first, colmap::VectorToCSV(options.housing_options->second)));
    }
    else {
      housing_type_selector_->SetHousingOptions({});
    }
    projection_distance_ = options.projection_distance;
    virtual_d0_ = options.virtual_d0;
    output_map_ = options.output_map;

  }

  
}