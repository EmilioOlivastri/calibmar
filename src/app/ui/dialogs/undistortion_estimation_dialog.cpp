#include "undistortion_estimation_dialog.h"

#include <calibmar/core/report.h>
#include <colmap/util/misc.h>
#include <filesystem>

namespace calibmar {

  UndistortionEstimationDialog::UndistortionEstimationDialog(QWidget* parent) : QDialog(parent) {
    // directory groupbox
    QGroupBox* image_groupbox = new QGroupBox(this);
    image_groupbox->setTitle("Path to Image");
    image_edit_ = new QLineEdit(image_groupbox);
    QPushButton* select_image_button = new QPushButton(image_groupbox);
    select_image_button->setText("Browse");
    connect(select_image_button, &QPushButton::released, this, [this]() {
      
      QString image_path = QFileDialog::getOpenFileName(this, "Select image to be undistorted",
                                                       QString(), "Images (*.jpg *.png *.jpeg)");
      this->image_path_ = image_path.toStdString();
      this->image_edit_->setText(image_path);
    });
    QHBoxLayout* horizontal_layout_directory = new QHBoxLayout(image_groupbox);
    horizontal_layout_directory->addWidget(image_edit_);
    horizontal_layout_directory->addWidget(select_image_button);

    // common options
    undistortion_estimation_options_widget_ = new UndistortionEstimationOptionsWidget(this);

    // import button
    QHBoxLayout* horizontalLayout_run = new QHBoxLayout();
    QPushButton* import_button = new QPushButton(this);
    import_button->setText("Import...");
    connect(import_button, &QPushButton::released, this, [this]() { ImportParameters(); });
    horizontalLayout_run->addWidget(import_button, 0, Qt::AlignLeft | Qt::AlignTop);

    // run button
    QPushButton* run_button = new QPushButton(this);
    run_button->setText("Run");
    run_button->setDefault(true);
    connect(run_button, &QPushButton::released, this, [this]() {
      if (Validate()) {
        this->accept();
      }
    });

    horizontalLayout_run->addWidget(run_button, 0, Qt::AlignRight | Qt::AlignTop);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(image_groupbox);
    layout->addWidget(undistortion_estimation_options_widget_);
    layout->addLayout(horizontalLayout_run);
    setWindowTitle("Undistortion Estimation");

    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  void UndistortionEstimationDialog::SetOptions(Options options) {
    image_edit_->setText(QString::fromStdString(options.image_path));
    UndistortionEstimationOptionsWidget::Options undistortion_estimation_options;
    undistortion_estimation_options.camera_model = options.camera_model;
    undistortion_estimation_options.housing_options = options.housing_calibration;
    undistortion_estimation_options.camera_parameters = options.camera_parameters;
    undistortion_estimation_options.image_path = options.image_path;
    undistortion_estimation_options.projection_distance = options.projection_distance;
    undistortion_estimation_options.virtual_d0 = options.virtual_d0;
    undistortion_estimation_options.output_map = options.output_map;
    undistortion_estimation_options_widget_->SetOptions(undistortion_estimation_options);
  }

  UndistortionEstimationDialog::Options UndistortionEstimationDialog::GetOptions() {
    UndistortionEstimationOptionsWidget::Options undistortion_estimation_options = undistortion_estimation_options_widget_->GetOptions();
    Options options;
    options.camera_model = undistortion_estimation_options.camera_model;
    options.housing_calibration = undistortion_estimation_options.housing_options;
    options.camera_parameters = undistortion_estimation_options.camera_parameters;
    options.image_path = image_path_;
    options.projection_distance = undistortion_estimation_options.projection_distance;
    options.virtual_d0 = undistortion_estimation_options.virtual_d0;
    options.output_map = undistortion_estimation_options.output_map;
    return options;
  }

  bool UndistortionEstimationDialog::Validate() {
    if (!std::filesystem::is_regular_file(image_edit_->text().toStdString())) {
      QMessageBox::information(this, "Validation Error", "Image does not exist.");
      return false;
    }
    return undistortion_estimation_options_widget_->Validate();
  }

  void UndistortionEstimationDialog::ImportParameters() {
    std::string path =
        QFileDialog::getOpenFileName(this, "Import Parameters", QString(), "Calibration YAML (*.yaml *.yml)").toStdString();
    if (path.empty()) {
      return;
    }

    ImportedParameters p = ImportedParameters::ImportFromYaml(path);
    Options options;
    options.camera_model = p.camera_model;
    if (p.housing_model.has_value()) {
      options.housing_calibration = {p.housing_model.value(), p.housing_parameters};
    }
    options.camera_parameters = p.camera_parameters;
    SetOptions(options);
  }
}