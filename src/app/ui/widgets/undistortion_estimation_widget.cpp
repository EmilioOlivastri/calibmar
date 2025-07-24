#include "ui/widgets/undistortion_estimation_widget.h"

namespace calibmar {
  UndistortionEstimationWidget::UndistortionEstimationWidget(
      QWidget* parent, const std::function<void(const std::string&, const class TargetVisualizer&)> double_click_callback)
      : QWidget(parent), undistortion_estimation_ended_(false) {
    main_layout_ = new QVBoxLayout(this);
    extraction_images_ = new ExtractionImagesWidget(this, double_click_callback);

    QGroupBox* features_groupbox = new QGroupBox("Input Image");
    QVBoxLayout* features_groupbox_layout = new QVBoxLayout(features_groupbox);
    features_groupbox_layout->addWidget(extraction_images_);
    features_groupbox_layout->setContentsMargins(0, 0, 0, 0);
    main_layout_->addWidget(features_groupbox);

    undistortion_estimation_widget_ = new QGroupBox("Undistorted Image");
    QVBoxLayout* undistortion_estimation_layout = new QVBoxLayout(undistortion_estimation_widget_);
    undistortion_estimation_widget_->setVisible(false);
    main_layout_->addWidget(undistortion_estimation_widget_);
  }

  void UndistortionEstimationWidget::AddExtractionItem(QWidget* widget) {
    // if EndCalibration is called this might be null (can happen with unfortunate ordering in deferred qt calls)
    if (!undistortion_estimation_ended_) {
      extraction_images_->AddImage(widget);
    }
  }

  void UndistortionEstimationWidget::StartUndistortionEstimation() {
    undistortion_estimation_widget_->setVisible(true);
    progress_bar_ = new QProgressBar();
    progress_bar_->setTextVisible(false);
    progress_bar_->setMaximum(100);
    progress_bar_->setMinimum(0);
    progress_bar_->setValue(0);
    progress_bar_->show();
    undistortion_estimation_widget_->layout()->addWidget(progress_bar_);
  }

  void UndistortionEstimationWidget::SetProgress(int value) {
    if (progress_bar_) {
      progress_bar_->setValue(value);
    }
  }

  void UndistortionEstimationWidget::EndUndistortionEstimation(QWidget* undistortion_result) {
    undistortion_estimation_ended_ = true;

    if (undistortion_estimation_widget_) {
      delete undistortion_estimation_widget_;
    }

    /* ADD THE RESULTING IMAGE IN THE RESULT WIDGET */
    undistortion_estimation_widget_ = new QGroupBox("Undistorted Image");
    QVBoxLayout* undistortion_estimation_layout = new QVBoxLayout(undistortion_estimation_widget_);
    undistortion_estimation_layout->addWidget(undistortion_result);
    main_layout_->addWidget(undistortion_estimation_widget_);
    /* FIX UNCOMPLETE SECTION */
  }

  void UndistortionEstimationWidget::SetTargetVisualizer(std::unique_ptr<class TargetVisualizer> target_visualizer) {
    target_visualizer_ = std::move(target_visualizer);
  }

  const class TargetVisualizer& UndistortionEstimationWidget::TargetVisualizer() {
    return *target_visualizer_;
  }
}
