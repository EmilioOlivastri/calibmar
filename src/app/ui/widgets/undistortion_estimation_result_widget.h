#pragma once

#include "colmap/scene/camera.h"
#include "calibmar/core/pixmap.h"

#include <colmap/controllers/option_manager.h>

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  // Widget that holds the undistoriton results (e.g. report or error message)
  class UndistortionEstimationResultWidget : public QWidget {
   public:
    UndistortionEstimationResultWidget(colmap::Camera& camera,
                                       std::unique_ptr<Pixmap> original_image,
                                       std::unique_ptr<Pixmap> undistorted_image,
                                       std::pair<double, double> distances, 
                                       QWidget* parent = nullptr);

    UndistortionEstimationResultWidget(const std::string& message, QWidget* parent = nullptr);

   protected:
    virtual void showEvent(QShowEvent* e) override;

   private:
    std::unique_ptr<Pixmap> undistorted_image_;
    std::unique_ptr<Pixmap> original_image_;
    std::unique_ptr<colmap::OptionManager> options_manager_;
    QTextEdit* result_text_;
  };
}