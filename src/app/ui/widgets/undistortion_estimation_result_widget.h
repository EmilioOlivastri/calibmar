#pragma once

#include "calibmar/core/camera.h"
#include "calibmar/core/pixmap.h"

#include <colmap/controllers/option_manager.h>

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  // Widget that holds the undistoriton results (e.g. report or error message)
  class UndistortionEstimationResultWidget : public QWidget {
   public:
    UndistortionEstimationResultWidget(colmap::Camera& camera, 
                                       std::unique_ptr<Pixmap> offset_visu_pixmap = std::unique_ptr<Pixmap>(), 
                                       QWidget* parent = nullptr);

    UndistortionEstimationResultWidget(const std::string& message, QWidget* parent = nullptr);

   protected:
    virtual void showEvent(QShowEvent* e) override;

   private:
    std::unique_ptr<Pixmap> offset_visu_pixmap_;
    std::unique_ptr<colmap::OptionManager> options_manager_;
    QTextEdit* result_text_;
  };
}