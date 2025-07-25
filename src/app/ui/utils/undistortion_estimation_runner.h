#pragma once

#include "calibmar/core/calibration.h"
#include "ui/dialogs/undistortion_estimation_dialog.h"
#include "ui/widgets/undistortion_estimation_widget.h"

namespace calibmar {

  class UndistortionEstimationRunner {
   public:
    UndistortionEstimationRunner(UndistortionEstimationWidget* undistortion_widget, UndistortionEstimationDialog::Options options);

    bool Run();

   private:
    UndistortionEstimationWidget* undistortion_widget_;
    UndistortionEstimationDialog::Options options_;
  };
}
