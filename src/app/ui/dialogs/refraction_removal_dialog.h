#pragma once

#include "ui/utils/cancellation_token.h"
#include "ui/widgets/camera_model_selector_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class RefractionRemovalDialog : public QDialog {
   public:
    RefractionRemovalDialog(QWidget* parent = nullptr);

   private:
    void ImportParameters();
    bool StartRemovingRefraction();

    QLineEdit* input_directory_edit_;
    QLineEdit* output_directory_edit_;
    QLineEdit* undistortion_map_edit_;

    std::unique_ptr<std::thread> runner_;
    std::atomic<bool> cancellation_flag_;
    CancellationToken cancel_;
  };
}
