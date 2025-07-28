#include "refraction_removal_dialog.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/filesystem.hpp>

#include <colmap/util/misc.h>
#include <glob.h>

#include <filesystem>
#include <thread>


namespace calibmar {

  RefractionRemovalDialog::RefractionRemovalDialog(QWidget* parent)
      : QDialog(parent), cancellation_flag_(false), cancel_(cancellation_flag_) {
    
    // input directory groupbox
    QGroupBox* input_directory = new QGroupBox(this);
    input_directory->setTitle("Input images directory (affected by refraction)");
    input_directory_edit_ = new QLineEdit(input_directory);
    QPushButton* input_directory_button = new QPushButton(input_directory);
    input_directory_button->setText("Browse");
    connect(input_directory_button, &QPushButton::released, this, [this]() {
      this->input_directory_edit_->setText(QFileDialog::getExistingDirectory(this, "Select input images directory"));
    });
    QHBoxLayout* horizontal_layout_input = new QHBoxLayout(input_directory);
    horizontal_layout_input->addWidget(input_directory_edit_);
    horizontal_layout_input->addWidget(input_directory_button);
    
    // output directory groupbox
    QGroupBox* output_directory = new QGroupBox(this);
    output_directory->setTitle("Output images directory (undistorted images)");
    output_directory_edit_ = new QLineEdit(output_directory);
    QPushButton* output_directory_button = new QPushButton(output_directory);
    output_directory_button->setText("Browse");
    connect(output_directory_button, &QPushButton::released, this, [this]() {
      this->output_directory_edit_->setText(QFileDialog::getExistingDirectory(this, "Select output images directory"));
    });
    QHBoxLayout* horizontal_layout_output = new QHBoxLayout(output_directory);
    horizontal_layout_output->addWidget(output_directory_edit_);
    horizontal_layout_output->addWidget(output_directory_button);

    // Undistortion map groupbox
    QGroupBox* undistortion_map_gbox = new QGroupBox(this);
    undistortion_map_gbox->setTitle("Undistortion map");
    undistortion_map_edit_ = new QLineEdit(undistortion_map_gbox);
    QPushButton* undistortion_map_button = new QPushButton(undistortion_map_gbox);
    undistortion_map_button->setText("Browse");
    connect(undistortion_map_button, &QPushButton::released, this, [this]() {
      this->undistortion_map_edit_->setText(QFileDialog::getOpenFileName(
          this, "Select undistortion map", QString(), "Undistortion Map (*.yaml *.yml)"));
    });
    QHBoxLayout* horizontal_layout_undistortion_map = new QHBoxLayout(undistortion_map_gbox);
    horizontal_layout_undistortion_map->addWidget(undistortion_map_edit_);
    horizontal_layout_undistortion_map->addWidget(undistortion_map_button);
    
    // run button
    QHBoxLayout* horizontalLayout_run = new QHBoxLayout();
    QPushButton* run_button = new QPushButton(this);
    run_button->setText("Start");
    run_button->setDefault(true);
    connect(run_button, &QPushButton::released, this, [this]() {
      setEnabled(false);
      if (!StartRemovingRefraction()) {
        // If undistorting didnt start (some input error) reenable dialog
        setEnabled(true);
      }
    });

    horizontalLayout_run->addWidget(run_button, 0, Qt::AlignRight | Qt::AlignTop);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(input_directory);
    layout->addWidget(output_directory);
    layout->addWidget(undistortion_map_gbox);
    layout->addLayout(horizontalLayout_run);
    setWindowTitle("Undistort Images");

    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  bool RefractionRemovalDialog::StartRemovingRefraction() {
    // validate inputs
    std::filesystem::path input_dir(input_directory_edit_->text().toStdString());
    std::filesystem::path output_dir(output_directory_edit_->text().toStdString());
    if (!std::filesystem::is_directory(input_dir)) {
      QMessageBox::critical(this, "", "Input directory does not exist!");
      return false;
    }
    if (!std::filesystem::is_directory(output_dir)) {
      QMessageBox::critical(this, "", "Ouput directory does not exist!");
      return false;
    }
    std::string msg(undistortion_map_edit_->text().toStdString());
    std::string extension = msg.empty() ? "" : msg.substr(msg.find_last_of('.') + 1);
    if (msg.empty() || !std::filesystem::exists(msg) || (extension != "yaml" && extension != "yml")) {
      QMessageBox::critical(this, "", "Please select a valid undistortion map file!");
      return false;
    }

    if (!std::filesystem::is_empty(output_dir)) {
      if (QMessageBox::warning(
              this, "",
              "Output directory is not empty. Potentially duplicate files will be overwritten (input filenames are "
              "suffixed with '_u')",
              QMessageBox::Ok, QMessageBox::Cancel) != QMessageBox::Ok) {
        return false;
      }
    }

    std::vector<std::string> img_names;
    std::string pattern = input_dir.string() + "/*.[jp]*g";
    std::string output_folder = output_dir.string();

    // UNIX GLOB
    glob_t glob_result;
    glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
      img_names.push_back(glob_result.gl_pathv[i]);
    }
    globfree(&glob_result);


    if (img_names.empty()) {
      QMessageBox::critical(this, "", "No images found in input directory!");
      return false;
    }

    cv::FileStorage fs(undistortion_map_edit_->text().toStdString(), cv::FileStorage::READ);    
    cv::Mat roi, undist_map;
    fs["ROI"] >> roi;
    fs["Undistortion Map"] >> undist_map;

    cv::Range cr(roi.at<int>(0), roi.at<int>(2));
    cv::Range rr(roi.at<int>(1), roi.at<int>(3));

    // Show busy dialog with cancel option
    QDialog* progress_dialog = new QDialog(this);
    progress_dialog->setWindowTitle("Undistorting...");
    QVBoxLayout* layout = new QVBoxLayout(progress_dialog);
    QProgressBar* bar = new QProgressBar(progress_dialog);
    bar->setTextVisible(false);
    bar->setMaximum(100);
    bar->setMinimum(0);
    bar->show();
    bar->setOrientation(Qt::Orientation::Horizontal);
    QPushButton* button = new QPushButton("Cancel", progress_dialog);
    connect(button, &QPushButton::pressed, [flag = &cancellation_flag_, button]() {
      flag->store(true);
      button->setDisabled(true);
    });
    layout->addWidget(bar);
    layout->addWidget(button);
    progress_dialog->show();
    progress_dialog->setFixedSize(progress_dialog->size());

    double progress = 0.0;
    double step = 100.0 / img_names.size();
    bar->setValue(static_cast<int>(progress));

    #pragma omp parallel for
    for (int i = 0; i < img_names.size() ; ++i) 
    {
        if (cancellation_flag_) continue;

        std::string img_path = img_names[i];
        std::string img_name = img_path.substr(img_path.find_last_of("/\\") + 1);
        cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
        cv::Mat dst_img = cv::Mat::zeros(img.rows, img.cols, img.type());
        cv::remap(img, dst_img, undist_map, cv::noArray(), cv::INTER_LINEAR, cv::BorderTypes::BORDER_CONSTANT);
        cv::Mat roi_img = dst_img(rr, cr).clone();

        cv::imwrite(output_folder + "/" + img_name, roi_img);
        progress += step;
        int progress_int = static_cast<int>(progress);
        QMetaObject::invokeMethod(progress_dialog, [bar, progress_int]() {
          bar->setValue(progress_int);
        });
    }

    if (cancellation_flag_) {
      QMessageBox::warning(this, "Cancelled", "Refraction removal process was cancelled by the user.");
      QMetaObject::invokeMethod(progress_dialog, [progress_dialog]() {
            progress_dialog->close();
            progress_dialog->deleteLater();
          });
      return false;
    }


    bar->setValue(100);

    // Close the progress dialog
    QMetaObject::invokeMethod(progress_dialog, [progress_dialog]() {
      progress_dialog->close();
      progress_dialog->deleteLater();
    });

    // Terminate the dialog
    QMetaObject::invokeMethod(this, [this]() {
      this->setEnabled(true);
      this->close();
    });

    QMessageBox::information(this, "Success", "Refraction removal completed successfully!");

    return true;
  }
}