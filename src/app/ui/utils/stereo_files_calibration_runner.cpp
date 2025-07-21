#include "stereo_files_calibration_runner.h"
#include "calibmar/calibrators/calibrator3D.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/calibrators/stereo_calibrator.h"
#include "calibmar/extractors/aruco_sift_extractor.h"
#include "calibmar/extractors/sift_extractor.h"
#include "calibmar/readers/filesystem_reader.h"

#include "ui/widgets/calibration_result_widget.h"

#include <colmap/scene/reconstruction.h>

namespace {

  void SetupPlanarCalibration(
    std::variant<calibmar::ChessboardFeatureExtractor::Options, calibmar::ArucoBoardFeatureExtractor::Options>& target_options,
    const calibmar::StereoFileCalibrationDialog::Options& options, calibmar::Calibration& calibration, std::pair<int, int> image_size,
    std::unique_ptr<calibmar::FeatureExtractor>& extractor, 
    std::unique_ptr<calibmar::TargetVisualizer>& target_visualizer) {
    using namespace calibmar;

    ChessboardFeatureExtractor::Options* chessboard_options =
        std::get_if<calibmar::ChessboardFeatureExtractor::Options>(&target_options);

    if (chessboard_options) {
      chessboard_options->fast = false;
      std::unique_ptr<ChessboardFeatureExtractor> chessboard_extractor =
          std::make_unique<ChessboardFeatureExtractor>(*chessboard_options);
      calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(*chessboard_options));
      calibration.SetPoints3D(chessboard_extractor->Points3D());
      extractor = std::move(chessboard_extractor);
      target_visualizer = std::make_unique<ChessboardTargetVisualizer>(chessboard_options->chessboard_columns,
                                                                      chessboard_options->chessboard_rows);
    }
    else {
      ArucoBoardFeatureExtractor::Options& aruco_options = std::get<ArucoBoardFeatureExtractor::Options>(target_options);
      std::unique_ptr<ArucoBoardFeatureExtractor> aurco_extractor = std::make_unique<ArucoBoardFeatureExtractor>(aruco_options);
      calibration.SetPoints3D(aurco_extractor->Points3D());
      calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(aruco_options));
      extractor = std::move(aurco_extractor);
      target_visualizer = std::make_unique<ArucoBoardTargetVisualizer>();
    }


  }


  std::unique_ptr<calibmar::ExtractionImageWidget::Data> ReadAndExtractImage(calibmar::ImageReader& reader,
                                                                             calibmar::FeatureExtractor& extractor,
                                                                             calibmar::Image& image) {
    using namespace calibmar;

    std::unique_ptr<Pixmap> pixmap = std::make_unique<Pixmap>();
    ImageReader::Status reader_status = reader.Next(image, *pixmap);
    FeatureExtractor::Status extractor_status;
    std::unique_ptr<ExtractionImageWidget::Data> data = std::make_unique<ExtractionImageWidget::Data>();
    if (reader_status == ImageReader::Status::SUCCESS) {
      data->image_name = image.Name();

      extractor_status = extractor.Extract(image, *pixmap);

      data->image = std::move(pixmap);
      data->status = ExtractionImageWidget::ConvertStatus(extractor_status);
    }
    else {
      data->status = ExtractionImageWidget::ConvertStatus(reader_status);
    }

    return std::move(data);
  }
}

namespace calibmar {

  StereoFilesCalibrationRunner::StereoFilesCalibrationRunner(CalibrationWidget* calibration_widget,
                                                             StereoFileCalibrationDialog::Options options)
      : calibration_widget_(calibration_widget), options_(options) {}

  bool StereoFilesCalibrationRunner::Run(Calibration& calibration1, Calibration& calibration2) {
    FilesystemImageReader::Options reader_options1;
    reader_options1.image_directory = options_.images_directory1;
    FilesystemImageReader::Options reader_options2;
    reader_options2.image_directory = options_.images_directory2;
    FilesystemImageReader reader1(reader_options1);
    FilesystemImageReader reader2(reader_options2);
    std::pair<int, int> image_size{reader1.ImagesWidth(), reader1.ImagesHeight()};


    /* EMILIO'S CODE*/
    std::unique_ptr<FeatureExtractor> extractor;
    std::unique_ptr<TargetVisualizer> target_visualizer;
    std::variant<calibmar::ChessboardFeatureExtractor::Options, calibmar::ArucoBoardFeatureExtractor::Options> target_options;
    if (std::holds_alternative<ChessboardFeatureExtractor::Options>(options_.calibration_target_options)) {
      target_options = std::get<ChessboardFeatureExtractor::Options>(options_.calibration_target_options);
    }
    else {
      target_options = std::get<ArucoBoardFeatureExtractor::Options>(options_.calibration_target_options);
    }
    SetupPlanarCalibration(target_options, options_, calibration1, image_size, extractor, target_visualizer);
    SetupPlanarCalibration(target_options, options_, calibration2, image_size, extractor, target_visualizer);
    /* EMILIO'S CODE*/


    /* ORIGINAL CODE *
    ChessboardFeatureExtractor::Options extractor_options;
    extractor_options.chessboard_columns = options_.calibration_target_options.chessboard_columns;
    extractor_options.chessboard_rows = options_.calibration_target_options.chessboard_rows;
    extractor_options.square_size = options_.calibration_target_options.square_size;
    ChessboardFeatureExtractor extractor(extractor_options);
    calibration1.SetPoints3D(extractor.Points3D());
    calibration1.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(options_.calibration_target_options));
    calibration2.SetPoints3D(extractor.Points3D());
    calibration2.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(options_.calibration_target_options));
    /*-------------*/

    StereoCalibrator::Options calibrator_options;
    calibrator_options.estimate_pose_only = options_.estimate_pose_only;
    colmap::Camera camera1;
    colmap::Camera camera2;
    if (options_.initial_camera_parameters.has_value()) {
      camera1 = CameraModel::InitCamera(options_.camera_model, image_size, options_.initial_camera_parameters->first);
      camera2 = CameraModel::InitCamera(options_.camera_model, image_size, options_.initial_camera_parameters->second);
      calibrator_options.use_intrinsics_guess = true;
    }
    else {
      calibrator_options.camera_model = options_.camera_model;
      calibrator_options.image_size = image_size;
    }

    calibration1.SetCamera(camera1);
    calibration2.SetCamera(camera2);

    StereoCalibrator calibrator(calibrator_options);
    /* ORIGINAL CODE *
    std::unique_ptr<TargetVisualizer> target_visualizer = std::make_unique<ChessboardTargetVisualizer>(
        options_.calibration_target_options.chessboard_columns, options_.calibration_target_options.chessboard_rows);
    /**/

    calibration_widget_->SetTargetVisualizer(std::move(target_visualizer));

    try {
      while (reader1.HasNext() && reader2.HasNext()) {
        Image image1, image2;
        std::unique_ptr<calibmar::ExtractionImageWidget::Data> data1 = ReadAndExtractImage(reader1, *extractor, image1);
        std::unique_ptr<calibmar::ExtractionImageWidget::Data> data2 = ReadAndExtractImage(reader2, *extractor, image2);

        if (data1->status == ExtractionImageWidget::Status::SUCCESS && data2->status == ExtractionImageWidget::Status::SUCCESS) {
          
          /* EMILIO'S ADAPTATION CODE FOR ARUCO BOARDS*/
          bool correct_corr = true;
          if ( std::holds_alternative<ArucoBoardFeatureExtractor::Options>(options_.calibration_target_options)) {
            
            std::unordered_map<size_t, uint32_t> corr1 = image1.Correspondences();
            std::unordered_map<size_t, uint32_t> corr2 = image2.Correspondences();

            for (auto it_x = corr1.begin(); it_x != corr1.end() && correct_corr; ++it_x) {
              size_t point2D_idx = it_x->first;
              uint32_t point3D_idx = it_x->second;

              if (point2D_idx != point3D_idx)
              {
                std::cout << "Tuple: [" << point2D_idx << " | " << point3D_idx << "]" << std::endl;
                correct_corr = false;
                continue;
              }

              const auto& it_y = corr2.find(point2D_idx);
              if (it_y == corr2.end()) 
              {
                correct_corr = false;
                continue;
              }

              size_t point2D_idy = it_y->first;
              correct_corr &= point3D_idx == corr2[point2D_idy];
              correct_corr &= point2D_idy == point2D_idx;
              
              if (!correct_corr)
              {
                std::cout << "Image 1: " << image1.Name() << " Image 2: " << image2.Name() << std::endl;
                std::cout << "Details of correspondences: " << std::endl;
                std::cout << "point2D_idx: " << point2D_idx << std::endl;
                std::cout << "point3D_idx: " << point3D_idx << std::endl;
                std::cout << "point2D_idy: " << point2D_idy << std::endl;
                std::cout << "point3D_idy: " << corr2[point2D_idy] << std::endl;
                std::cout << "Coordinates of point2D_idx: " << image1.Point2D(point2D_idx).transpose() << std::endl;
                std::cout << "Coordinates of point3D_idx: " << calibration1.Point3D(point3D_idx).transpose() << std::endl;
                std::cout << "Coordinates of point2D_idx_2: " << image2.Point2D(point2D_idy).transpose() << std::endl;
                std::cout << "Coordinates of point3D_idx_2: " << calibration2.Point3D(corr2[point2D_idy]).transpose() << std::endl;
                std::cout << "-------------------------" << std::endl;
              }
            }
          }
          /* EMILIO'S ADAPTATION CODE FOR ARUCO BOARDS*/


          if (!correct_corr)
          {
            std::cout << "Image 1: " << image1.Name() << " Image 2: " << image2.Name() << std::endl;
            std::cout << "Correspondences are not correct. Skipping images." << std::endl;
            continue;
          }

          size_t id = calibration1.AddImage(image1);
          data1->image_data = calibration1.Image(id);
          id = calibration2.AddImage(image2);
          data2->image_data = calibration2.Image(id);
        }

        // currently ignore read errors for visu. Check visualization, when reenabling.
        if (data1->status == ExtractionImageWidget::Status::READ_ERROR ||
            data2->status == ExtractionImageWidget::Status::READ_ERROR) {
          continue;
        }

        QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, data1 = std::move(data1),
                                                        data2 = std::move(data2)]() mutable {
          QFrame* frame = new QFrame();
          frame->setFrameStyle(QFrame::StyledPanel);
          QGridLayout* grid = new QGridLayout(frame);
          ExtractionImageWidget* image_widget1 =
              new ExtractionImageWidget(std::move(data1), calibration_widget->TargetVisualizer());
          ExtractionImageWidget* image_widget2 =
              new ExtractionImageWidget(std::move(data2), calibration_widget->TargetVisualizer());
          grid->addWidget(image_widget1, 0, 0);
          grid->addWidget(image_widget2, 0, 1);
          frame->setLayout(grid);

          calibration_widget->AddExtractionItem(frame);
        });
      }

      QMetaObject::invokeMethod(calibration_widget_,
                                [calibration_widget = calibration_widget_]() { calibration_widget->StartCalibration(); });

      calibrator.Calibrate(calibration1, calibration2);
    }
    catch (std::exception& ex) {
      std::string message(ex.what());

      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, message]() {
        calibration_widget->EndCalibration(new CalibrationResultWidget(message));
      });
      return false;
    }

    QMetaObject::invokeMethod(calibration_widget_,
                              [calibration_widget = calibration_widget_, &calibration1, &calibration2]() mutable {
      QWidget* frame = new QWidget();
      QVBoxLayout* layout = new QVBoxLayout(frame);
      QMargins margin = layout->contentsMargins();
      margin.setLeft(0);
      margin.setRight(0);
      layout->setContentsMargins(margin);
      layout->addWidget(new QLabel("<h2>Camera 1</h2>"));
      layout->addWidget(new CalibrationResultWidget(calibration1));
      layout->addWidget(new QLabel("<h2>Camera 2</h2>"));
      layout->addWidget(new CalibrationResultWidget(calibration2));

      calibration_widget->EndCalibration(frame);
    });

    return true;
  }
}
