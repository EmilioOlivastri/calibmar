#include "undistortion_estimation_runner.h"
#include "ui/widgets/undistortion_estimation_result_widget.h"
#include "calibmar/core/undistort.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace {
  void SetupUndistortionEstimation(const calibmar::UndistortionEstimationDialog::Options& options, 
                                   const std::pair<int, int>& image_size,
                                   colmap::Camera& camera,
                                   std::optional<double>& prj_dist,
                                   std::optional<double>& virtual_d0) {
    using namespace calibmar;

    camera = CameraModel::InitCamera(options.camera_model, image_size, 
                                     options.camera_parameters.value());

    camera.refrac_params = options.housing_calibration.value().second;
    calibmar::HousingInterfaceType housing_model = options.housing_calibration.value().first;
    camera.refrac_model_id = colmap::CameraRefracModelId::kInvalid;
    if (housing_model == calibmar::HousingInterfaceType::DoubleLayerSphericalRefractive)
    {
        camera.refrac_model_id = colmap::CameraRefracModelId::kDomePort;
        virtual_d0 = 0.0;
    }
    else if (housing_model == calibmar::HousingInterfaceType::DoubleLayerPlanarRefractive)
    {
        camera.refrac_model_id = colmap::CameraRefracModelId::kFlatPort;
        virtual_d0 = options.virtual_d0;
    }

    prj_dist = options.projection_distance;

    /**
    std::cout << "Type: " << camera.ModelName() << std::endl;
    std::cout << "Camera Size: " << camera.width << "x" << camera.height << std::endl;
    std::cout << "Camera parameters: " << camera.ParamsInfo() << std::endl;
    std::cout << "Parameters: " << camera.ParamsToString() << std::endl;
    std::cout << "Refractive parameters: " << camera.RefracParamsInfo() << std::endl;
    std::cout << "Refractive parameters: " << camera.RefracParamsToString() << std::endl;
    std::cout << "Projection distance: " 
              << (prj_dist.has_value() ? std::to_string(prj_dist.value()) : "not set") << std::endl;
    std::cout << "Virtual d0: "
              << (virtual_d0.has_value() ? std::to_string(virtual_d0.value()) : "not set") << std::endl;
    /**/

  }

  cv::Mat UndistortPixmap(const calibmar::Pixmap& input, calibmar::Pixmap& output, 
                        const colmap::Camera& distortion_camera,
                        const std::optional<double>& prj_dist,
                        const std::optional<double>& virtual_d0,
                        int roi[4],
                        calibmar::UndistortionEstimationWidget* undistortion_widget)
    {
        if (input.Width() != output.Width() || input.Height() != output.Height()) 
        throw std::runtime_error("input and output size must match!");

        colmap::Camera undistort_camera = calibmar::undistort::CreateUndistortedCamera(distortion_camera);
        cv::Mat map(distortion_camera.height, distortion_camera.width, CV_32FC2);
        Eigen::Matrix3d K = undistort_camera.CalibrationMatrix();
        Eigen::Matrix3d K_inv = K.inverse();
        double dist = prj_dist.value();

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        roi[0] = input.Width(); roi[1] = input.Height(); roi[2] = 0.0; roi[3] = 0.0;
        double progress_step = 100.0 / input.Height();
        int prev_progress = 0;
        for (int y = 0; y < input.Height(); y++) 
        {
            #pragma omp parallel for
            for (int x = 0; x < input.Width(); x++) 
            {
                Eigen::Vector2d src_px;
                if (prj_dist.has_value() && distortion_camera.IsCameraRefractive()) 
                {
                    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
                    offset[2] = virtual_d0.has_value() ? virtual_d0.value() : 0.0; 
                    Eigen::Vector3d uv_coords_homo(x, y, 1.0);
                    Eigen::Vector3d point2D = K_inv * uv_coords_homo;
                    Eigen::Vector3d point3D = dist * point2D + offset;
                    src_px = distortion_camera.ImgFromCamRefrac(point3D);

                    bool in_image_x = src_px.x() > 0 && src_px.x() < input.Width();
                    bool in_image_y = src_px.y() > 0 && src_px.y() < input.Height();
                    bool in_image = in_image_x && in_image_y; 
                    
                    roi[0] = x < roi[0] && in_image ? x : roi[0];
                    roi[1] = y < roi[1] && in_image ? y : roi[1];
                    roi[2] = x > roi[2] && in_image ? x : roi[2];
                    roi[3] = y > roi[3] && in_image ? y : roi[3];
                }

                map.at<cv::Vec2f>(y, x) = cv::Vec2f(src_px.x(), src_px.y());
            }
            int progress = std::min(100, static_cast<int>((y+1) * progress_step));
            if (prev_progress < progress) undistortion_widget->SetProgress(progress);
        }
        undistortion_widget->SetProgress(100);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
        //std::cout << "Mapping took: " << diff << std::endl;
        cv::remap(input.Data(), output.Data(), map, cv::noArray(), cv::INTER_LINEAR, cv::BorderTypes::BORDER_CONSTANT);
        
        return map;
    }

    void DistortPixmap(const calibmar::Pixmap& input, 
                    calibmar::Pixmap& output, 
                    const colmap::Camera& distortion_camera,
                    std::optional<double> distance) 
    {
        if (input.Width() != output.Width() || input.Height() != output.Height()) 
        throw std::runtime_error("input and output size must match!");

        colmap::Camera undistort_camera = calibmar::undistort::CreateUndistortedCamera(distortion_camera);
        cv::Mat map(distortion_camera.height, distortion_camera.width, CV_32FC2);

        for (int y = 0; y < input.Height(); y++) 
        {
            for (int x = 0; x < input.Width(); x++) 
            {
                Eigen::Vector2d src_px;
                if (distance.has_value() && distortion_camera.IsCameraRefractive()) 
                {
                    Eigen::Vector3d point3D = distortion_camera.CamFromImgRefracPoint({x, y}, *distance);
                    src_px = undistort_camera.ImgFromCam(point3D.hnormalized());
                }
                else 
                {
                    Eigen::Vector2d cam_point = distortion_camera.CamFromImg({x, y});
                    src_px = undistort_camera.ImgFromCam(cam_point);
                }
                map.at<cv::Vec2f>(y, x) = cv::Vec2f(src_px.x(), src_px.y());
            }
        }

        cv::remap(input.Data(), output.Data(), map, cv::noArray(), cv::INTER_LINEAR, cv::BorderTypes::BORDER_CONSTANT);

        return;
    }

    bool fixRowRoi(const cv::Mat& gray, int roi[4], int roi_id)
    {
        int non_zero_count = 0;
        int min_neighbours = 4;

        int mid_col = gray.cols / 2;
        int iters = 15;
        
        for (int r_id = -iters; r_id <= iters; r_id++) 
        {
            if (r_id == 0 ) continue;
            
            int row = roi[roi_id] + r_id;

            if (row >= 0 && row < gray.rows && gray.at<uchar>(row, mid_col) > 0) 
                non_zero_count++;
        }

        // Skip if there are not enough neighbours
        if (non_zero_count > min_neighbours) return true;

        // Find new roi
        bool found = false;
        int incr = roi_id == 1 ? 1 : -1;
        for (int r_id = roi[roi_id % 4] + incr; r_id > roi[1] && r_id < roi[3] && !found; r_id += incr) 
        {
            if (gray.at<uchar>(r_id, mid_col) == 0) continue; 
            
            roi[roi_id] = r_id;
            found = true;
        }

        return false;
    }

    bool fixColRoi(const cv::Mat& gray, int roi[4], int roi_id)
    {
        int non_zero_count = 0;
        int min_neighbours = 4;

        int mid_row = gray.rows / 2;
        int iters = 15;

        for (int c_id = -iters; c_id <= iters; c_id++) 
        {
            if (c_id == 0) continue;
            
            int col = roi[roi_id] + c_id;

            if (col >= 0 && col < gray.cols && gray.at<uchar>(mid_row, col) > 0) 
                non_zero_count++;
        }

        // Skip if there are not enough neighbours
        if (non_zero_count > min_neighbours) return true;

        // Find new roi
        bool found = false;
        int incr = roi_id == 0 ? 1 : -1;
        for (int c_id = roi[roi_id % 4] + incr; c_id > roi[0] && c_id < roi[2] && !found; c_id += incr) 
        {
            if (gray.at<uchar>(mid_row, c_id) == 0) continue; 
            
            roi[roi_id] = c_id;
            found = true;
        }


        return false;
    }

  
}

namespace calibmar {

  UndistortionEstimationRunner::UndistortionEstimationRunner(UndistortionEstimationWidget* undistortion_widget, UndistortionEstimationDialog::Options options)
      : undistortion_widget_(undistortion_widget), options_(options) {}

  bool UndistortionEstimationRunner::Run() {
    
    cv::Mat cv_src_img = cv::imread(options_.image_path, cv::IMREAD_UNCHANGED);
    std::unique_ptr<Pixmap> src_pixmap = std::make_unique<Pixmap>();
    Image src_image_data;
    src_image_data.SetName(options_.image_path);

    std::unique_ptr<Pixmap> dst_pixmap = std::make_unique<Pixmap>();    
    src_pixmap->Assign(cv_src_img.clone());
    dst_pixmap->Assign(cv::Mat::zeros(cv_src_img.rows, cv_src_img.cols, cv_src_img.type()));
    std::pair<int, int> image_size = {src_pixmap->Width(), src_pixmap->Height()};
    colmap::Camera camera;
    std::optional<double> prj_dist, virtual_d0;
    SetupUndistortionEstimation(options_, image_size, camera, prj_dist, virtual_d0);
    
    std::unique_ptr<Pixmap> original_pixmap = std::make_unique<Pixmap>(src_pixmap->Clone());

    std::unique_ptr<TargetVisualizer> target_visualizer = std::make_unique<ImageTargetVisualizer>(image_size.first, image_size.second);
    undistortion_widget_->SetTargetVisualizer(std::move(target_visualizer));

    std::unique_ptr<ExtractionImageWidget::Data> data_input = std::make_unique<ExtractionImageWidget::Data>();
    data_input->image_name = options_.image_path;
    data_input->image_data = src_image_data;
    data_input->image = std::make_unique<Pixmap>(src_pixmap->Clone());
    data_input->status = ExtractionImageWidget::ConvertStatus(ImageReader::Status::SUCCESS);

    QMetaObject::invokeMethod(undistortion_widget_,
                              [undistortion_widget = undistortion_widget_, data = std::move(data_input)]() mutable {
                              undistortion_widget->AddExtractionItem(new ExtractionImageWidget(std::move(data), undistortion_widget->TargetVisualizer()));
                            });

    
    QMetaObject::invokeMethod(undistortion_widget_,
                              [undistortion_widget = undistortion_widget_]() { 
                               undistortion_widget->StartUndistortionEstimation(); 
                            });

    /* UNDISTORTION ESTIMATION  HERE */
    int roi[4];
    for (int i = 0; i < 4; roi[i++] = 0);
    /**/
    cv::Mat undist_map = UndistortPixmap(*src_pixmap, *dst_pixmap, 
                                         camera, prj_dist, virtual_d0, roi,
                                         undistortion_widget_);
    /**
    cv::Mat undist_map = cv::Mat::zeros(image_size.second, image_size.first, cv_src_img.type());
    /**/
    roi[2] += 1; roi[3] += 1;
    
    cv::Mat cv_roi = cv::Mat(1, 4, CV_32S, roi);
    for (int i = 0; i < 4; i++) cv_roi.at<int>(0, i) = roi[i];
    cv::Mat cv_dst_img = dst_pixmap->Data();

    // Check that the roi point has neighbours
    cv::Mat gray_img = cv_dst_img.clone();
    if (gray_img.channels() > 1)
        cv::cvtColor(gray_img, gray_img, cv::COLOR_BGR2GRAY);
    /**/
    while(!fixColRoi(gray_img, roi, 0));
    while(!fixColRoi(gray_img, roi, 2));
    while(!fixRowRoi(gray_img, roi, 1));
    while(!fixRowRoi(gray_img, roi, 3));
    /**/
    
    cv::Range cr(roi[0], roi[2]);
    cv::Range rr(roi[1], roi[3]);
    cv::Mat cv_roi_img = cv_dst_img(rr, cr).clone();
    dst_pixmap->Assign(cv_roi_img);
    /* UNDISTORTION ESTIMATION  HERE */

    cv::FileStorage fs(options_.output_map + "undistortion_map.yaml", cv::FileStorage::WRITE);
    fs << "ROI" << cv_roi;
    fs << "Undistortion Map" << undist_map;
    fs.release(); 
    
    // Save the undistorted image
    std::unique_ptr<Pixmap> undistorted_pixmap = std::make_unique<Pixmap>(dst_pixmap->Clone());
    //std::unique_ptr<Pixmap> undistorted_pixmap = std::make_unique<Pixmap>();
    //undistorted_pixmap->Assign(cv_dst_img.clone());

    std::unique_ptr<ExtractionImageWidget::Data> data_output = std::make_unique<ExtractionImageWidget::Data>();
    data_output->image_name = options_.image_path;
    data_output->image_data = src_image_data;
    data_output->image = std::move(dst_pixmap);
    data_output->status = ExtractionImageWidget::ConvertStatus(ImageReader::Status::SUCCESS);

    QMetaObject::invokeMethod(undistortion_widget_,
                              [undistortion_widget = undistortion_widget_, 
                               original_pixmap = std::move(original_pixmap),
                               undistorted_pixmap = std::move(undistorted_pixmap),
                               camera = camera,
                               distances = std::make_pair(prj_dist.value_or(0.0), virtual_d0.value_or(0.0))]() mutable {
                               undistortion_widget->EndUndistortionEstimation(new UndistortionEstimationResultWidget(camera, std::move(original_pixmap), std::move(undistorted_pixmap), distances));
                              });
    


    return true;


  }
}
