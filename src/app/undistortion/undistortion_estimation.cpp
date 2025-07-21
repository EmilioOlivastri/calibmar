#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <colmap/util/misc.h>
#include "colmap/scene/camera.h"

#include "calibmar/core/pixmap.h"
#include "calibmar/core/report.h"
#include "calibmar/core/undistort.h"

void DistortPixmap(const calibmar::Pixmap& input, 
                   calibmar::Pixmap& output, 
                   const colmap::Camera& distortion_camera,
                   std::optional<double> distance);

cv::Mat UndistortPixmap(const calibmar::Pixmap& input, calibmar::Pixmap& output, 
                        const colmap::Camera& distortion_camera,
                        const std::optional<double>& prj_dist,
                        const std::optional<double>& virtual_d0,
                        int roi[4]); 

bool fixRowRoi(const cv::Mat& gray, int roi[4], int roi_id);
bool fixColRoi(const cv::Mat& gray, int roi[4], int roi_id);

int main(int argc, char** argv) 
{
    if (argc < 3) 
    {
        std::cerr << "Usage: " << argv[0] << " <camera_parameters.yaml> <path 2 image> <distance>" << std::endl;
        std::cerr << "Distance parameters is optional" << std::endl;
        return 1;
    }

    cv::Mat cv_src_img = cv::imread(argv[2], cv::IMREAD_COLOR);
    calibmar::Pixmap src_img, dst_img;
    src_img.Assign(cv_src_img.clone());
    dst_img.Assign(cv::Mat::zeros(cv_src_img.rows, cv_src_img.cols, cv_src_img.type()));

    int width = src_img.Width();
    int height = src_img.Height();

    calibmar::ImportedParameters p = calibmar::ImportedParameters::ImportFromYaml(argv[1]);
    colmap::Camera camera = calibmar::CameraModel::InitCamera(p.camera_model, {width, height}, 
                                                              p.camera_parameters);
    camera.refrac_params = p.housing_parameters;
    if (p.housing_model.value() == calibmar::HousingInterfaceType::DoubleLayerSphericalRefractive)
    {
        std::cout << "Refractive model: Dome Port" << std::endl;
        camera.refrac_model_id = colmap::CameraRefracModelId::kDomePort;
    } 
    else if (p.housing_model.value() == calibmar::HousingInterfaceType::DoubleLayerPlanarRefractive)
    {
        std::cout << "Refractive model: Flat Port" << std::endl;
        camera.refrac_model_id = colmap::CameraRefracModelId::kFlatPort;
    }
    else
    {
        std::cout << "Refractive model: Unknown" << std::endl;
        camera.refrac_model_id = colmap::CameraRefracModelId::kInvalid;
    }

    std::cout << "Type: " << camera.ModelName() << std::endl;
    std::cout << "Camera Size: " << camera.width << "x" << camera.height << std::endl;
    std::cout << "Camera parameters: " << camera.ParamsInfo() << std::endl;
    std::cout << "Parameters: " << camera.ParamsToString() << std::endl;
    std::cout << "Refractive parameters: " << camera.RefracParamsInfo() << std::endl;
    std::cout << "Refractive parameters: " << camera.RefracParamsToString() << std::endl;

    std::optional<double> prj_dist, virtual_d0;
    prj_dist = argc >=4 ? std::optional<double>(std::stod(argv[3])) : std::nullopt;
    virtual_d0 = argc >=5 ? std::optional<double>(std::stod(argv[4])) : std::nullopt;

    if (p.housing_model.value() == calibmar::HousingInterfaceType::DoubleLayerPlanarRefractive 
        && !virtual_d0.has_value() )
    {
        std::cerr << "Virtual distance is required for planar refractive model!!" << std::endl;
        return 1;
    }

    if (p.housing_model.value() == calibmar::HousingInterfaceType::DoubleLayerSphericalRefractive 
        && prj_dist.has_value() ) virtual_d0 = 0.0;

    int roi[4];
    for (int i = 0; i < 4; roi[i++] = 0);
    cv::Mat undist_map = UndistortPixmap(src_img, dst_img, camera, prj_dist, virtual_d0, roi);
    roi[2] += 1; roi[3] += 1;
    cv::Mat cv_roi = cv::Mat(1, 4, CV_32S, roi);
    for (int i = 0; i < 4; i++) cv_roi.at<int>(0, i) = roi[i];

    cv::Mat cv_dst_img = dst_img.Data();

    // Check that the roi point has neighbours
    cv::Mat gray_img;
    cv::cvtColor(cv_dst_img, gray_img, cv::COLOR_BGR2GRAY);
    std::cout << "Estimating ROI..." << std::endl;
    while(!fixColRoi(gray_img, roi, 0));
    while(!fixColRoi(gray_img, roi, 2));
    while(!fixRowRoi(gray_img, roi, 1));
    while(!fixRowRoi(gray_img, roi, 3));
    std::cout << "Estimation completed..." << std::endl;

    cv::Range cr(roi[0], roi[2]);
    cv::Range rr(roi[1], roi[3]);
    cv::Mat cv_roi_img = cv_dst_img(rr, cr).clone();

    Eigen::Matrix3d kMat_eig = camera.CalibrationMatrix();

    cv::Mat kMat_cv = cv::Mat::eye(3, 3, CV_64F);
    kMat_cv.at<double>(0, 0) = kMat_eig(0, 0);
    kMat_cv.at<double>(1, 1) = kMat_eig(1, 1);
    kMat_cv.at<double>(0, 2) = kMat_eig(0, 2);
    kMat_cv.at<double>(1, 2) = kMat_eig(1, 2);


    cv::Mat d_cv = cv::Mat::zeros(1, 5, CV_64F);    
    cv::Rect roi_rect(roi[0], roi[1], roi[2] - roi[0], roi[3] - roi[1]);
    cv::Size new_size = cv::Size(roi_rect.width, roi_rect.height);

    std::cout << "Original Size: " << cv_src_img.size() << std::endl;
    std::cout << "New Size:" << new_size << std::endl;
    std::cout << "Original K:\n" << kMat_cv << std::endl;
    std::cout << "ROI: " << roi_rect << std::endl;

    cv::rectangle(cv_dst_img, cv::Point(roi[0], roi[1]), cv::Point(roi[2], roi[3]), cv::Scalar(0, 255, 0), 2);
    cv::resize(cv_dst_img, cv_dst_img, cv::Size(1440, 900));
    cv::resize(cv_src_img, cv_src_img, cv::Size(1440, 900));
    cv::resize(cv_roi_img, cv_roi_img, cv::Size(1440, 900));

    cv::imshow("Original", cv_src_img);
    cv::imshow("Dome Reprojection", cv_dst_img);
    cv::imshow("ROI", cv_roi_img);
    cv::waitKey(0);

    cv::FileStorage fs("./undistort_map.yaml", cv::FileStorage::WRITE);
    fs << "ROI" << cv_roi;
    fs << "Undistortion Map" << undist_map;

    return 0;
}



cv::Mat UndistortPixmap(const calibmar::Pixmap& input, calibmar::Pixmap& output, 
                        const colmap::Camera& distortion_camera,
                        const std::optional<double>& prj_dist,
                        const std::optional<double>& virtual_d0,
                        int roi[4])
{
    if (input.Width() != output.Width() || input.Height() != output.Height()) 
    throw std::runtime_error("input and output size must match!");

    colmap::Camera undistort_camera = calibmar::undistort::CreateUndistortedCamera(distortion_camera);
    cv::Mat map(distortion_camera.height, distortion_camera.width, CV_32FC2);

    Eigen::Matrix3d K = undistort_camera.CalibrationMatrix();
    Eigen::Matrix3d K_inv = K.inverse();
    double dist = prj_dist.value();

    std::cout << "Building undistortion map" << std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
    roi[0] = input.Width(); roi[1] = input.Height(); roi[2] = 0.0; roi[3] = 0.0;
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
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
    std::cout << "Mapping took: " << diff << std::endl;

    std::cout << "ROI = [" << roi[0] << ", " << roi[1] << ", "
              << roi[2] << ", " << roi[3] << "]" << std::endl; 

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