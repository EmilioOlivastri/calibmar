#include "calibmar/calibrators/stereo_calibrator.h"
#include "calibmar/calibrators/general_calibration.h"
#include "calibmar/calibrators/stereo_calibration.h"

#include <colmap/sensor/models.h>

namespace calibmar {

  void StereoCalibrator::Options::Check() {
    if (!use_intrinsics_guess && (image_size.first == 0 || image_size.second == 0)) {
      throw std::runtime_error("Image size must be set!.");
    }
  }

  StereoCalibrator::StereoCalibrator(const Options& options) : options_(options) {}

  void StereoCalibrator::Calibrate(Calibration& calibration1, Calibration& calibration2) {
    options_.Check();

    if (calibration1.Images().size() == 0) {
      throw std::runtime_error("No images to calibrate from.");
    }
    if (calibration1.Images().size() != calibration2.Images().size()) {
      throw std::runtime_error("Stereo calibration images are not balanced.");
    }

    if (calibration1.Points3D().size() == 0) {
      throw std::runtime_error("3D Points not set.");
    }

    colmap::Camera& camera1 = calibration1.Camera();
    colmap::Camera& camera2 = calibration2.Camera();
    if (options_.use_intrinsics_guess &&
        (camera1.model_id == colmap::CameraModelId::kInvalid || camera2.model_id == colmap::CameraModelId::kInvalid)) {
      throw std::runtime_error("Intrinsics guess specified, but camera not initialized.");
    }

    if (!options_.use_intrinsics_guess) {
      camera1.width = options_.image_size.first;
      camera1.height = options_.image_size.second;
      camera1.model_id = colmap::CameraModelNameToId(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
      camera2.width = options_.image_size.first;
      camera2.height = options_.image_size.second;
      camera2.model_id = colmap::CameraModelNameToId(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
    }

    std::vector<std::vector<Eigen::Vector2d>> pointSets2D_1, pointSets2D_2;
    std::vector<std::vector<Eigen::Vector3d>> pointSets3D_1, pointSets3D_2;
    
    /* EMILIO'S ADAPTATION CODE FOR ARUCO BOARDS*/
    std::string full_calib_info = calibration1.GetCalibrationTargetInfo();
    std::string target_type = full_calib_info.substr(0, full_calib_info.find(","));
    std::string aruco_board = "aruco grid board";
    std::cout << "target type: " << target_type << std::endl;
    if ( target_type == aruco_board) {
      // remove the 3D points that are not visible in both images
      std::vector<std::vector<Eigen::Vector3d>> pts3D_1_filt, pts3D_2_filt;
      std::vector<std::vector<Eigen::Vector2d>> pts2D_1_filt, pts2D_2_filt;
      int num_imgs = calibration1.Images().size();
      pts2D_1_filt.resize(num_imgs);
      pts2D_2_filt.resize(num_imgs);
      pts3D_1_filt.resize(num_imgs); 
      pts3D_2_filt.resize(num_imgs);
      for (size_t im_id = 0; im_id < num_imgs; ++im_id) {
        const Image& image1 = calibration1.Images()[im_id];
        const Image& image2 = calibration2.Images()[im_id];
        std::unordered_map<uint32_t, size_t> corr1 = image1.ReverseCorrespondences();
        std::unordered_map<uint32_t, size_t> corr2 = image2.ReverseCorrespondences();

        int min_num_points = std::min(corr1.size(), corr2.size());
        pts2D_1_filt[im_id].reserve(min_num_points);
        pts2D_2_filt[im_id].reserve(min_num_points);
        pts3D_1_filt[im_id].reserve(min_num_points);
        pts3D_2_filt[im_id].reserve(min_num_points);
        
        for (const auto& it_x : corr1) {
          
          uint32_t point3D_idx = it_x.first;
          size_t point2D_idx = it_x.second;
          const auto& it_y = corr2.find(point3D_idx);

          if (it_y == corr2.end()) continue;

          uint32_t point3D_idy = it_y->first;
          size_t point2D_idy = it_y->second;
          /**
          bool condition_printing = false;
          condition_printing |= point2D_idy != point2D_idx;
          condition_printing |= point3D_idx != corr2[point2D_idy];
          condition_printing |= point2D_idy != corr2[point2D_idy];
          condition_printing |= point3D_idx != point3D_idx; 
          if (condition_printing)
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
          /**/

          pts2D_1_filt[im_id].push_back(image1.Point2D(point2D_idx));
          pts3D_1_filt[im_id].push_back(calibration1.Point3D(point3D_idx));
          pts2D_2_filt[im_id].push_back(image2.Point2D(point2D_idy));
          pts3D_2_filt[im_id].push_back(calibration2.Point3D(point3D_idy));


        }

      }

      pointSets2D_1 = pts2D_1_filt;
      pointSets2D_2 = pts2D_2_filt;
      pointSets3D_1 = pts3D_1_filt;
      pointSets3D_2 = pts3D_2_filt;
    }
    else
    {
      calibration1.GetCorrespondences(pointSets2D_1, pointSets3D_1);
      calibration2.GetCorrespondences(pointSets2D_2, pointSets3D_2);
    }
    /* EMILIO'S ADAPTATION CODE FOR ARUCO BOARDS*/


    std::vector<std::vector<double>> per_view_rms(2);
    colmap::Rigid3d relative_pose;
    std::vector<colmap::Rigid3d> poses;

    std::vector<double> std_dev_camera1;
    std::vector<double> std_dev_camera2;
    stereo_calibration::StereoStdDeviations std_devs;
    std_devs.std_deviations_intrinsics1 = &std_dev_camera1;
    std_devs.std_deviations_intrinsics2 = &std_dev_camera2;
    // extrinsics std devs are currently not used, because their interpretation is unclear
    stereo_calibration::CalibrateStereoCameras(pointSets3D_1, pointSets2D_1, pointSets2D_2, camera1, camera2,
                                               options_.use_intrinsics_guess, options_.estimate_pose_only, relative_pose, poses,
                                               &std_devs);

    std::vector<colmap::Rigid3d> poses2;
    poses2.reserve(poses.size());
    for (const auto& pose1 : poses) {
      // calculate poses of the second camera
      poses2.push_back(relative_pose * pose1);
    }

    double rms1 = general_calibration::CalculateOverallRMS(pointSets3D_1, pointSets2D_1, poses, camera1, per_view_rms[0]);
    double rms2 = general_calibration::CalculateOverallRMS(pointSets3D_1, pointSets2D_2, poses2, camera2, per_view_rms[1]);

    calibration1.SetCalibrationRms(rms1);
    calibration1.SetPerViewRms(per_view_rms[0]);
    calibration1.SetIntrinsicsStdDeviations(*std_devs.std_deviations_intrinsics1);
    calibration2.SetCalibrationRms(rms2);
    calibration2.SetPerViewRms(per_view_rms[1]);
    calibration2.SetIntrinsicsStdDeviations(*std_devs.std_deviations_intrinsics2);

    // The calibration pose is defined as camera to world and camera 1 is supposed to be world here
    // The pose from StereoCalibrateCamera() is camera1 to camera2, so we need to invert it here (to get 2 to 1, i.e. 2 to world).
    calibration1.SetCameraToWorldStereo(colmap::Rigid3d());
    calibration2.SetCameraToWorldStereo(colmap::Inverse(relative_pose));
  }
}