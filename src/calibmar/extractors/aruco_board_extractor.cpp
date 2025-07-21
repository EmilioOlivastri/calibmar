#include "calibmar/extractors/aruco_board_extractor.h"
#include "calibmar/apriltags/Tag16h5.h"
#include "calibmar/apriltags/Tag25h7.h"
#include "calibmar/apriltags/Tag25h9.h"
#include "calibmar/apriltags/Tag36h9.h"

namespace {
  inline int MapCornerToPointId(int marker_id, int corner_idx) {
    return marker_id * 4 + corner_idx;
  }
}

namespace calibmar {
  ArucoBoardFeatureExtractor::ArucoBoardFeatureExtractor(const Options& options) : options_(options) {
    // assuming marker ids start at 0
    // 4 corners per marker, clockwise starting top left
    uint32_t marker_corner_id = 0;
    int size = options.marker_cols * options.marker_rows;

    int width = options.marker_cols;
    int height = options.marker_rows;

    kalibr_extractors::GridCalibrationTargetAprilgrid::AprilgridOptions aprilgrid_options;
    aprilgrid_options.doSubpixRefinement = true;
    aprilgrid_options.showExtractionVideo = false;
    aprilgrid_options.minTagsForValidObs = 4;
    aprilgrid_options.minBorderDistance = 4.0;
    aprilgrid_options.blackTagBorder = options.border_bits;
    
    extractor_ = std::make_shared<kalibr_extractors::GridCalibrationTargetAprilgrid>(
        height, width, 
        options.marker_size, 
        options.marker_spacing);

    extractor_->setTagFamily(getTagCodes(options.aruco_type));

    double marker_spacing_calibmar = options.marker_size * options.marker_spacing;

    for (size_t row = 0; row < options_.marker_rows; row++) {
      for (size_t col = 0; col < options_.marker_cols; col++) {
        double x = col * (options_.marker_size + marker_spacing_calibmar);
        double y = row * (options_.marker_size + marker_spacing_calibmar);

        int marker_col, marker_row;
        switch (options.grid_origin) {
          case ArucoGridOrigin::TopLeft:
            marker_col = options.grid_direction == ArucoGridDirection::Horizontal ? col : row;
            marker_row = options.grid_direction == ArucoGridDirection::Horizontal ? row : col;
            break;
          case ArucoGridOrigin::TopRight:
            marker_col = options.grid_direction == ArucoGridDirection::Horizontal ? (width - 1) - col : row;
            marker_row = options.grid_direction == ArucoGridDirection::Horizontal ? row : (width - 1) - col;
            break;
          case ArucoGridOrigin::BottomLeft:
            marker_col = options.grid_direction == ArucoGridDirection::Horizontal ? col : (height - 1) - row;
            marker_row = options.grid_direction == ArucoGridDirection::Horizontal ? (height - 1) - row : col;
            break;
          case ArucoGridOrigin::BottomRight:
            marker_col = options.grid_direction == ArucoGridDirection::Horizontal ? (width - 1) - col : (height - 1) - row;
            marker_row = options.grid_direction == ArucoGridDirection::Horizontal ? (height - 1) - row : (width - 1) - col;
            break;
        }
        int marker_id = marker_col + marker_row * (options.grid_direction == ArucoGridDirection::Horizontal ? width : height);

        points3D_[MapCornerToPointId(marker_id, 0)] = Eigen::Vector3d(x, y, 0);
        points3D_[MapCornerToPointId(marker_id, 1)] = Eigen::Vector3d(x + options_.marker_size, y, 0);
        points3D_[MapCornerToPointId(marker_id, 2)] = Eigen::Vector3d(x + options_.marker_size, y + options_.marker_size, 0);
        points3D_[MapCornerToPointId(marker_id, 3)] = Eigen::Vector3d(x, y + options_.marker_size, 0);
      }
    }
  }

  FeatureExtractor::Status ArucoBoardFeatureExtractor::Extract(Image& image, const Pixmap& pixmap) {
    if (pixmap.Width() <= 0 || pixmap.Height() <= 0) {
      return Status::DETECTION_ERROR;
    }

    bool is_apriltag = options_.aruco_type == ArucoMarkerTypes::DICT_APRILTAG_16h5 ||
                       options_.aruco_type == ArucoMarkerTypes::DICT_APRILTAG_25h9 ||
                       options_.aruco_type == ArucoMarkerTypes::DICT_APRILTAG_36h10 ||
                       options_.aruco_type == ArucoMarkerTypes::DICT_APRILTAG_36h11;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Mat image_data(pixmap.Data());
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = std::make_shared<cv::aruco::DetectorParameters>();
    parameters->markerBorderBits = options_.border_bits;
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    if (is_apriltag) {
      // The opencv implementation of AprilTag seems to work differently
      parameters->adaptiveThreshWinSizeStep = 1;
    }

    /**/
    cv::Mat gray;
    cv::cvtColor(image_data, gray, cv::COLOR_BGR2GRAY);
    Eigen::MatrixXd outImagePoints;
    std::vector<bool> outCornerObserved;
    std::vector<AprilTags::TagDetection> detections;
    bool success_kalibr = detect(gray, outImagePoints, outCornerObserved, detections); 

    
    // store aruco keypoints in the image
    image.SetPoints2D({});
    image.ClearCorrespondences();
    std::map<int, std::vector<Eigen::Vector2d>> aruco_keypoints;

    /** ORIGINAL CODE **
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(static_cast<int>(options_.aruco_type)));
    cv::aruco::detectMarkers(image_data, dictionary, marker_corners, marker_ids, parameters, rejected_candidates);

    if (marker_corners.size() == 0) {
      return Status::DETECTION_ERROR;
    }
    
    std::map<int, size_t> id_to_idx;
    for (size_t i = 0; i < marker_ids.size(); i++) {
      // sort the points accoring to marker id, to keep stable detection for livestream
      id_to_idx[marker_ids[i]] = i;
    }
    
    for (const auto& marker_id_idx : id_to_idx) {
      std::vector<Eigen::Vector2d> corners;
      size_t i = marker_id_idx.second;
      corners.reserve(marker_corners[i].size());
      std::cout << "Tag Id: " << marker_ids[i] << std::endl;
      for (size_t j = 0; j < marker_corners[i].size(); j++) {
        // for apriltag the opencv detector sets the origin to the bottom right,
        // which is inconsistent with Aruco, so shift them to match top left origin
        size_t j2 = is_apriltag ? (j + 2) % 4 : j;  // 2, 3, 0, 1 : 0, 1, 2, 3
        const auto& corner = marker_corners[i][j2];
        Eigen::Vector2d point2D(corner.x, corner.y);
        corners.push_back(point2D);
        size_t idx = image.AddPoint2D(point2D);
        image.SetPoint3DforPoint2D(MapCornerToPointId(marker_ids[i], j), idx);
      }
      aruco_keypoints.emplace(marker_ids[i], corners);
    }

    /**/
    if ( !success_kalibr ) return Status::DETECTION_ERROR;
    
    int _cols = extractor_->cols();
    for (unsigned int i = 0; i < detections.size(); i++) 
    {
      // get the tag id
      unsigned int tagId = detections[i].id;
      
      // calculate the grid idx for all four tag corners given the tagId and cols
      unsigned int baseId = (int) (tagId / (_cols / 2)) * _cols * 2
          + (tagId % (_cols / 2)) * 2;
      unsigned int pIdx[] = { baseId, baseId + 1, baseId + (unsigned int) _cols
          + 1, baseId + (unsigned int) _cols };

      bool skip = false;
      for (int j = 0; j < 4; j++) skip |= !outCornerObserved[pIdx[j]];
      if (skip) continue;

      // add four points per tag
      std::vector<Eigen::Vector2d> corners, corners_raw;
      corners.reserve(4);
      corners_raw.reserve(4);
      int index_mapping[] = {3, 2, 1, 0};
      for (int j = 0; j < 4; j++) 
      {
        // for apriltag the kalibr detector sets the indices by rows, so 
        // to make it consistent with Aruco, we need to change the order
        int j2 = index_mapping[j];
        Eigen::Vector2d point(outImagePoints.row(pIdx[j2]));        
        corners.emplace_back(point);

        size_t idx = image.AddPoint2D(point);
        image.SetPoint3DforPoint2D(MapCornerToPointId(tagId, j), idx);
      }
      aruco_keypoints.emplace(tagId, corners);
    }


    image.SetArucoKeypoints(aruco_keypoints);

    return Status::SUCCESS;
  }


  bool ArucoBoardFeatureExtractor::detect(const cv::Mat & image, 
                                          Eigen::MatrixXd & outImagePoints,
                                          std::vector<bool> &outCornerObserved,
                                          std::vector<AprilTags::TagDetection>& detections)
  {

    bool success = true;

    // detect the tags
    detections = extractor_->getTagDetector()->extractTags(image);
    kalibr_extractors::GridCalibrationTargetAprilgrid::AprilgridOptions options = extractor_->getOptions();

    /* handle the case in which a tag is identified but not all tag
    * corners are in the image (all data bits in image but border
    * outside). tagCorners should still be okay as apriltag-lib
    * extrapolates them, only the subpix refinement will fail
    */

    //min. distance [px] of tag corners from image border (tag is not used if violated)
    std::vector<AprilTags::TagDetection>::iterator iter = detections.begin();
    for (iter = detections.begin(); iter != detections.end();) {
      // check all four corners for violation
      bool remove = false;

      for (int j = 0; j < 4; j++) {
      remove |= iter->p[j].first < options.minBorderDistance;
      remove |= iter->p[j].first > (float) (image.cols) - options.minBorderDistance;  //width
      remove |= iter->p[j].second < options.minBorderDistance;
      remove |= iter->p[j].second > (float) (image.rows) - options.minBorderDistance;  //height
      }

      //also remove tags that are flagged as bad
      if (iter->good != 1)
      remove |= true;

      //also remove if the tag ID is out-of-range for this grid (faulty detection)
      if (iter->id >= (int) extractor_->size() / 4)
      remove |= true;

      // delete flagged tags
      if (remove) {
      // delete the tag and advance in list
      iter = detections.erase(iter);
      } else {
      //advance in list
      ++iter;
      }
    }

    //did we find enough tags?
    if (detections.size() < options.minTagsForValidObs) {
      success = false;

      //immediate exit if we dont need to show video for debugging...
      //if video is shown, exit after drawing video...
      if (!options.showExtractionVideo)
      return success;
    }

    //sort detections by tagId
    std::sort(detections.begin(), detections.end(),
    AprilTags::TagDetection::sortByIdCompare);

    // check for duplicate tagIds (--> if found: wild Apriltags in image not belonging to calibration target)
    // (only if we have more than 1 tag...)
    if (detections.size() > 1) {
      for (unsigned i = 0; i < detections.size() - 1; i++)
        if (detections[i].id == detections[i + 1].id) {
          //show the duplicate tags in the image
          cv::destroyAllWindows();
          cv::namedWindow("Wild Apriltag detected. Hide them!");
          cv::startWindowThread();

          cv::Mat imageCopy = image.clone();
          cv::cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2RGB);

          //mark all duplicate tags in image
          for (int j = 0; j < detections.size() - 1; j++) {
            if (detections[j].id == detections[j + 1].id) {
              detections[j].draw(imageCopy);
              detections[j + 1].draw(imageCopy);
            }
          }

          cv::putText(imageCopy, "Duplicate Apriltags detected. Hide them.",
          cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
          CV_RGB(255,0,0), 2, 8, false);
          cv::putText(imageCopy, "Press enter to exit...", cv::Point(50, 80),
          cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255,0,0), 2, 8, false);
          cv::imshow("Duplicate Apriltags detected. Hide them", imageCopy);  // OpenCV call

          cv::waitKey();
          exit(0);
        }
    }

    // convert corners to cv::Mat (4 consecutive corners form one tag)
    /// point ordering here
    ///          11-----10  15-----14
    ///          | TAG 2 |  | TAG 3 |
    ///          8-------9  12-----13
    ///          3-------2  7-------6
    ///    y     | TAG 0 |  | TAG 1 |
    ///   ^      0-------1  4-------5
    ///   |-->x
    cv::Mat tagCorners(4 * detections.size(), 2, CV_32F);

    for (unsigned i = 0; i < detections.size(); i++) {
      for (unsigned j = 0; j < 4; j++) {
        tagCorners.at<float>(4 * i + j, 0) = detections[i].p[j].first;
        tagCorners.at<float>(4 * i + j, 1) = detections[i].p[j].second;
      }
    }

    //store a copy of the corner list before subpix refinement
    cv::Mat tagCornersRaw = tagCorners.clone();

    //optional subpixel refinement on all tag corners (four corners each tag)
    if (options.doSubpixRefinement && success)
      cv::cornerSubPix(image, tagCorners, cv::Size(2, 2), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::Type::EPS + cv::TermCriteria::Type::MAX_ITER, 30, 0.1));

    if (options.showExtractionVideo) {
      //image with refined (blue) and raw corners (red)
      cv::Mat imageCopy1 = image.clone();
      cv::cvtColor(imageCopy1, imageCopy1, cv::COLOR_GRAY2RGB);
      for (unsigned i = 0; i < detections.size(); i++)
        for (unsigned j = 0; j < 4; j++) {
          //raw apriltag corners
          //cv::circle(imageCopy1, cv::Point2f(detections[i].p[j].first, detections[i].p[j].second), 2, CV_RGB(255,0,0), 1);

          //subpixel refined corners
          cv::circle(
          imageCopy1,
          cv::Point2f(tagCorners.at<float>(4 * i + j, 0),
          tagCorners.at<float>(4 * i + j, 1)),
          3, CV_RGB(0,0,255), 1);

          if (!success)
          cv::putText(imageCopy1, "Detection failed! (frame not used)",
          cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
          CV_RGB(255,0,0), 3, 8, false);
        }

      cv::imshow("Aprilgrid: Tag corners", imageCopy1);  // OpenCV call
      cv::waitKey(1);

      /* copy image for modification */
      cv::Mat imageCopy2 = image.clone();
      cv::cvtColor(imageCopy2, imageCopy2, cv::COLOR_GRAY2RGB);
      /* highlight detected tags in image */
      for (unsigned i = 0; i < detections.size(); i++) {
        detections[i].draw(imageCopy2);

        if (!success)
          cv::putText(imageCopy2, "Detection failed! (frame not used)",
          cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
          CV_RGB(255,0,0), 3, 8, false);
      }

      cv::imshow("Aprilgrid: Tag detection", imageCopy2);  // OpenCV call
      cv::waitKey(1);

      //if success is false exit here (delayed exit if _options.showExtractionVideo=true for debugging)
      if (!success) return success;
    }

    //insert the observed points into the correct location of the grid point array
    /// point ordering
    ///          12-----13  14-----15
    ///          | TAG 2 |  | TAG 3 |
    ///          8-------9  10-----11
    ///          4-------5  6-------7
    ///    y     | TAG 0 |  | TAG 1 |
    ///   ^      0-------1  2-------3
    ///   |-->x

    outCornerObserved.resize(extractor_->size(), false);
    outImagePoints.resize(extractor_->size(), 2);

    int _cols = extractor_->cols();

    for (unsigned int i = 0; i < detections.size(); i++) {
      // get the tag id
      unsigned int tagId = detections[i].id;

      // calculate the grid idx for all four tag corners given the tagId and cols
      unsigned int baseId = (int) (tagId / (_cols / 2)) * _cols * 2
      + (tagId % (_cols / 2)) * 2;
      unsigned int pIdx[] = { baseId, baseId + 1, baseId + (unsigned int) _cols
      + 1, baseId + (unsigned int) _cols };

      // add four points per tag
      for (int j = 0; j < 4; j++) {
        //refined corners
        double corner_x = tagCorners.row(4 * i + j).at<float>(0);
        double corner_y = tagCorners.row(4 * i + j).at<float>(1);

        //raw corners
        double cornerRaw_x = tagCornersRaw.row(4 * i + j).at<float>(0);
        double cornerRaw_y = tagCornersRaw.row(4 * i + j).at<float>(1);

        //only add point if the displacement in the subpixel refinement is below a given threshold
        double subpix_displacement_squarred = (corner_x - cornerRaw_x)
        * (corner_x - cornerRaw_x)
        + (corner_y - cornerRaw_y) * (corner_y - cornerRaw_y);

        //add all points, but only set active if the point has not moved to far in the subpix refinement
        outImagePoints.row(pIdx[j]) = Eigen::Matrix<double, 1, 2>(corner_x,
                                  corner_y);

        if (subpix_displacement_squarred <= options.maxSubpixDisplacement2) {
          outCornerObserved[pIdx[j]] = true;
        } else {
          outCornerObserved[pIdx[j]] = false;
        }
      }
    }

    //succesful observation
    return success;
  }


  AprilTags::TagCodes ArucoBoardFeatureExtractor::getTagCodes(const ArucoMarkerTypes& type)
  {
    
    switch (type) {
      case ArucoMarkerTypes::DICT_APRILTAG_36h11:
        return AprilTags::tagCodes36h11;
      case ArucoMarkerTypes::DICT_APRILTAG_16h5:
        return AprilTags::tagCodes16h5;
      case ArucoMarkerTypes::DICT_APRILTAG_25h9:
        return AprilTags::tagCodes25h9;
    }

    return AprilTags::tagCodes36h11;
  }
}
