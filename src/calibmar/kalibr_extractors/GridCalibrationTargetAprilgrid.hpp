#ifndef ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HPP

#include <vector>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include "GridCalibrationTargetBase.hpp"

// April tags detector and various tag families
#include "calibmar/apriltags/TagDetector.h"
//#include "apriltags/Tag16h5.h"
//#include "apriltags/Tag25h7.h"
//#include "apriltags/Tag25h9.h"
//#include "apriltags/Tag36h9.h"
#include "calibmar/apriltags/Tag36h11.h"


namespace calibmar 
{
namespace kalibr_extractors 
{

class GridCalibrationTargetAprilgrid : public GridCalibrationTargetBase 
{
  public:

  typedef boost::shared_ptr<GridCalibrationTargetAprilgrid> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetAprilgrid> ConstPtr;

  //target extraction options
  struct AprilgridOptions 
  {
      AprilgridOptions() : doSubpixRefinement(true),
                           maxSubpixDisplacement2(1.5),
                           showExtractionVideo(false),
                           minTagsForValidObs(4),
                           minBorderDistance(4.0),
                           blackTagBorder(2) {};

      //options
      /// \brief subpixel refinement of extracted corners
      bool doSubpixRefinement;

      /// \brief max. displacement squarred in subpixel refinement  [px^2]
      double maxSubpixDisplacement2;

      /// \brief show video during extraction
      bool showExtractionVideo;

      /// \brief min. number of tags for a valid observation
      unsigned int minTagsForValidObs;

      /// \brief min. distance form image border for valid points [px]
      double minBorderDistance;

      /// \brief size of black border around the tag code bits (in pixels)
      unsigned int blackTagBorder;
  };

  /// \brief initialize based on checkerboard geometry
  GridCalibrationTargetAprilgrid(size_t tagRows, size_t tagCols, double tagSize,
                                 double tagSpacing, 
                                 const AprilgridOptions &options = AprilgridOptions());

  virtual ~GridCalibrationTargetAprilgrid() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat & image,
                          Eigen::MatrixXd & outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

  void setTagFamily(const AprilTags::TagCodes& tagFamily) { _tagCodes = tagFamily; }

  inline boost::shared_ptr<AprilTags::TagDetector> getTagDetector() const { return _tagDetector; }
  inline AprilgridOptions getOptions() const { return _options; }


 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief size of a tag [m]
  double _tagSize;

  /// \brief space between tags (tagSpacing [m] = tagSize * tagSpacing)
  double _tagSpacing;

  /// \brief target extraction options
  AprilgridOptions _options;

  // create a detector instance
  AprilTags::TagCodes _tagCodes;
  boost::shared_ptr<AprilTags::TagDetector> _tagDetector;

};


}  // namespace cameras
}  // namespace aslam


#endif /* ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HPP */

