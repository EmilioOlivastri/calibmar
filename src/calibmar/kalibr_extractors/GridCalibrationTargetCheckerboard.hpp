#ifndef ASLAM_GRID_CALIBRATION_TARGET_CHECKERBOARD_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_CHECKERBOARD_HPP

#include <vector>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>

#include "GridCalibrationTargetBase.hpp"

namespace calibmar { namespace kalibr_extractors {

class GridCalibrationTargetCheckerboard : public GridCalibrationTargetBase {
 public:

  typedef boost::shared_ptr<GridCalibrationTargetCheckerboard> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetCheckerboard> ConstPtr;

  //target extraction options
  struct CheckerboardOptions {
    CheckerboardOptions() :
      useAdaptiveThreshold(true),
      normalizeImage(true),
      performFastCheck(true),
      filterQuads(false),
      doSubpixelRefinement(true),
      showExtractionVideo(false),
      windowWidth(11) {};

    /// \brief opencv options
    bool useAdaptiveThreshold;
    bool normalizeImage;
    bool performFastCheck;
    bool filterQuads;
    bool doSubpixelRefinement;
    unsigned int windowWidth;

    /// \brief show extracted corners
    bool showExtractionVideo;

  };

  /// \brief initialize based on checkerboard geometry
  GridCalibrationTargetCheckerboard(size_t rows, size_t cols, double rowSpacingMeters,
                                    double colSpacingMeters,
                                    const GridCalibrationTargetCheckerboard::CheckerboardOptions &options = CheckerboardOptions());

  virtual ~GridCalibrationTargetCheckerboard() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat &image, Eigen::MatrixXd &outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief size of a checkerboard square in rows direction [m]
  double _rowSpacingMeters;

  /// \brief size of a checkerboard square in cols direction [m]
  double _colSpacingMeters;

  /// \brief checkerboard extraction options
  CheckerboardOptions _options;

};

}  // namespace cameras
}  // namespace aslam


#endif /* ASLAM_GRID_CALIBRATION_TARGET_CHECKERBOARD_HPP */
