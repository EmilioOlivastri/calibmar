#ifndef ASLAM_GRID_CALIBRATION_TARGET_CIRCLEGRID_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_CIRCLEGRID_HPP

#include <vector>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>

#include "GridCalibrationTargetBase.hpp"

namespace calibmar { namespace kalibr_extractors {

class GridCalibrationTargetCirclegrid : public GridCalibrationTargetBase {
 public:

  typedef boost::shared_ptr<GridCalibrationTargetCirclegrid> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetCirclegrid> ConstPtr;

  //target extraction options
  struct CirclegridOptions {
    CirclegridOptions() :
      useAsymmetricCirclegrid(false),
      showExtractionVideo(false) {};

    /// \brief asymmetric circlegrid (-->opencv)
    bool useAsymmetricCirclegrid;

    /// \brief show extracted corners
    bool showExtractionVideo;

  };

  /// \brief initialize based on circlegrid geometry
  GridCalibrationTargetCirclegrid(size_t rows, size_t cols, double spacingMeters,
                                  const GridCalibrationTargetCirclegrid::CirclegridOptions &options = CirclegridOptions());

  virtual ~GridCalibrationTargetCirclegrid() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat &image, Eigen::MatrixXd &outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief size of a circlegrid "square" [m]
  double _spacing;

  /// \brief grid options
  CirclegridOptions _options;
};

}  // namespace cameras
}  // namespace aslam

#endif /* ASLAM_GRID_CALIBRATION_TARGET_CIRCLEGRID_HPP */
