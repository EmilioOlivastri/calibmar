#include <utility>
#include "GridCalibrationTargetBase.hpp" // ok

namespace calibmar { namespace kalibr_extractors {

GridCalibrationTargetBase::GridCalibrationTargetBase(size_t rows, size_t cols)
    : _rows(rows),
      _cols(cols) {
}

/// \brief get all points from the target expressed in the target frame
Eigen::MatrixXd GridCalibrationTargetBase::points() const {
  return _points;
}

/// \brief get a point from the target expressed in the target frame
Eigen::Vector3d GridCalibrationTargetBase::point(size_t i) const {
  return _points.row(i);
}

/// \brief get the grid coordinates for a point
std::pair<size_t, size_t> GridCalibrationTargetBase::pointToGridCoordinates(size_t i) const {
  return std::pair<size_t, size_t>(i % cols(), (int) i / cols());
}

/// \brief get the point index from the grid coordinates
size_t GridCalibrationTargetBase::gridCoordinatesToPoint(size_t r, size_t c) const {
  return cols() * r + c;
}

/// \brief get a point from the target expressed in the target frame
///        by row and column
Eigen::Vector3d GridCalibrationTargetBase::gridPoint(size_t r, size_t c) const {
  return _points.row(gridCoordinatesToPoint(r, c));
}

double * GridCalibrationTargetBase::getPointDataPointer(size_t i) {
  return &_points(i, 0);
}

}  // namespace cameras
}  // namespace aslam
