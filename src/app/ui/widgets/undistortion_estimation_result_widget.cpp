#include "undistortion_estimation_result_widget.h"

#include "calibmar/core/report.h"
#include "ui/utils/heatmap.h"
#include "ui/widgets/collapsible_widget.h"
#include "ui/widgets/image_widget.h"
#include "ui/widgets/offset_diagram_widget.h"
#include "ui/widgets/zoomable_scroll_area.h"

#include <colmap/ui/model_viewer_widget.h>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <regex>

namespace {
  std::string cssStyle = R"(
h3 {
  margin: 0px 0px 0px 0px;
}

p {
  margin: 0px 0px 10px 0px;
}

table {
  margin: 0px 0px 10px 0px;  
}

td {
  padding-right: 15px;
}
)";

  Eigen::IOFormat htmlTableFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "</td>\n<td>", "", "<tr>\n<td>", "</td>\n</tr>\n",
                                  "<table>\n<tbody>\n", "</tbody>\n</table>");

  std::vector<std::string> Split(const std::string& input, const std::string& regex) {
    // passing -1 as the submatch index parameter performs splitting
    std::regex re(regex);
    std::sregex_token_iterator first{input.begin(), input.end(), re, -1}, last;
    return {first, last};
  }

  void FormatTableRows(std::ostream& stream, const std::vector<std::string>& first_col, const std::vector<std::string>& first_row,
                       const std::vector<double>& second_row, const std::vector<double>& third_row) {
    stream << "<table>\n<tbody>\n<tr>"
           << "<td><b>" << first_col[0] << ":</b></td>\n";
    for (auto& value : first_row) {
      stream << "<td><b>" << value << "</b></td>\n";
    }
    stream << "</tr>\n<tr>";
    // parameter values
    stream << "<td><b>" << first_col[1] << "</b></td>";
    for (auto& value : second_row) {
      stream << "<td>" << value << "</td>\n";
    }
    stream << "</tr>\n";
    // parameter std dev
    if (third_row.size() > 0) {
      stream << "<tr>\n<td><b>" << first_col[2] << "</b></td>";
      for (auto& value : third_row) {
        stream << "<td>" << value << "</td>\n";
      }
      stream << "</tr>";
    }

    stream << "\n</tbody>\n</table>\n";
  }

  void GenerateResultHtml(std::ostream& stream, const colmap::Camera& camera, 
                          const std::pair<double, double>& distances) {
    // Title
    stream << "<h2>Undistortion Estimation Summary</h2>" << std::endl;
    
    // camera model
    std::string header = "<h3>Camera &amp; Housing Model:</h3>";
    stream << header << std::endl << "<p>" << camera.ModelName();
    stream << " " << camera.RefracModelName();
    stream << "</p>" << std::endl << std::endl;
    // width & height
    stream << "<h3>Width &amp; Height:</h3>" << std::endl;
    stream << "<p>" << camera.width << " " << camera.height << "</p>";
    stream << std::endl << std::endl;
    // camera matrix
    stream << "<h3>Camera Matrix:</h3>" << std::endl
           << camera.CalibrationMatrix().format(htmlTableFormat) << std::endl
           << std::endl;
    
    // Undistortion parameters
    stream << "<h3>Projection Distance &amp; Virtual D0:</h3>" << std::endl;
    stream << "<p>" << distances.first << " " << distances.second << "</p>";
    stream << std::endl << std::endl;
    
  }

  void GeneratePerViewHtml(std::ostream& stream, const calibmar::Calibration& calibration) {
    // per view rms & per view observation
    if (calibration.PerViewRms().size() > 0) {
      struct Stats {
        std::string name;
        double rms;
        int point_3d_count;
      };
      std::vector<Stats> stats;
      bool contains_3d_count = calibration.PerView3DPointCount().size() > 0;
      stats.reserve(calibration.Images().size());
      for (size_t i = 0; i < calibration.Images().size(); i++) {
        stats.push_back({std::filesystem::path(calibration.Image(i).Name()).filename().string(), calibration.PerViewRms()[i],
                         contains_3d_count ? calibration.PerView3DPointCount()[i] : -1});
      }
      std::sort(stats.begin(), stats.end(), [](Stats& a, Stats& b) { return a.rms > b.rms; });

      stream << std::endl
             << std::endl
             << "<h3>Per View RMS (" << stats.size() << " images, ordered descending):</h3>" << std::endl;
      stream << "<table>\n<tbody>\n";

      stream << "\n<tr>\n<td><b>Image</b></td>\n<td><b>RMS</b></td>\n";
      if (contains_3d_count) {
        stream << "<td><b>Observed 3D Points</b></td>\n";
      }
      stream << "</tr>\n";

      for (const auto& stat : stats) {
        stream << "\n<tr>\n<td>" << stat.name << "</td>\n<td>" << stat.rms << "</td>\n";
        // optionally add 3d point count if it exists
        if (contains_3d_count) {
          stream << "<td>" << stat.point_3d_count << "</td>\n";
        }
        stream << "</tr>\n";
      }

      stream << "\n</tbody>\n</table>\n";
    }
  }
}

namespace calibmar {
  UndistortionEstimationResultWidget::UndistortionEstimationResultWidget(colmap::Camera& camera,
                                                                         std::unique_ptr<Pixmap> original_image, 
                                                                         std::unique_ptr<Pixmap> undistorted_image,
                                                                         std::pair<double, double> distances,
                                                                         QWidget* parent) : QWidget(parent), 
                                                                                            original_image_(std::move(original_image)), 
                                                                                            undistorted_image_(std::move(undistorted_image)) 
  {
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    // Result report
    std::stringstream result_stream;
    GenerateResultHtml(result_stream, camera, distances);
    QTextDocument* doc = new QTextDocument(this);
    doc->setDefaultStyleSheet(QString::fromStdString(cssStyle));
    QString text = QString::fromStdString(result_stream.str());
    result_text_ = new QTextEdit(this);
    result_text_->setDocument(doc);
    result_text_->setHtml(text);
    result_text_->setReadOnly(true);
    result_text_->setFrameStyle(QFrame::NoFrame);
    layout->addWidget(result_text_);
    layout->addStretch();

    int target_height = 500;

    cv::Mat cv_undistorted_image = undistorted_image_->Data().clone();
    cv::Mat cv_original_image = original_image_->Data().clone();

    if (cv_original_image.channels() > 1)
    {
      cv::cvtColor(cv_original_image, cv_original_image, cv::COLOR_BGR2GRAY);
      cv::cvtColor(cv_undistorted_image, cv_undistorted_image, cv::COLOR_BGR2GRAY);
    }

    cv::resize(cv_original_image, cv_original_image, cv_undistorted_image.size());
    original_image_->Assign(cv_original_image.clone());

    cv::Mat diff;
    cv::absdiff(cv_original_image, cv_undistorted_image, diff);
    cv::normalize(diff, diff, 0, 255, cv::NORM_MINMAX);
    cv::applyColorMap(diff, diff, cv::COLORMAP_JET);
    
    ZoomableScrollArea* original_area = new ZoomableScrollArea(this);
    original_area->setFrameShape(QFrame::Shape::NoFrame);
    ImageWidget* original_image_widget = new ImageWidget(this);
    original_area->setWidget(original_image_widget);
    original_area->widget()->resize(QSize(camera.width, camera.height)
                                   .scaled(camera.width, target_height, Qt::AspectRatioMode::KeepAspectRatio));
    original_image_widget->SetImage(std::move(original_image_));
    CollapsibleWidget* original_collapse = new CollapsibleWidget("Original", nullptr, this);
    original_collapse->SetWidget(original_area, target_height);
    layout->addWidget(original_collapse);

    ZoomableScrollArea* undistorted_area = new ZoomableScrollArea(this);
    undistorted_area->setFrameShape(QFrame::Shape::NoFrame);
    ImageWidget* undistorted_image_widget = new ImageWidget(this);
    undistorted_area->setWidget(undistorted_image_widget);
    undistorted_area->widget()->resize(QSize(camera.width, camera.height)
                                   .scaled(camera.width, target_height, Qt::AspectRatioMode::KeepAspectRatio));
    undistorted_image_widget->SetImage(std::move(undistorted_image_));
    CollapsibleWidget* undistorted_collapse = new CollapsibleWidget("Undistorted", nullptr, this);
    undistorted_collapse->SetWidget(undistorted_area, target_height);
    layout->addWidget(undistorted_collapse);


    std::unique_ptr<Pixmap> diff_pixmap = std::make_unique<Pixmap>();
    diff_pixmap->Assign(diff);
    ZoomableScrollArea* diff_area = new ZoomableScrollArea(this);
    diff_area->setFrameShape(QFrame::Shape::NoFrame);
    ImageWidget* diff_image = new ImageWidget(this);
    diff_area->setWidget(diff_image);
    diff_area->widget()->resize(QSize(camera.width, camera.height)
                                   .scaled(camera.width, target_height, Qt::AspectRatioMode::KeepAspectRatio));
    diff_image->SetImage(std::move(diff_pixmap));
    CollapsibleWidget* diff_collapse = new CollapsibleWidget("Difference", nullptr, this);
    diff_collapse->SetWidget(diff_area, target_height);
    layout->addWidget(diff_collapse);

  }

  UndistortionEstimationResultWidget::UndistortionEstimationResultWidget(const std::string& message, QWidget* parent) : QWidget(parent) {
    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    QString text = QString::fromStdString(message);
    result_text_ = new QTextEdit(this);
    result_text_->setWordWrapMode(QTextOption::NoWrap);
    result_text_->setFontFamily("Courier New");
    result_text_->setPlainText(text);
    result_text_->setReadOnly(true);
    result_text_->setFrameStyle(QFrame::NoFrame);
    layout->addWidget(result_text_);
  }

  void UndistortionEstimationResultWidget::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);

    result_text_->setFixedHeight(result_text_->document()->size().height() + 20);
  }
}