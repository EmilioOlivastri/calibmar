#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/filesystem.hpp>

#include "ui/dialogs/file_calibration_dialog.h"
#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/calibrators/calibrator3D.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/extractors/aruco_sift_extractor.h"
#include "calibmar/extractors/sift_extractor.h"
#include "calibmar/readers/filesystem_reader.h"

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

using namespace calibmar;


void printProgress(double percentage) 
{
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

// Writes the calibration result as a YAML matching the flat-port input format.
// FlatPort refrac_params layout: [Nx, Ny, Nz, int_dist, int_thick, na, ng, nw]
void createD0EstimationYaml(const std::string& path, const Calibration& calibration)
{
    const colmap::Camera& camera = calibration.Camera();
    const std::vector<double>& rp = camera.refrac_params;
    const Eigen::Matrix3d K = camera.CalibrationMatrix();
    const std::vector<double>& params = camera.params;

    std::ofstream out(path);

    out << std::fixed << std::setprecision(2);
    out << "# Physical parameters (millimiters)\n";
    out << "glass_thickness: " << rp[4] * 1000 << "\n";
    out << "normal_vector: [0.0, 0.0, 1.0]\n";
    out << "distance_to_glass: " << rp[3] * 1000 << "\n";
    out << "\n";

    out << std::fixed << std::setprecision(3);
    out << "# Refraction parameters\n";
    out << "glass_refractive_index: " << rp[6] << "\n";
    out << "water_refractive_index: " << rp[7] << "\n";
    out << "\n";

    out << std::fixed << std::setprecision(2);
    out << "# Camera parameters\n";
    out << "K_matrix: [";
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            out << K(r, c);
            if (r < 2 || c < 2) out << ", ";
        }
    out << "]\n";

    out << std::fixed << std::setprecision(6);
    out << "D_matrix: [";
    for (size_t i = 4; i < params.size(); i++) {
        out << params[i];
        if (i < params.size() - 1) out << ", ";
    }
    out << "]\n";

    out << "# Optimization parameters (millimiters)\n";
    out << "initial_guess: 0.1\n";
    out << "verbose: true\n";

    out.close();
}

void ImportParameters(const std::string& path, FileCalibrationDialog::Options& options)
{
    
    
    ImportedParameters p = ImportedParameters::ImportFromYaml(path);
    options.calibration_target_options = ArucoBoardFeatureExtractor::Options{p.aruco_type, p.grid_origin, ArucoGridDirection::Horizontal,
                                                                             p.columns, p.rows, p.square_size, p.spacing, p.border_bits};
    options.camera_model = p.camera_model;
    options.housing_calibration = {p.housing_model.value(), p.housing_parameters};
    options.initial_camera_parameters = p.camera_parameters;

    return;
  }

void setupCalibration(ArucoBoardFeatureExtractor::Options& aruco_options,
                      const FileCalibrationDialog::Options& options, 
                      Calibration& calibration, 
                      std::pair<int, int> image_size,
                      std::unique_ptr<FeatureExtractor>& extractor,
                      std::unique_ptr<Calibrator>& calibrator) 
{

    std::unique_ptr<ArucoBoardFeatureExtractor> aurco_extractor = std::make_unique<ArucoBoardFeatureExtractor>(aruco_options);
    calibration.SetPoints3D(aurco_extractor->Points3D());
    calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(aruco_options));
    extractor = std::move(aurco_extractor);
 
    HousingCalibrator::Options calibrator_options;
    calibrator_options.camera_model = options.camera_model;
    calibrator_options.image_size = image_size;
    calibrator_options.estimate_initial_dome_offset = options.housing_calibration.value().first == HousingInterfaceType::DoubleLayerSphericalRefractive;
    calibrator_options.housing_interface = options.housing_calibration.value().first;
    calibrator_options.camera_params = options.initial_camera_parameters.value();
    calibrator_options.initial_housing_params = options.housing_calibration.value().second;
    calibrator = std::make_unique<HousingCalibrator>(calibrator_options);

    return;
}

int main(int argc, char** argv) 
{

    if (argc < 3) 
    {
        std::cerr << "Usage: " << argv[0] << " <calibration.yaml> <path 2 image directory>" << std::endl;
        return 1;
    }

    FileCalibrationDialog::Options options;
    options.images_directory = argv[2];
    ImportParameters(argv[1], options);

    FilesystemImageReader::Options reader_options;
    reader_options.image_directory = options.images_directory;
    FilesystemImageReader reader(reader_options);

    std::pair<int, int> image_size;
    try 
    {
      image_size = {reader.ImagesWidth(), reader.ImagesHeight()};
    }
    catch (std::exception& ex) 
    { 
        std::cerr << "Error reading images: " << ex.what() << std::endl;
      return false;
    }

    std::unique_ptr<FeatureExtractor> extractor;
    std::unique_ptr<Calibrator> calibrator;
    Calibration calibration;
    ArucoBoardFeatureExtractor::Options target_options = std::get<ArucoBoardFeatureExtractor::Options>(options.calibration_target_options);
    setupCalibration(target_options, options, calibration, image_size, extractor, calibrator);

    int tot_images = reader.ImagesCount();
    int current_image = 0;
    try 
    {
      while (reader.HasNext()) 
      {
        printProgress((double)(current_image++) / (double)tot_images);
        Image image;
        std::unique_ptr<Pixmap> pixmap = std::make_unique<Pixmap>();
        ImageReader::Status reader_status = reader.Next(image, *pixmap);
        
        if (reader_status != ImageReader::Status::SUCCESS) continue;

        FeatureExtractor::Status extractor_status;
        extractor_status = extractor->Extract(image, *pixmap);

        if (extractor_status != FeatureExtractor::Status::SUCCESS) continue; 
            
        size_t id = calibration.AddImage(image);
      }
      calibrator->Calibrate(calibration);
    }
    catch (std::exception& ex) 
    {
      std::string message(ex.what());
      return 1;
    }

    std::cout << "Calibration RMS: " << calibration.CalibrationRms() << std::endl;
    colmap::Camera& camera = calibration.Camera();
    std::cout << "Camera model: " << camera.ModelName() << std::endl;
    std::string out_yaml = std::string(argv[1]).substr(0, std::string(argv[1]).rfind('/')) + "/flatportd0.yaml";
    createD0EstimationYaml(out_yaml, calibration);
    std::cout << "Result written to: " << out_yaml << std::endl;

    return 0;
}
