#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/filesystem.hpp>


int main(int argc, char** argv) 
{
    if (argc < 4) 
    {
        std::cerr << "Usage: " << argv[0] << " <undistortion_map.yaml> <path 2 image folder> <output folder>" << std::endl;
        return 1;
    }

    std::string undistortion_parameters = argv[1];
    std::string image_folder = argv[2];
    std::string output_folder = argv[3];

    std::vector<cv::String> images;
    cv::String pattern = "*.png";
    cv::utils::fs::glob(image_folder, pattern, images);

    cv::FileStorage fs(undistortion_parameters, cv::FileStorage::READ);
    cv::Mat kMat, roi, undist_map;
    fs["Camera Matrix"] >> kMat; 
    fs["ROI"] >> roi;
    fs["Undistortion Map"] >> undist_map;

    int num_images = images.size();
    std::cout << "Number of images: " << num_images << std::endl;
    std::cout << "Camera Matrix: " << kMat << std::endl;
    std::cout << "ROI: " << roi << std::endl;

    cv::Range cr(roi.at<int>(0), roi.at<int>(2));
    cv::Range rr(roi.at<int>(1), roi.at<int>(3));

    for (int i = 0; i < num_images; i++) 
    {
        std::string img_path = images[i];
        std::string img_name = img_path.substr(img_path.find_last_of("/\\") + 1);
        cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
        cv::Mat dst_img = cv::Mat::zeros(img.rows, img.cols, img.type());
        cv::remap(img, dst_img, undist_map, cv::noArray(), cv::INTER_LINEAR, cv::BorderTypes::BORDER_CONSTANT);
        cv::Mat roi_img = dst_img(rr, cr).clone();

        cv::imwrite(output_folder + "/" + img_name, roi_img);
    }

    return 0;
}
