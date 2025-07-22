> **NOTE**: This repository is hosted on [Gitlab](https://cau-git.rz.uni-kiel.de/inf-ag-koeser/calibmar) and mirrored to [Github](https://github.com/MDSKiel/calibmar). Please use Github for [issues](https://github.com/MDSKiel/calibmar/issues) and get the [releases](https://cau-git.rz.uni-kiel.de/inf-ag-koeser/calibmar/-/releases) on Gitlab.

# Calibmar

Calibmar is a camera and underwater housing calibration tool.

Features include:
- Camera housing calibration based on '[Refractive Geometry for Underwater Domes](https://doi.org/10.1016/j.isprsjprs.2021.11.006)'
	- with a Flat Port model from '[Refractive Calibration of Underwater Cameras](https://doi.org/10.1007/978-3-642-33715-4_61)'
- Camera and stereo camera calibration 
- Calibration guidance implementation of '[Calibration Wizard](https://doi.org/10.1109/iccv.2019.00158)'
- [COLMAP](https://colmap.github.io/) compliant camera models

## Install

Binaries for Windows and Linux are available at https://cau-git.rz.uni-kiel.de/inf-ag-koeser/calibmar/-/releases.

## Build from Source

### CUDA

To build with CUDA support install the latest CUDA from NVIDIA's homepage.

You will also need to specify a CUDA architecture during cmake configuration as e.g. `-DCMAKE_CUDA_ARCHITECTURES=native`.

There is a known error for Windows, when installing CUDA together with Visual Studio Build Tools (as opposed to Visual Studio IDE), where CUDA does not properly integrate into the build tools and some files have to be manually copied.

### Linux

The following build has been tested under Ubuntu 22.04.

Dependencies from default Ubuntu repositories:

    sudo apt-get install \
        git \
        cmake \
        build-essential \
        libboost-program-options-dev \
        libboost-filesystem-dev \
        libboost-graph-dev \
        libboost-system-dev \
        libboost-test-dev \
        libeigen3-dev \
        libflann-dev \
        libfreeimage-dev \
        libmetis-dev \
        libgoogle-glog-dev \
        libglew-dev \
        libsqlite3-dev \
        qtbase5-dev \
        libqt5opengl5-dev \
        libcgal-dev \
        libceres-dev \
        libopencv-dev

Clone the calibmar repository and cd into it:
```
git clone https://github.com/EmilioOlivastri/calibmar.git
cd path/to/calibmar
```
Then pull changes that could have been made on their colmap repo to have it always up-to-date.
```
git subtree pull --prefix lib/colmap https://cau-git.rz.uni-kiel.de/inf-ag-koeser/colmap_underwater.git underwater --squash
```
Finally build the calibmar framework
```
mkdir build && cd build
cmake .. -DCMAKE_CUDA_ARCHITECTURES=native \
-DCALIBMAR_TESTS_ENABLED=OFF
make -j4
```

# Features added by me:
- Undistortion map estimation for both dome and flat port models;
- Image undistortions using the specified remappings;
- Apriltag detection using the more robust Kalibr library;
- Given a video, produce a set of images for the calibration (for GoPro mainly);
- Stream edge detection for better mechanical alignment of camera in dome;

### How to use the undistortion estimation code
The code is in the folder app/undistortion and is named "undistortion_estimation.cpp".
To estimate the undistortion map the following files are needed as input:
- calibration_file : YAML file which contains the calibraiton of the camera performed in AIR, and the parameters of the of the dome/flat port obtained using the underwater calibration;
- input_image: Path to an image used for debugging (results of the remapping);
- reprojection_distance: Distance at which to put the 3D planar points to be reprojected on the camera, and used for computing the mapping;
- virtual_distance: Optional field used only in the case of flatport. Field that signifies the optimal distance to put the virtual camera from the real camera to minimize the length of the caustic.
```
./build/src/app/undistortion_estim calibration_file.yaml input_image reprojection_distance virtual_distance
```
The computation of the mapping just needs to be done once, and then the remapping is real-time using the opencv functions. 
In order to have the correct intrinsic parameters it is necessary to perform again the calibration (using the same underwater data). While the extrinsics, of course, don't change.

### How to use the undistortion visualizer
After estimating an undistortion map, the effect of the undistortion can be qualitatively evaluated using the visualizer, that compares the original image and the undistorted one. The visualizer code is in the folder app/undistortion and is called "undistort_visualizer.cpp".
To run this code the following inputs are needed:
- undistortion_map: YAML file containing the intrinsics of the camera, final ROI and undistortion map;
- images_folder: Path to the folder of images that you want to undistort;
```
./build/src/app/undistort_viz undistortion_map images_folder output_folder
```

### How to use the undistorter code
In the folder app/undistortion, the file "undistorter.cpp" performs the undistortion of a folder of images given as input using the map estimated at the previous step.
The following inputs are needed:
- undistortion_map: YAML file containing the intrinsics of the camera, final ROI and undistortion map;
- images_folder: Path to the folder of images that you want to undistort;
- output_folder: Output folder in which to store the undistorted images;

```
./build/src/app/undistorter undistortion_map images_folder output_folder
```
This code also functions as an example of how the undistortion could be applied during online applications.  

### How to use the edge detection code
The reason as to why use edge detection is to be able to visually evaluate the misalignment along the optical axes of the camera with respect to the center of the dome. If the camera is perfectly (or almost) aligned with the dome, the underwater instrinsics will be the same as the in air intrinsics, and not true in any other case.
If it is perfectly aligned and you have a planar object (e.g. checkerboard), then if you put the checkerboard and the underwater camera half in the water and half out of the water, the upper part of the checkerboard should coincide with the submerged part in the image.
The code allows for easy streaming of the original images or its edges. It both can used classic Canny Edge Detection or Deep Learning based Opencv edge detection. The code is in the folder scripts.

```
usage: edgedetection.py [-h] [--video VIDEO] [--factor FACTOR] [--vis_edge VIS_EDGE] [--vis_deep VIS_DEEP]
optional arguments:
  -h, --help                show this help message and exit
  --video VIDEO             Path to video file
  --factor FACTOR           Resize factor for images
  --vis_edge VIS_EDGE(0/1)  Visualize the edge detection
  --vis_deep VIS_DEEP(0/1)  Visualize the deep edge detection
```
Here it is showcased an example of usage of the script
```
python scripts/edgedetection.py --video ../../Data/videos/gopro/GX019894.MP4 --factor 0.5 --vis_edge 1 --vis_deep 1
```

### How to use the video to images converter
It is a python script that allows you to extrapolate images from a video. The ouput is produced in a friendly format to Kalibr so a calibration bag can be easily created. In order to use it it is necessary to specify a series of parameters:
```
usage: vid2imgs.py [-h] [--video VIDEO] [--output OUTPUT] [--fps FPS] [--skip SKIP] [--max_frames MAX_FRAMES] [--vis VIS] [--factor FACTOR]
                   [--format FORMAT]

Convert video to images at desired frame rate

optional arguments:
  -h, --help            show this help message and exit
  --video VIDEO         Path to video file
  --output OUTPUT       Path to output directory
  --fps FPS             Frame rate of the video
  --skip SKIP           Number of frames to skip
  --max_frames MAX_FRAMES
                        Maximum number of frames to extract
  --vis VIS             Visualize the video
  --factor FACTOR       Resize factor for images
  --format FORMAT       Image format
```
And here is an example of the parameters used for the GoPro11
```
python scripts/vid2imgs.py --video ../../Data/videos/gopro/GX019896.MP4 --output ../../Data/imgs/200325/cam0/ --fps 50 --skip 30 --max_frames 2000
```

### Other utils
There are some other utils that you can use. In the scripts folder there is a script to resize the images in a folder to help the apriltag_detection, and another one to visualize and manually remove the undesired images from a folder. 
