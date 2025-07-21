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

Configure and compile Calibmar:

	cd path/to/calibmar
    mkdir build
    cd build
    cmake .. -GNinja
    ninja



# Features added by me:
- Undistortion for both dome and flat port model;
- Apriltag detection using the more robust Kalibr library;
- Given a video, produce a set of images for the calibration (for GoPro mainly);
- Stream edge detection for better mechanical alignment of camera in dome;

### How to use the undistortion code
The code is in the folder app and is named "undistortion_app.cpp". It requires as input the calibration file as a yaml, which contains the calibraiton in air, the parameters of the dome/flatport (obtain with calibration in water), and, optionally, the distance at which reproject the image (typically done at the working distance, but can be set-up experimentally).

```
./build/src/app/undistort calibration.yaml path_to_image.png 5.0
```
The computation of the mapping just needs to be done once, and then the remapping is real-time using the opencv functions. 


### How to use the video to images
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
### How to use the edge detection code
The code allows for easy streaming of the original images or its edges. It both can used classic Canny Edge Detection or Deep Learning based Opencv edge detection.

```
usage: edgedetection.py [-h] [--video VIDEO] [--factor FACTOR] [--vis_edge VIS_EDGE] [--vis_deep VIS_DEEP]

Convert video to images at desired frame rate

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