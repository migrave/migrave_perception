# ROS Wrapper for OpenPose Skeleton Tracking

This repository contains a ROS wrapper for skeleton tracking using [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose).

### Requirements
[Install OpenPose 1.7](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/0_index.md#compiling-and-running-openpose-from-source)
* Clone the repo
  ```
  git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
  cd openpose/
  git checkout v1.7.0
  git submodule update --init --recursive --remote
  ```
* Create build dir and configure cmake
  * With `cmake-gui` (you need cmake-gui for this `sudo apt install cmake-gui`)
    ```
    mkdir build
    cd build/

    # we will use sudo since we want to install openpose binaries to /opt/openpose
    cmake-gui ..
    ```
    * Press configure and it will pop up CMakeSetup window and then select Unix Makefiles and default native compilers. It will show avalaible options such as GPU_MODE, and supported libraries
      * If there is no GPU, select `CPU_ONLY` in `GPU_MODE`, and then press `Configure` again.
        If `CPU_ONLY` is chosen, select `DOWNLOAD_BODY_COCO_MODEL` as well since it runs faster on CPU, see this [optimization page](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/06_maximizing_openpose_speed.md) for more details.
      * `GPU_MODE` with CUDA
        * Install [cuda libaries](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) and [cudnn](https://developer.nvidia.com/rdp/cudnn-download)
    * Once it's done, press `Generate`, and then close cmake-gui

  * With cmake
    * Install with CUDA and CUDNN supports
      ```
      cmake .. -DGPU_MODE=CUDA -DUSE_CUDNN=ON
      ```

    * Install with CPU_ONLY
      ```
      cmake .. -DGPU_MODE=CPU_ONLY
      ```

    If you have [a runtime error](https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1527) with CUDNN ON, you can disable it `-DUSE_CUDNN=OFF`.

* Compile and install OpenPose
  ```
  make -j`nproc`

  # install
  sudo make install
  ```

* Once it finishes compiling the codes, move models directory to `/opt/openpose`
  ```
  # from openpose root dir
  # create openpose directory
  sudo mkdir /opt/openpose

  # move caffe and models dir from openpose root dir to /opt/openpose
  sudo mv models/ /opt/openpose
  ```

* Test if OpenPose runs properly
  ```
  # go to OpenPose root dir
  ./build/examples/openpose/openpose.bin --camera 0 -net_resolution -1x96
  ```

  Adjust `-net_resolution` if you have OOM problems.

### Usage
* Launch OpenPose ROS wrapper
  ```
  roslaunch migrave_skeleton_tracking skeleton_tracking.launch
  ```
* Start tracking
  ```
  rostopic pub /migrave_perception/openpose_ros/event_in std_msgs/String e_start
  ```
* Outputs
  ```
  /migrave_perception/openpose_ros/debug_image
  /migrave_perception/openpose_ros/skeleton_visualization
  /migrave_perception/openpose_ros/skeletons
  ```
* Stop tracking
  ```
  rostopic pub /migrave_perception/openpose_ros/event_in std_msgs/String e_stop
  ```

Continuous tracking can be disabled in the launch file.