# migrave_perception_face_feature_detector

This package contains a ROS wrapper for face feature detector using [OpenFace](https://github.com/TadasBaltrusaitis/OpenFace).

## [Installation](https://github.com/TadasBaltrusaitis/OpenFace/wiki/Unix-Installation)

* Dependencies
  
  * Install GCC and openblast
  
    ```
    sudo apt-get update
    sudo apt-get install build-essential
    sudo apt-get install g++-8

    # Install openblas
    sudo apt-get install libopenblas-dev
    ```
  * Install OpenCV 4.1.0 (the version which comes with ROS noetic should also work)
  * Install [Dlib 19.22](http://dlib.net/files)
    ```
    wget http://dlib.net/files/dlib-19.22.tar.bz2;
    tar sxvf dlib-19.22.tar.bz2;
    cd dlib-19.22;
    mkdir build;
    cd build;
    cmake ..;
    cmake --build . --config Release;
    sudo make install;
    sudo ldconfig;
    cd ../..;    
    ```
* Install OpenFace

  * Clone OpenFace repo and checkout to `OpenFace_2.2.0`
    ```
    git clone https://github.com/TadasBaltrusaitis/OpenFace.git
    cd OpenFace

    # Checkout to OpenFace_2.2.0
    git checkout OpenFace_2.2.0

    mkdir build
    cd build
    ```
  * Built and install OpenFace (if using g++ version 8, change to clang or other version appropriately)
    ```
    cmake -D CMAKE_CXX_COMPILER=g++-8 -D CMAKE_C_COMPILER=gcc-8 -D CMAKE_BUILD_TYPE=RELEASE ..
    make

    # Install
    sudo make install
    ```
  * Download pretrained models
    * Download the models from [this link](https://github.com/TadasBaltrusaitis/OpenFace/wiki/Model-download)
    * Move them the `model/patch_experts`
      ```
      # Create patch_experts dir
      sudo mkdir /usr/local/etc/OpenFace/model/patch_experts/

      # Move all models to patch_experts dir
      sudo mv cen_patches_0.* /usr/local/etc/OpenFace/model/patch_experts/
      ```
* Test OpenFace installation
  ```
  # Go to the OpenFace root directory
  FaceLandmarkVidMulti -f samples/multi_face.avi
  ```

## Usage
* Start tracking
  ```
  rostopic pub /migrave_perception/openface_ros/event_in std_msgs/String e_start
  ```
* Stop tracking
  ```
  rostopic pub /migrave_perception/openface_ros/event_in std_msgs/String e_stop
  ```
* Outputs
  ```
  # Face features
  /migrave_perception/openface_ros/faces

  # Debug image
  /migrave_perception/openface_ros/debug_image

  ```
