# migrave_perception_face_feature_detector

Face feature detector using [OpenFace](https://github.com/TadasBaltrusaitis/OpenFace).

## Dependencies
* [OpenFace_2.2.0](https://github.com/TadasBaltrusaitis/OpenFace/tree/OpenFace_2.2.0)
* OpenCV 4.2 (the one that comes with ros noetic should be fine)
* [Dlib 19.22](http://dlib.net/files)

Install the dependencies as described on the OpenFace [wiki](https://github.com/TadasBaltrusaitis/OpenFace/wiki/Unix-Installation).

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