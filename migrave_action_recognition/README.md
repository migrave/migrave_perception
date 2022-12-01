# MigrAVE Continual Action Learning (CAL) Module

Repository for the Continual Action Learning (CAL) module used in the MigrAVE project.

## Package dependencies

* FTSM: https://github.com/migrave/ftsm
* QT Nuitrack App: https://github.com/luxai-qtrobot/software/tree/master/headers/qt_nuitrack_app
* MigrAVE Skeleton Tools: https://github.com/migrave/migrave_skeleton_tools
* CTR-GCN: https://github.com/migrave/CTR-GCN
* FACIL: https://github.com/migrave/FACIL

## Usage
* Launch the node:
  
  ```
  rosrun migrave_action_recogntion continual_action_server
  ```

* Subscribe to ROS topic to receive action class label:

  ```
  rostopic echo /continual_action_server/feedback
  ```

* Publish execution request: 

  ```
  rostopic pub /continual_action_server/goal migrave_action_recognition/ContinualActionLearningActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
  goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
  goal:
  request_type: ''
  num_actions: 0
  action_names:
  - ''
  task_id: 0"
  ```
  
  The main fields to fill is:
  * request_type: possible options - classify, learn, stop
  * num_actions: number of actions to be learnt
  * action_names: name of the actions to be learnt
  * task_id: task id to modify task with new action, input -1 to train new task
  
  Example request
  ```
  rostopic pub /continual_action_learning/goal migrave_action_recognition/ContinualActionLearningActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
  goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
  goal:
  request_type: 'learn'
  num_actions: 1
  action_names:
  - 'talking on phone'
  task_id: -1" -1
  ```
  
