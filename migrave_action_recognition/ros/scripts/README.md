This directory contains scripts for launching the CAL server as well as demos for both the learning and recognition functions.

### Launching CAL Server

When launching the CAL Server, the parameters of the script and usage description are as follows:
```
usage: rosrun migrave_action_recogntion continual_action_server [--model-type MODEL_TYPE
                                                   --nuitrack-skeleton-topic TOPIC_NAME
                                                   --seq-size SEQ_SIZE
                                                   --num-seq NUM_SEQ
                                                   --max-recovery-attempts NUM_ATTEMPTS]

The arguments are described below:
  --model-type -m
              Model type to use (default '1')
  --nuitrack-skeleton-topic -t
              Name of a logged skeleton topic (default /qt_nuitrack_app/skeletons)
  --seq-size -l
              Length of sequences for classifying and learning (default 50)
  --num-seq -n
              Number of sequences to be captured (default 25)
  --max-recovery-attempts -r
              Number of attempts to recover server (default 1)
  
```

An example call is given below:
```
rosrun migrave_action_recogntion continual_action_server -m 1 \
                                            -t /qt_nuitrack_app/skeletons
                                            -l 50
                                            -n 25
                                            -r 1
```

### Running Recognition Demo

When launching the Recognition demo, the parameters of the script and usage description are as follows:
```
usage: rosrun migrave_action_recogntion recognize_action_demo [--save_path SAVE_PATH
                                                   --participant-id PARTICIPANT_ID
                                                   --model-type MODEL_TYPE
                                                   --nuitrack-skeleton-topic TOPIC_NAME
                                                   --seq-size SEQ_SIZE]

The arguments are described below:
  --save-path -s
              Path to directory where recognition data to be saved (default /home/qtrobot)
  --participant-id -i
              ID of the participant (default 0)
  --model-type -m
              Model type to use (default '1')
  --nuitrack-skeleton-topic -t
              Name of a logged skeleton topic (default /qt_nuitrack_app/skeletons)
  --seq-size -l
              Length of sequences for classifying and learning (default 50)
  
```

An example call is given below:
```
rosrun migrave_action_recogntion recognize_action_demo -s /path/to/save_dir \
                                            -i 0 \
                                            -m 1 \
                                            -t /qt_nuitrack_app/skeletons
                                            -l 50
```

### Running Learn Action Demo

When launching the Learning demo, the parameters of the script and usage description are as follows:
```
usage: rosrun migrave_action_recogntion learn_action_demo [--save_path SAVE_PATH
                                                   --participant-id PARTICIPANT_ID
                                                   --nuitrack-skeleton-topic TOPIC_NAME
                                                   --seq-size SEQ_SIZE
                                                   --num-seq NUM_SEQ]

The arguments are described below:
  --save-path -s
              Path to directory where learning data to be saved (default /home/qtrobot)
  --participant-id -i
              ID of the participant (default 0)
  --nuitrack-skeleton-topic -t
              Name of a logged skeleton topic (default /qt_nuitrack_app/skeletons)
  --seq-size -l
              Length of sequences for classifying and learning (default 50)
  --num-seq -n
              Number of sequences to be captured (default 25)
  
```

An example call is given below:
```
rosrun migrave_action_recogntion recognize_action_demo -s /path/to/save_dir \
                                            -i 0 \
                                            -t /qt_nuitrack_app/skeletons
                                            -l 50
                                            -n 25
```

### Running Learn Action Client

When launching the Learning client, the parameters of the script and usage description are as follows:
```
usage: rosrun migrave_action_recogntion learn_action_client [--num_actions NUM_ACTIONS
                                                   --action-names ACTIONs_LIST
                                                   --task-id TASK_ID]

The arguments are described below:
  --num_actions -n
              Number of actions to be learnt (default 4)
  --action-names -a
              List of action names to be learnt (default ["Talking on Phone", "Cutting Food", "Hand Waving", "Hopping"])
  --task-id -i
              ID of task id for new actions to be added to (default -1)
  
```

An example call is given below:
```
rosrun migrave_action_recognition learn_action_client -n 2 \
                                            -a "Talking on Phone" "Cutting Food" \
                                            -i -1 
```
