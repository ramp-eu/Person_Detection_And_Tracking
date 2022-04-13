# User & Programmers Manual

This module is integrated in OPIL, so the people detected will be added to the virtual representation of the environment through the FIROS bridge. 

To send the information about the position of the operators we are using the `Local Sensing and Perception` module. We have done this because the type of message we need in our application has already been defined and implemented in the `Central Sensing and Perception` and the `Local Sensing and Perception`. 

We are using the same type of messages that are used to indicate the position of the robots within the map (check the definition [here](https://github.com/ramp-eu/Local_Sensing_and_Perception/blob/4fb4894a63e3a4bb5ec3712fad6c646a3013de3a/src/mapupdates/msg/NewObstacles.msg)). The name of the new topic is `/worker/newObstacles`.

The weights of the `YOLO` neural network can be found [here](https://pjreddie.com/media/files/yolov3.weights).