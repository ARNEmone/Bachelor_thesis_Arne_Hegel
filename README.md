# Sampling-based Path Planning for Human Steering Evaluation

## Table of Contents
1. [General Info](#general-info)
2. [Introduction](#introduction)
3. [Technologies](#technologies)

## General Info
This bachelor thesis addresses the topic Sampling-based Path Planning for Human Steering Evaluation. The bachelor thesis itself with all used references can be found in the folder "Report". In the following a short introduction to the implementations is given.

## Introduction

### Add new Map
To use on a new map, a .pgm file of the map and a .yaml file with the associated "origin" and "resolution" values must be created. These must be saved in a new folder in directory "worlds". It should be noted that the .pgm and .yaml file are named like the new folder.
Example: worlds/world1/world1.pgm worlds/world1/world1.yaml

### Graph generation
A new graph must be generated on this map, which enables you to find paths on it later. This is doable with the Python script scripts/create_graph.py. Parameters to be selected are located in the script in the area "launch main". Further adjustments are also possible in the "Global Parameters" section. The generated graph is stored in worlds/graph.obj.

### Performing and evaluating a drive in Gazebo
To perform a drive in Gazebo and evaluate it, use the launch/start.launch script. Here the arguments start position, goal positions, world and saved_graph can be choosen. If the argument saved_graph=true is chosen, the graph of the corresponding map is loaded from its directory. The name of the file must be structured as follows: graph_<name of map>.obj . For example worlds/world1/graph_world1.obj . If the argument saved_graph=false is chosen, the last generated graph from worlds/graph.obj is loaded. At the end of the drive, the rosbag is saved in the rosbag_analysis/rosbags/record.bag file. It should be noted that the goal positions are entered as followed: x1,y1,phi1;y2,y2,phi2;...;xn,yn,phin

### Evaluation of a recorded drive
To evaluate a recorded ride, a rosbag with the topic /odom is required. This must be stored in the directory of the corresponding map and be named rosbag.bag. For example worlds/world1/rosbag.bag . Again the arguments start position, goal positions, world and saved_graph can be choosen. If the argument saved_graph=true is chosen, the graph of the corresponding map is loaded from its directory. The name of the file must be structured as follows: graph_<name of map>.obj . For example worlds/world1/graph_world1.obj . If the argument saved_graph=false is chosen, the last generated graph from worlds/graph.obj is loaded. Then the script launch/analyse_rosbag.launch can be used to evaluate the drive. Afterwards, the rosbag with the ratings is saved again in the file rosbag_analysis/rosbags/record.bag. 

### Parameters of the evaluation
The evaluation is done via the script scripts/analyze_drive.py. The parameters can be changed in the script in the "Global Parameters" section.

### Evaluation of the rosbag record.bag
To look at the evaluations in the rosbag record.bag, the rosbag_analysis/analyse_rosbag.py script can be used. In the script in the "Global Parameters" area, the map and the rosbag to be analyzed can be specified. If the parameter world=None is set, no map is needed and used.

## Technologies
A list of technologies used within the project:
* [Technologie name](https://example.com): Version 12.3 
* [Technologie name](https://example.com): Version 2.34
* [Library name](https://example.com): Version 1234