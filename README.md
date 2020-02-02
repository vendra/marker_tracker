# Marker Tracker

A ROS based multicamera system to track in realtime an IR marker in 3D space using Kinect One Sensors with data fusion and synchronization. 

### Marker and Visualization 
![The Marker](https://raw.githubusercontent.com/vendra/marker_tracker/local/cube.png)
![Detection Rviz](https://raw.githubusercontent.com/vendra/marker_tracker/local/view1.png)

##  Instructions
-Remember to change the file path inside the launcher to the correct .bag file  
  
-The bag must contain 2 topics for each Kinect: /kinect_0x/ir/image and /kinect_0x/depth/CompressedDepth  
  
-Use one of the following:   
```
$ roslaunch marker_tracker marker.launch #Standard  
$ roslaunch marker_tracker slow_marker.launch #Slow rate bag play  
$ roslaunch marker_tracker noview_marker.launch #No visualization  
$ roslaunch marker_tracker test.launch #Only launch detector, need bag manually and republish nodes  
```
-The launch the master:  
```
$ rosrun marker_tracker master_node  
```
use Rviz to visualize results  
  
==================  
All the code has been developed and tested on Ubuntu 16.04, OpenCV3, ROS Kinetic.  
It was partially tested on Ubuntu 16.04, OpenCV2 and ROS Indigo.  
  
### Italian
  
-Potrebbe essere necessario cambiare il path della .bag nel launcher.  
La bag deve contenere 2 topic per Kinect, /kinect_0x/ir/image e /kinect_0x/depth/CompressedDepth  
  
-Utilizzare uno dei seguenti:   
```
$ roslaunch marker_tracker marker.launch #Standard  
$ roslaunch marker_tracker slow_marker.launch #Slow rate bag play   
$ roslaunch marker_tracker noview_marker.launch #No visualization  
$ roslaunch marker_tracker test.launch #Only launch detector, need bag manually and republish nodes  
```
-Poi avviare il master:  
```
$ rosrun marker_tracker master_node  
```

Utilizzare Rviz per visualizzare i risultati  

### Details
![Detection](https://raw.githubusercontent.com/vendra/marker_tracker/local/pipeline.png)
![View Kalman Filter](https://raw.githubusercontent.com/vendra/marker_tracker/local/view2.png)

### ROS Graph
![ROS Graph](https://raw.githubusercontent.com/vendra/marker_tracker/local/graph.png)
