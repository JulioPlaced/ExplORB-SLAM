ExplORB-SLAM
============

A package for Active visual SLAM using the structure of the underlying pose-graph.

Code used for the paper "ExplORB-SLAM: Active Visual SLAM Exploiting the Pose-graph Topology", accepted for presentation in the Fifth Iberian Robotics Conference (ROBOT 2022).

Tested by jplaced for Ubuntu 20.04, ROS Noetic.

Contact: jplaced@unizar.es, jjgomez@unizar.es

Citation
------------

Placed, J. A., Gómez-Rodríguez, J. J., Tardós, J. D., & Castellanos, J. A. (2022). ExplORB-SLAM: Active Visual SLAM Exploiting the Pose-graph Topology. In 2022 Fifth Iberian Robotics Conference (ROBOT).

Detected dependencies:
------------
- Eigen
- OpenCV
- Python3
  * Numpy
  * Sklearn
  * Numba
  * OpenCV
- Gazebo
- ROS Noetic
  * rviz
  * turtlebot3_teleop
  * gazebo_ros
  * octomap_ros
  * octomap_rviz_plugins
  * move_base

Building
------------
1. Clone repo:
```
git clone https://github.com/JulioPlaced/ExplORBSLAM.git
```

2. Build repo:
```
cd ExplORBSLAM/
catkin b
```

3. Remember to source the ExplORBSLAM workspace:

  ```
  source devel/setup.bash
  ```

  If sourcing doesn't work properly, try

  ```
  catkin config --no-install
  catkin clean --all
  ```

  and rebuild.

Running
------------
1. Launch the scenario:

  AWS house environment:
  ```
  roslaunch robot_description single_house.launch
  ```
  Or AWS bookstore environment:
  ```
  roslaunch robot_description single_bookstore.launch
  ```

2. Launch the decision maker
  ```
  roslaunch decision_maker autonomous_agent.launch
  ```
