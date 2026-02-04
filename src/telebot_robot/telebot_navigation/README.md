# telebot Robot - localization and navigation setup.

## Overview

This project involved setting up a complete **ROS 2 Nav2 navigation pipeline** for a mobile robot in simulation using **ROS 2 Humble**. The goal was to bring up localization, mapping, planning, control, and recovery behaviors, and successfully navigate the robot to user-defined goal poses using Nav2.

The work covered configuration, debugging, and integration of multiple Nav2 components, along with RViz visualization and action-based goal execution.

---


### 1. Map Server

* A previously generated 2D occupancy grid map (`.pgm` + `.yaml`) was loaded using the **Nav2 map_server**.
* The map server publishes the `/map` topic and provides the static environment required for localization.
* The map frame (`map`) acts as the global reference frame.

### 2. AMCL Node (Adaptive Monte Carlo Localization)

* AMCL was used to estimate the robot’s pose in the map frame using:

  * LaserScan data
  * Odometry from the differential drive controller
  * The static map from the map server

* AMCL parameters were configured via a YAML file, including:

  * `base_frame_id`: `base_footprint`
  * `odom_frame_id`: `odom`
  * `global_frame_id`: `map`
  * Laser model and motion model parameters

* AMCL publishes the **dynamic transform** `map → odom`, which corrects odometry drift over time.



### 3. Navigation

The following Nav2 components were successfully configured and brought to the **active** lifecycle state:

* `map_server`
* `amcl`
* `planner_server`
* `controller_server`
* `bt_navigator`
* `behavior_server`
* `global_costmap`
* `local_costmap`
* `velocity_smoother`
* `collision_monitor`

Lifecycle management was handled via:

* `nav2_lifecycle_manager` (localization and navigation)


###  Navigation Parameters

A consolidated `nav2_params.yaml` file was created to configure:

* Global and local costmaps
* Planner and controller plugins
* Behavior Tree (BT) plugins
* Velocity smoother
* Collision monitor

Note:
* Matching BT XML files with available plugins

---

### 4. Launch System


* The system was split into **modular launch files**:

  * A **bringup launch file** that starts the robot, simulation, and core nodes
  * A **map server launch file** to load the map_server
  * A **localization launch file** dedicated to AMCL
  * A custom `navigation.launch.py` file was written to, bring up Nav2 navigation nodes and manage lifecycle transitions automatically


---

### 5. RViz Visualization

RViz was configured to visualize:

* Static map
* Robot model and TF
* Laser scan
* Global and local costmaps

* Localization was verified visually by observing laser scans aligning with the map.

---

## How Localization Was Verified

1. The map was successfully visualized in RViz via `/map`.
2. Laser scans were confirmed to be correct in the robot frame.
3. AMCL was verified to be running by checking:

   * `/amcl_pose` topic
   * `map → odom` transform using `tf2_echo`
4. The robot’s estimated pose converged as it moved, confirming correct localization.

---
### 6. TF Tree Setup

The final TF tree used for localization was:

```
map
 └── odom        (published by AMCL)
      └── base_footprint  (published by diff_drive controller)
           └── base_link
                └── lidar_link
```

This TF structure is required for correct scan-to-map alignment.

---

### 7. Initial Pose Handling

The robot’s initial pose was critical for alignment between:

* Map
* Laser scan
* Global and local costmaps

 a **C++ node** was implemented to publish the initial pose.

---

### 8. Goal Execution

Navigation goals were sent using a **C++ action client** for:

* `nav2_msgs/action/NavigateToPose`


## Challenges Faced and Resolutions

### 1. Lifecycle Nodes Not Activating

**Problem:**
Some Nav2 nodes remained in the `unconfigured` or `inactive` state.

**Cause:**

* Missing or misconfigured parameters
* Incorrect lifecycle manager `node_names` list

**solution:**

* Verified lifecycle configuration
* Ensured all required parameters were present
* Restarted the system cleanly between launches



### 2. Missing Nav2 Recoveries Package Confusion

**Problem:**
Attempted installation of `ros-humble-nav2-recoveries` failed.

**Cause:**

* In ROS 2 Humble, recovery behaviors are included in `nav2_behaviors`

**solution:**

* Verified installed Nav2 packages
* Adjusted expectations and configuration accordingly

---


## Final Outcome

At the end of the project:

* All Nav2 nodes successfully reached the **active** state
* Localization and mapping were stable
* Costmaps updated dynamically and visualized correctly
* Robot navigated reliably to goal poses using a C++ action client
* The navigation pipeline was fully functional and robust


---





