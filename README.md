# ROS Projects at Politecnico di Milano - course of Robotics [A.A. 2024/2025]

## Second Project

In `robotics/catkin_ws/src/second_project/` you can find the implementation of a mapping and navigation pipeline for a mobile robot using ROS.

![Robot of the second project](robot_project2.png)

### Project Structure

- **Task 1: Mapping**
  - Use two 2D LiDAR scans and robot odometry from a ROS bag to reconstruct an environment map
  - Data topics: `/scan_back`, `/scan_front`, `/odometry`, `/tf`, `/tf_static`
  - Merge the two lasers for 360° coverage and **filter out points belonging to the robot**
  - Export the map as a `.png` image and a `.yaml` file.

- **Task 2: Navigation**
  - Simulate the robot in Stage using the generated map.
  - Setup navigation stack to localize and plan using the static map.
  - Drive to goals loaded from a CSV file, published via a custom node using ROS actions.
  - Visualize in RViz: robot, map, TFs, particle cloud (AMCL), paths, goals.

### How to Run

1. **Mapping**
    - Edit and use the launch files in `launch/`:

      ```bash
      roslaunch second_project mapping.launch
      ```

    - Play the bag file:

      ```bash
      rosbag play --clock data/robotics2.bag
      ```

    - Save the map using [map_server](http://wiki.ros.org/map_server):

      ```bash
      rosrun map_server map_saver -f ./catkin_ws/src/second_project/map/map
      ```

      ![Mapping of the second project](mapping_project2.png)

2. **Navigation**
    - Use the included launch files in `launch/`.
    - The map from Task 1 must be placed in the `map/` folder.
    - Run with simulated time enabled.
    - Goals are read automatically from `csv/goals.csv`.

3. **Visualization**
    - On Windows, connect to [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html)
    - RViz configuration files are provided in `cfg/`.
    - View the map, robot, sensors, and navigation status.

### Requirements

- ROS Noetic
- Mapping package: gmapping
- Stage simulator
- Additional ROS packages: `map_server`, `move_base`, `amcl`

### Provided Folders

- `mapping/` – Source and launch files for Task 1.
- `navigation/` – Source and launch files for Task 2.
- `map/` – Output map files.
- `csv/` – File with navigation goals.
- `cfg/` – RViz configs.

## Authors

- Matteo Pompilio
- Piero Burigana
- Merve Rana Kizil
