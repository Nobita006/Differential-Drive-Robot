# 🤖 Autonomous ROS 2 Lawnmower Bot

A fully autonomous, differential-drive lawnmower simulation built on **ROS 2 Jazzy** and **Gazebo**. This project demonstrates a complete robotic pipeline including state-machine execution, SLAM mapping, Boustrophedon coverage planning, computer vision obstacle detection, and Nav2 autonomous navigation.

---

## 📖 Table of Contents
1. [What It Does](#what-it-does)
2. [Technical Capabilities & Tech Stack](#technical-capabilities--tech-stack)
3. [Sensors & Hardware](#sensors--hardware)
4. [Working Principle (The Pipeline)](#working-principle-the-pipeline)
5. [How to Run & Showcase](#how-to-run--showcase)
6. [Commands](#commands)
7. [Limitations & Future Improvements](#limitations--future-improvements)

---

## 🚀 What It Does
The Autonomous Lawnmower Bot is designed to be placed in an unknown yard and completely cut the grass without human intervention.
When started, the robot does **not** have a pre-loaded map. It autonomously drives around the perimeter of the yard using a "Boundary Discovery" routine, mapping the walls and obstacles. Once the perimeter is established, an internal planner breaks the lawn down into a grid and generates a highly efficient back-and-forth sweeping path (like a Roomba) to guarantee 100% grass coverage. It will actively dodge dynamic obstacles (toys, humans) and report its progress.

---

## 🛠️ Technical Capabilities & Tech Stack
- **Framework:** ROS 2 (Jazzy Jalisco)
- **Simulation:** Gazebo (DartSim physics engine)
- **Navigation:** Nav2 (Nav2 Simple Commander, Regulated Pure Pursuit Controller)
- **Mapping:** `slam_toolbox` (Online Asynchronous SLAM)
- **State Estimation:** `robot_localization` (Extended Kalman Filter / EKF)
- **Coverage Planning:** Custom Boustrophedon Cellular Decomposition Python implementation
- **Computer Vision:** OpenCV (Color thresholding and pinhole projection)

---

## 📡 Sensors & Hardware
The simulated differential-drive chassis is equipped with the following sensors:

1. **2D LiDAR (360-degree):**
   - *Function:* Shoots laser pulses to measure exact distances to solid objects (trees, walls, rocks).
   - *Usage:* Used by `slam_toolbox` to build the 2D occupancy grid map, and used by `Nav2` to avoid collisions locally.
2. **RGB Camera (Front-Facing):**
   - *Function:* Captures standard color video of the environment in front of the mower.
   - *Usage:* Processed by the `vision_processor.py` node to detect non-green objects (potential obstacles too low for LiDAR) and project them onto the ground plane.
3. **IMU (Inertial Measurement Unit):**
   - *Function:* Measures the robot's acceleration and rotational velocity.
   - *Usage:* Fused with the wheel odometry by the EKF to give the robot a highly accurate sense of what direction it is facing, even if the wheels slip on grass.
4. **Wheel Encoders (Odometry):**
   - *Function:* Counts how many times the left and right wheels have turned.
   - *Usage:* Calculates the robot's physical movement across the ground.

---

## ⚙️ Working Principle (The Pipeline)
The robot's brain is orchestrated by a central executor node (`gemini_mow_executor.py`). The step-by-step lifecycle is:

### 1. Bootup & Initialization
The `start_simulation.launch.py` script starts Gazebo, loads the URDF models, bridges the standard Gazebo topics to ROS 2, and boots the Nav2 lifecycle manager.

### 2. Phase 1: Boundary Discovery (Hardcoded Perimeter)
The executor sends a hardcoded loop of waypoints to the Nav2 `WaypointFollower`. The robot drives around the perimeter of the yard. During this time, the `slam_toolbox` is listening to the LiDAR and generating a black-and-white `OccupancyGrid` map of the yard.
> **Note:** Currently, this initial boundary loop is specific to the L-shaped `complex_lawn` world. To make the bot truly universal for *any* lawn shape, this phase will be upgraded to a "Wall Following" algorithm (driving forward until it sees a fence, turning 90 degrees, and hugging the wall until it makes a full loop).

### 3. Phase 2: Coverage Planning
Once the perimeter is complete, the `coverage_planner.py` node reads the generated SLAM map. It identifies all "Free Space" (white pixels). It then executes a **Boustrophedon Cellular Decomposition** algorithm. This algorithm sweeps a vertical line across the map to generate back-and-forth parallel lawnmowing lines. It successfully avoids the black pixels (trees, walls).

### 4. Phase 3: Autonomous Mowing & Dynamic Obstacle Avoidance
The executor receives the hundreds of zig-zag waypoints and feeds them to Nav2. The robot follows the path precisely using the `RegulatedPurePursuitController`.
As it drives, it publishes its path to `/mowed_path` (Red Line in RViz) and updates an overlay on `/mowed_area` (Green Grid in RViz) so the user can see exactly what grass has been cut!

**Dynamic Obstacle Reaction (e.g., A moving dog):**
If an object (like a dog or a person) walks in front of the mower *after* the initial SLAM map was built, the robot uses its real-time sensors (LiDAR & Camera) to populate a `local_costmap` in Nav2. 
- The local costmap temporarily paints a high-cost "bubble" around the moving object.
- The `RegulatedPurePursuitController` instantly detects this bubble and computes an arc to smoothly steer *around* the dog in real-time.
- If the dog blocks the entire path and the robot cannot drive around it, the robot will brake to a stop and wait for the dynamic object to move out of the way before resuming its path.

**What happens if the dog was sleeping during Phase 1 (SLAM Mapping) but wakes up and leaves during Phase 3?**
Because the Coverage Planner relies on a *static snapshot* of the SLAM map generated in Phase 1, the planner believes the dog is a permanent structure (like a rock). It will generate a path that intentionally loops around where the dog was sleeping.
When the robot drives by during Phase 3, the sensors will see that the dog is gone, but the robot will **not** automatically go back to mow that empty patch of grass because it has no waypoints directing it to go there. *See "Limitations & Future Improvements" for the solution to this.*

---

## 🎮 How to Run & Showcase

### 1. Start the Simulation
This drops the robot into a complex yard with trees, rocks, a garden bed, and a bench.
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch lawn_mower_bot start_simulation.launch.py
```

### 2. Monitor Visualizations
When the Gazebo window opens, an **RViz2** window will open alongside it.
In RViz, you will automatically see:
- The black-and-white SLAM map building in real time.
- The `Mowed Path` (red line).
- The `Mowed Area` (green overlay).

### 3. Observe the AI Computer Vision Feed
You can pop open a live feed of the computer vision obstacle detection using `rqt`:
```bash
ros2 run rqt_image_view rqt_image_view
```
*Select `/vision_debug_image` from the dropdown in the top left.*

### 4. Run Automated End-to-End Tests
To verify all internal logic works without a GUI:
```bash
cd ~/ros2_ws
source install/setup.bash
pytest src/lawn_mower_bot/test/test_mowing_pipeline.py -v -s
```

---

## ⌨️ Commands
The robot listens to commands on the `/mow_command` string topic. You can issue manual overrides mid-mow!

- **Normal Full Pipeline:**
  The robot automatically starts when launched, but you can restart it anytime:
  ```bash
  ros2 topic pub --once /mow_command std_msgs/msg/String "{data: 'start'}"
  ```
- **Hurry Up (Skip Lines):**
  If it starts raining, you can tell the robot to skip alternating lines to finish faster!
  ```bash
  ros2 topic pub --once /mow_command std_msgs/msg/String "{data: 'hurry'}"
  ```
- **Emergency Stop:**
  ```bash
  ros2 topic pub --once /mow_command std_msgs/msg/String "{data: 'stop'}"
  ```

---

## 🚧 Limitations & Future Improvements
While highly functional, the current architecture has areas for future expansion:

1. **Computer Vision Object Detection:**
   Currently relies on HSV Color Thresholding (identifying green vs non-green). This is fast but fails if grass is brown or lighting is low.
   *Improvement:* Integrate a YOLOv8 Nano model via ONNX to do real Object Detection, feeding dynamic humans/pets directly into the Nav2 Costmap.
2. **Path Smoothing:**
   The boustrophedon planner generates sharp 90-degree points at the end of each swath. While the pure pursuit controller takes a realistic arc over them, true Dubins-curve generation at the corners would prevent the robot from slowing down as much at the edges.
3. **Hardware Realization:**
   To translate this to the real world, the `base_link` needs to be hooked up to `ros2_control` hardware interfaces communicating via serial to a microcontroller (e.g. Arduino/Teensy) handling the physical motor PID loops and PWM signals.
4. **GPS Fusion:**
   Currently relies entirely on LiDAR/Odometry/IMU. A physical yard usually lacks perfectly straight walls to bounce LiDAR off of. Fusing an RTK-GPS sensor into the `robot_localization` EKF is highly recommended for real outdoor operation.
5. **Universal Boundary Discovery (Wall Following & Frontier Exploration):**
   Upgrading the Phase 1 mapping routine from a hardcoded list of waypoints to an active LiDAR-based Wall Following algorithm allows the boundary to be mapped. However, if the lawn is massive (e.g., >25 meters wide), the robot's 12-meter LiDAR will not be able to "see" the center of the yard while hugging the wall. This leaves a donut of mapped space with an "Unknown" grey blob in the center. 
   *Solution:* After Wall Following, the robot must execute a "Frontier Exploration" algorithm to actively drive into large patches of Unknown pixels until the entire center is painted white (Free Space), at which point Phase 2 (Coverage Planning) can begin!
6. **Continuous SLAM & Dynamic Coverage Replanning:**
   Currently, the system takes a static snapshot of the map after Phase 1. If an obstacle (e.g. a lawn chair) is moved *after* Phase 1, the robot will skip the grass where the chair used to be. A future improvement would allow `slam_toolbox` to continually update the global map during Phase 3, and trigger the `coverage_planner` to constantly recalculate missing patches of grass on the fly.
