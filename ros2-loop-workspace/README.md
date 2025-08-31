# ROS2 Ping-Pong (Loop) Messaging with Bag Playback

![ROS2 Loop Messaging Diagram](./images/control-flow.png)

- Node A:
    - Reads from a rosbag2 (SQLite format).
    - Publishes each message (wrapped in JSON string).
    - Listens for Node B’s reply and republishes until `max_hops`.

- Node B:
    - Listens for Node A’s messages.
    - Modifies (`+1` if int, else append suffix).
    - Republishes with origin `B`. 

Both will use `std_msgs/msg/String` so we don’t need dynamic message types. JSON encoding is used to keep track of `"origin"`, `"hops"`, and `"data"`.

## 1. Create Workspace
```bash
mkdir -p ~/ros2-loop-workspace/src
```

## 2. Create a minimal C++ package
```bash
cd ~/ros2-loop-workspace/src
ros2 pkg create ros2_loop_demo_cpp --build-type ament_cmake --license Apache-2.0 --dependencies rclcpp std_msgs rosbag2_cpp
```

## 3. Add the C++ nodes (Node A + Node B)
- [Node A ](src/ros2_loop_demo_cpp/src/node_a.cpp)
- [Node B ](src/ros2_loop_demo_cpp/src/node_b.cpp)

## 4. Update the CMakeLists.txt
- [CMakeLists.txt](src/ros2_loop_demo_cpp/CMakeLists.txt)

## 5. Build the workspace
```bash
cd ~/ros2-loop-workspace
colcon build --packages-select ros2_loop_demo_cpp
```

## 6. Record a small bag for testing
### Terminal A
```bash
source install/setup.bash
ros2 bag record -o demo_bag /loop_topic
```
### Terminal B
```bash
source install/setup.bash
ros2 topic pub /loop_topic std_msgs/msg/String "data: '5'" -r 1
```

## 7. Run the nodes
### Terminal A
```bash
cd ~/ros2-loop-workspace
source install/setup.bash
ros2 run ros2_loop_demo_cpp node_a --ros-args -p bag_path:=demo_bag -p topic_name:=/loop_topic -p max_hops:=10 -p playback_sleep_sec:=0.5
```
### Terminal B
```bash
cd ~/ros2-loop-workspace
source install/setup.bash
ros2 run ros2_loop_demo_cpp node_b --ros-args -p topic_name:=/loop_topic -p max_hops:=10
```

## 8. Quick runtime introspection
```bash
# list nodes
ros2 node list

# list topics
ros2 topic list

# echo messages on topic
ros2 topic echo /loop_topic

# get info about topic
ros2 topic info /loop_topic
```

## Real-world analogy for Node A + Node B

Think of **autonomous vehicles** (self-driving cars).

* A self-driving car might record **sensor data** (camera frames, LiDAR scans, GPS) into a **rosbag** during a test drive.
* Later, engineers can **replay** that bag file into the system as if the car were driving in real life.

Now you want to test a **multi-stage processing pipeline**:

1. **Node A (Data Player)**

   * Reads from rosbag (recorded sensor data).
   * Publishes it into the system.
   * Acts like a *sensor emulator*.

2. **Node B (Processor/Analyzer)**

   * Listens to the data.
   * Applies some transformation: e.g. detecting pedestrians, adjusting map coordinates, or filtering noise.
   * Sends the result back for further processing.

3. **Node A again**

   * Receives processed data, maybe applies scaling, cleaning, or additional processing.
   * Sends it back to Node B for another stage.

This back-and-forth continues, simulating **how real modules in a robotic system interact**. The `hops` counter is like a safety mechanism to prevent infinite looping.

## Example Scenarios

### 1. Robot Sensor Replay + Data Processing

* **Node A**: Plays back recorded LiDAR scans.
* **Node B**: Applies filtering (e.g., removes noise).
* Data bounces back and forth until fully processed (e.g., filter → transform → detect).

### 2. Distributed Computation

Imagine two robots working together:

* **Robot A**: Sends raw measurement (temperature sensor).
* **Robot B**: Adjusts it based on calibration.
* **Robot A**: Further scales/normalizes it.
* This repeats until the reading is finalized and logged.

### 3. Communication Testing

Sometimes engineers don’t want to deploy expensive sensors every time.

* Node A plays recorded sensor data.
* Node B simulates a remote system (like cloud processing, or another robot).
* They exchange messages to test network latency, correctness, and reliability.

## Simplified analogy (non-robotics)

Imagine **two translators**:

* Translator A: Takes an English word, doubles it.
* Translator B: Takes that result, adds 1 or modifies it further.
* They pass words back and forth until both agree or until they’ve modified it enough times.

That’s essentially what’s happening here — the "hops" is like saying “don’t translate forever.”
