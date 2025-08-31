# A basic ROS2 workspace

## 1. Create Workspace
```bash
mkdir -p ~/ros2-basic-workspace/src
```

## 2. Create a minimal C++ package
```bash
cd ~/ros2-basic-workspace/src
ros2 pkg create cpp_pubsub --build-type ament_cmake --license Apache-2.0 --dependencies rclcpp std_msgs
```

## 3. Add the minimal C++ nodes (talker + listener)
- [Talker Node](src/cpp_pubsub/src/talker.cpp)
- [Listener Node](src/cpp_pubsub/src/listener.cpp)

## 4. Update the CMakeLists.txt
- [CMakeLists.txt](src/cpp_pubsub/CMakeLists.txt)

## 5. Build the workspace
```bash
cd ~/ros2-basic-workspace
colcon build --packages-select cpp_pubsub
```

## 6. Run the nodes
### Terminal A
```bash
cd ~/ros2-basic-workspace
source install/setup.bash
ros2 run cpp_pubsub talker
```
### Terminal B
```bash
cd ~/ros2-basic-workspace
source install/setup.bash
ros2 run cpp_pubsub listener
```

## 7. Quick runtime introspection
```bash
# list nodes
ros2 node list

# list topics
ros2 topic list

# echo messages on topic
ros2 topic echo /chatter

# get info about topic
ros2 topic info /chatter
```
