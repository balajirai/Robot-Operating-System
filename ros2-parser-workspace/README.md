# A rosbag-parser ROS2 workspace

## 1. Create Workspace
```bash
mkdir -p ~/ros2-parser-workspace/src
```

## 2. Create a C++ package
```bash
cd ~/ros2-parser-workspace/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 bag_reader_cpp --dependencies rclcpp rosbag2_transport std_msgs
```

## 3. Add the parser code
- [Rosbag Parser/Reader](src/bag_reader_cpp/src/simple_bag_reader.cpp)

## 4. Update the CMakeLists.txt
- [CMakeLists.txt](src/bag_reader_cpp/CMakeLists.txt)

## 5. Build the workspace
```bash
cd ~/ros2-parser-workspace
colcon build --packages-select bag_reader_cpp
```

## 6. Run the parser
```bash
source install/setup.bash
ros2 run bag_reader_cpp simple_bag_reader subset
```
> [!Note]
> Here `subset` is a sample/recorded rosbag folder which contains: `metadata.yaml` and `subset_0.db3`
