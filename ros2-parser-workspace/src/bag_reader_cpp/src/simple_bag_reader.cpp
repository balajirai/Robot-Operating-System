#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

// optional, but nice to have:
#include "rosbag2_cpp/reader.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SimpleBagReader : public rclcpp::Node {
public:
  explicit SimpleBagReader(const std::string &bag_uri)
      : rclcpp::Node("simple_bag_reader") {
    // Create a timer so we read and print at ~10 Hz
    timer_ = this->create_wall_timer(100ms,
                                     std::bind(&SimpleBagReader::tick, this));

    // Set the storage options (uri is the bag folder path)
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_uri;              // e.g., /home/user/subset
    // storage_options.storage_id = "sqlite3";  // default; uncomment if needed

    // Build a reader (handles compressed/uncompressed automatically)
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    reader_->open(storage_options);
  }

private:
  void tick() {
    // Read until we find a message on /chatter, then print it and return.
    while (reader_->has_next()) {
      auto bag_msg = reader_->read_next();

      // Filter for our topic of interest (adjust if your bag uses another topic)
      if (bag_msg->topic_name != "/chatter") {
        continue;
      }

      // Convert serialized payload to a C++ ROS message
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto ros_msg = std::make_shared<std_msgs::msg::String>();
      serialization_.deserialize_message(&serialized, ros_msg.get());

      // Print timestamp, topic, and data
      std::cout << "[" << bag_msg->time_stamp << "] "
                << bag_msg->topic_name << " = " << ros_msg->data << std::endl;

      break; // process next message on next timer tick (~10 Hz)
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Serialization<std_msgs::msg::String> serialization_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_bag_directory>\n";
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagReader>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
