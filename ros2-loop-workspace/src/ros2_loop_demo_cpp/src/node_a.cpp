#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <nlohmann/json.hpp>  // for JSON encoding (you’ll add to CMakeLists)

#include <thread>
#include <chrono>
#include <string>

using namespace std::chrono_literals;
using json = nlohmann::json;

class NodeA : public rclcpp::Node{
public:
    NodeA() : Node("node_a"){
        bag_path_ = this->declare_parameter<std::string>("bag_path", "");
        topic_name_ = this->declare_parameter<std::string>("topic_name", "/loop_topic");
        max_hops_ = this->declare_parameter<int>("max_hops", 10);
        playback_sleep_sec_ = this->declare_parameter<double>("playback_sleep_sec", 0.5);

        if (bag_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'bag_path' is required");
            throw std::runtime_error("No bag_path provided");
        }

        pub_ = this->create_publisher<std_msgs::msg::String>(topic_name_, 10);
        sub_ = this->create_subscription<std_msgs::msg::String>(
            topic_name_, 10,
            std::bind(&NodeA::on_msg, this, std::placeholders::_1));

        reader_thread_ = std::thread(&NodeA::play_bag, this);

        RCLCPP_INFO(this->get_logger(), "Node A ready. bag=%s topic=%s max_hops=%d",
                    bag_path_.c_str(), topic_name_.c_str(), max_hops_);
    }

    ~NodeA(){
        stop_flag_ = true;
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
    }

private:
    void play_bag(){
        try {
            rosbag2_cpp::Reader reader;
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = bag_path_;
            storage_options.storage_id = "sqlite3";

            rosbag2_cpp::ConverterOptions converter_options{"", ""};

            reader.open(storage_options, converter_options);

            rclcpp::Serialization<std_msgs::msg::String> serializer;

            size_t count = 0;
            while (rclcpp::ok() && !stop_flag_ && reader.has_next()) {
                auto bag_msg = reader.read_next();

                if (bag_msg->topic_name != topic_name_) {
                    continue;
                }

                std_msgs::msg::String msg;
                rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
                serializer.deserialize_message(&serialized_msg, &msg);

                // Wrap into JSON
                json payload = {
                    {"origin", "A"},
                    {"hops", 0},
                    {"data", try_parse_int(msg.data)}
                };

                std_msgs::msg::String out;
                out.data = payload.dump();
                pub_->publish(out);
                RCLCPP_INFO(this->get_logger(), "[A] published from bag -> %s", out.data.c_str());

                count++;
                std::this_thread::sleep_for(std::chrono::duration<double>(playback_sleep_sec_));
            }
            RCLCPP_INFO(this->get_logger(), "[A] finished bag playback. Published %zu messages.", count);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading bag: %s", e.what());
        }
    }

    void on_msg(const std_msgs::msg::String::SharedPtr msg){
        try {
            auto payload = json::parse(msg->data);
            if (payload["origin"] != "B") return;

            int hops = payload["hops"];
            if (hops >= max_hops_) {
                RCLCPP_INFO(this->get_logger(), "[A] reached max_hops");
                return;
            }

            auto data = payload["data"];
            if (data.is_number_integer()) {
                data = data.get<int>() * 2;
            }
            else {
                data = data.get<std::string>() + "-procA";
            }

            json new_payload = {
                {"origin", "A"},
                {"hops", hops + 1},
                {"data", data}
            };

            std_msgs::msg::String out;
            out.data = new_payload.dump();
            pub_->publish(out);
            RCLCPP_INFO(this->get_logger(), "[A] reply -> %s", out.data.c_str());
        }
        catch (...) {
            RCLCPP_WARN(this->get_logger(), "[A] failed to parse incoming msg");
        }
    }

    static nlohmann::json try_parse_int(const std::string &s){
        try {
            return std::stoi(s);
        }
        catch (...) {
            return s;
        }
    }

    std::string bag_path_;
    std::string topic_name_;
    int max_hops_;
    double playback_sleep_sec_;
    bool stop_flag_{false};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::thread reader_thread_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeA>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
