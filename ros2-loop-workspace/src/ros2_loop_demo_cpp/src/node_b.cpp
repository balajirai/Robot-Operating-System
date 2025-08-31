#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class NodeB : public rclcpp::Node{
public:
    NodeB() : Node("node_b"){
        topic_name_ = this->declare_parameter<std::string>("topic_name", "/loop_topic");
        max_hops_ = this->declare_parameter<int>("max_hops", 10);

        pub_ = this->create_publisher<std_msgs::msg::String>(topic_name_, 10);
        sub_ = this->create_subscription<std_msgs::msg::String>(
            topic_name_, 10,
            std::bind(&NodeB::on_msg, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node B ready. topic=%s max_hops=%d",
                    topic_name_.c_str(), max_hops_);
    }

private:
    void on_msg(const std_msgs::msg::String::SharedPtr msg){
        try {
            auto payload = json::parse(msg->data);
            if (payload["origin"] != "A") return;

            int hops = payload["hops"];
            if (hops >= max_hops_) {
                RCLCPP_INFO(this->get_logger(), "[B] reached max_hops");
                return;
            }

            auto data = payload["data"];
            if (data.is_number_integer()) {
                data = data.get<int>() + 1;
            }
            else {
                data = data.get<std::string>() + "-procB(+1)";
            }

            json new_payload = {
                {"origin", "B"},
                {"hops", hops + 1},
                {"data", data}
            };

            std_msgs::msg::String out;
            out.data = new_payload.dump();
            pub_->publish(out);
            RCLCPP_INFO(this->get_logger(), "[B] reply -> %s", out.data.c_str());
        }
        catch (...) {
            RCLCPP_WARN(this->get_logger(), "[B] failed to parse incoming msg");
        }
    }

    std::string topic_name_;
    int max_hops_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeB>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
