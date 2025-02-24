#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>
#include <functional>

class TFPublisherNode : public rclcpp::Node
{
public:
    TFPublisherNode() : Node("tf_publisher_node")
    {
        this->declare_parameter("config_file", "");
        std::string config_file = this->get_parameter("config_file").as_string();
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        loadConfig(config_file);
    }

private:
    struct TransformConfig {
        std::string frame_id;
        std::string child_frame_id;
        std::string timestamp_topic_name;
        std::string timestamp_topic_type;
        double x, y, z;
        double roll, pitch, yaw;
        rclcpp::Time last_timestamp;
    };

    bool loadConfig(const std::string& config_file)
    {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            
            for (const auto& tf : config["transforms"]) {
                TransformConfig tf_config;
                tf_config.frame_id = tf["frame_id"].as<std::string>();
                tf_config.child_frame_id = tf["child_frame_id"].as<std::string>();
                
                // 설정값 로드 및 로그 출력
                tf_config.x = tf["translation"]["x"].as<double>();
                tf_config.y = tf["translation"]["y"].as<double>();
                tf_config.z = tf["translation"]["z"].as<double>();
                tf_config.roll = tf["rotation"]["roll"].as<double>();
                tf_config.pitch = tf["rotation"]["pitch"].as<double>();
                tf_config.yaw = tf["rotation"]["yaw"].as<double>();

                // 설정값 로그 출력
                RCLCPP_INFO(this->get_logger(), 
                    "Loaded transform: %s -> %s, Translation: [%f, %f, %f], Rotation: [%f, %f, %f]",
                    tf_config.frame_id.c_str(), tf_config.child_frame_id.c_str(),
                    tf_config.x, tf_config.y, tf_config.z,
                    tf_config.roll, tf_config.pitch, tf_config.yaw);
                
                if (tf["timestamp_topic"]) {
                    tf_config.timestamp_topic_name = tf["timestamp_topic"]["name"].as<std::string>();
                    tf_config.timestamp_topic_type = tf["timestamp_topic"]["type"].as<std::string>();
                    
                    if (tf_config.timestamp_topic_type == "sensor_msgs/msg/PointCloud2") {
                        auto callback = [this, tf_config](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                            publishTransform(tf_config, rclcpp::Time(msg->header.stamp));
                        };
                        pointcloud_subs_.push_back(
                            create_subscription<sensor_msgs::msg::PointCloud2>(
                                tf_config.timestamp_topic_name, 10, callback));
                    }
                    else if (tf_config.timestamp_topic_type == "sensor_msgs/msg/Image") {
                        auto callback = [this, tf_config](const sensor_msgs::msg::Image::SharedPtr msg) {
                            publishTransform(tf_config, rclcpp::Time(msg->header.stamp));
                        };
                        image_subs_.push_back(
                            create_subscription<sensor_msgs::msg::Image>(
                                tf_config.timestamp_topic_name, 10, callback));
                    }
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "Transform %s -> %s using timestamp from topic: %s",
                        tf_config.frame_id.c_str(),
                        tf_config.child_frame_id.c_str(),
                        tf_config.timestamp_topic_name.c_str());
                }
                
                transform_configs_.push_back(tf_config);
            }
            
            // 타이머 설정 (타임스탬프 참조가 없는 transform을 위해)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&TFPublisherNode::timerCallback, this));
                
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
            return false;
        }
    }

    void timerCallback()
    {
        // 타임스탬프 참조가 없는 transform만 발행
        for (const auto& config : transform_configs_) {
            if (config.timestamp_topic_name.empty()) {
                publishTransform(config, this->now());
            }
        }
    }

    void publishTransform(const TransformConfig& config, const rclcpp::Time& timestamp)
    {
        geometry_msgs::msg::TransformStamped transform;
        
        rclcpp::Time current_time = this->now();
        rclcpp::Time transform_time = (timestamp < current_time) ? current_time : timestamp;
        
        transform.header.stamp = transform_time + rclcpp::Duration::from_seconds(0.3);
        transform.header.frame_id = config.frame_id;
        transform.child_frame_id = config.child_frame_id;
        
        // 변환값 설정 및 로그 출력
        transform.transform.translation.x = config.x;
        transform.transform.translation.y = config.y;
        transform.transform.translation.z = config.z;
        
        tf2::Quaternion q;
        q.setRPY(config.roll, config.pitch, config.yaw);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        // 발행되는 transform 값 로그 출력
        RCLCPP_DEBUG(this->get_logger(), 
            "Publishing transform: %s -> %s, Translation: [%f, %f, %f], Rotation(quaternion): [%f, %f, %f, %f]",
            transform.header.frame_id.c_str(), transform.child_frame_id.c_str(),
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
            transform.transform.rotation.x, transform.transform.rotation.y, 
            transform.transform.rotation.z, transform.transform.rotation.w);
        
        tf_broadcaster_->sendTransform(transform);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<TransformConfig> transform_configs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 