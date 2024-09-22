
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class BikeSimulation : public rclcpp::Node {
    public:

        BikeSimulation() : Node("bike_simulation")
        {
            rclcpp::Parameter use_sim_time_param("use_sim_time", true);
            this->set_parameter(use_sim_time_param);

            model_state_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
                "model_states",
                10,
                std::bind(&BikeSimulation::publishBaseLinkTransform, this, _1));

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        }

    private:

        rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_state_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        void publishBaseLinkTransform(const gazebo_msgs::msg::ModelStates::SharedPtr msg) 
        {
            int index = -1;

            for (long unsigned int model = 0; model < msg->name.size(); model++) {
                if (msg->name[model] == "bike") {
                    index = model;
                }
            }

            if (index >= 0) {
                geometry_msgs::msg::TransformStamped base_link_transform;

                base_link_transform.header.stamp = this->get_clock()->now();
                base_link_transform.header.frame_id = "world";
                base_link_transform.child_frame_id = "base_link";

                base_link_transform.transform.translation.x = msg->pose[index].position.x;
                base_link_transform.transform.translation.y = msg->pose[index].position.y;
                base_link_transform.transform.translation.z = msg->pose[index].position.z;

                base_link_transform.transform.rotation.x = msg->pose[index].orientation.x;
                base_link_transform.transform.rotation.y = msg->pose[index].orientation.y;
                base_link_transform.transform.rotation.z = msg->pose[index].orientation.z;
                base_link_transform.transform.rotation.w = msg->pose[index].orientation.w;

                tf_broadcaster_->sendTransform(base_link_transform);
            } else {
                RCLCPP_WARN(this->get_logger(), "Bike Model not found in /ModelStates topic");
            }

        }
        void getMousePose() {
            
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BikeSimulation>());
    rclcpp::shutdown();
    return 0;
}