#include <rclcpp/rclcpp.hpp>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class BikeSimulation : public rclcpp::Node {
    public:

        BikeSimulation();

    private:
        
        void publishBaseLinkTransform(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

        // Subscribers
        rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_state_sub_;
        
        // Publishers
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};