#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
        this->read_in_file();


		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		global_position_subscriber_ = this->create_subscription<VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", 10,
            std::bind(&OffboardControl::global_position_callback, this, std::placeholders::_1));
		

		offboard_setpoint_counter_ = 0;


		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			// get x, y, z from our points function and then publish them
			std::array<double, 3> result = get_point();
			publish_trajectory_setpoint(result[0], result[1], result[2]);
			RCLCPP_INFO(this->get_logger(), std::to_string(result[0]).c_str());
			// sleep(10.0);
			// publish_trajectory_setpoint(5.0, 5.0, -5.0);
			// RCLCPP_INFO(this->get_logger(), "Second waypoint published.");

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// subcription to drone's position
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_position_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	double current_x_, current_y_, current_z_; // Current position of the drone

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(double x, double y, double z);

    void read_in_file();
	std::array<double, 3> get_point();
    vector<std::array<double, 3>> points;

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void global_position_callback(const VehicleGlobalPosition::SharedPtr msg);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}
void OffboardControl::read_in_file() {
    std::ifstream infile("way_points.txt");
    std::string line;
    std::vector<std::vector<double>> waypoints;

    if (!infile) {
        std::cerr << "Error opening file" << std::endl;
        return 1;
    }

    // Skip the first two lines (headers)
    std::getline(infile, line);  // "Total path length: ..."
    std::getline(infile, line);  // "Total points: ..."

    // Read each point and store in the waypoints vector
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double x, y, angle;
        
        if (iss >> x >> y >> angle) { // Read the x, y, and angle values, we need to convert to xyz here or somewhere later
        
            this->points.push_back({x, y, angle});
        }
    }
}

std::array<double, 3> OffboardControl::get_point() {
	 
    // do a position check, if we're sutitably close to the point, then move to the next point by popping front of vector
	

    return this.points[0];
};

// gets drone pos
void OffboardControl::global_position_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    current_x_ = msg->lat; // Latitude
    current_y_ = msg->lon; // Longitude
    current_z_ = msg->alt; // Altitude

	RCLCPP_INFO(this->get_logger(), "x: " + current_x_ + " y: " + current_y_ + " z: " + current_z_);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(double x, double y, double z)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
