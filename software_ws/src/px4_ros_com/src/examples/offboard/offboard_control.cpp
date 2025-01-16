/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <Eigen/Dense>
#include <fstream>
// #include <geometry_msgs/PoseStamped.h>
#include "geodetic_conv.hpp"

// #include <GeographicLib/Geocentric.hpp>
// #include <GeographicLib/LocalCartesian.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Point {
	double x, y, z;
};

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		
		// setting initial reference to origin for now
		geodetic_converter_.initialiseReference(0, 0, 0);

		auto node = rclcpp::Node::make_shared("vehicle_global_position_subscriber"); 
		// auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)); 
		// qos.reliability(RMW_QOS); // Match this to the publisher
		// auto subscription = node->create_subscription<VehicleGlobalPosition>("/fmu/out/vehicle_global_position", qos, 
		// [](auto msg) { RCLCPP_INFO(this->get_logger(), "Received data: [%f, %f]", msg->lat, msg->long); });
		global_position_subscriber_ = node->create_subscription<VehicleGlobalPosition>(
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
			std::array<float, 3> result = get_point();
			publish_trajectory_setpoint(result[0], result[1], result[2]);
			// RCLCPP_INFO(this->get_logger(), );
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
	int num_calls;

	geodetic_converter::GeodeticConverter geodetic_converter_;
	
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z);
	std::array<float, 3> get_point();
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
std::array<float, 3> OffboardControl::get_point() {
	// get our current location and get next point based on that
	//
	if(this->num_calls > 50)
	{
		this->num_calls+=1;
		if(this->num_calls > 100)
		{
			this->num_calls = 0;
		}
		return {5.0, 5.0, -5.0};
	}
	else 
	{
		this->num_calls+=1;
		return {0.0, 0.0, -5.0};
	}

};

// gets drone pos
void OffboardControl::global_position_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    current_x_ = msg->lat; // Latitude
    current_y_ = msg->lon; // Longitude
    current_z_ = msg->alt; // Altitude
	auto message = "x: " + std::to_string(current_x_) + " y: " + std::to_string(current_y_) + " z: " + std::to_string(current_z_);
	RCLCPP_INFO(this->get_logger(), message.c_str());
	std::cout << "{PE:AASE}";
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float x, float y, float z)
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


// outputs in lat long height in meters
std::vector<Point> read_waypoints(const std::string& file_path, geodetic_converter::GeodeticConverter &geodetic_converter_) {
    std::ifstream file(file_path);
    std::vector<Point> waypoints;

    if (file.is_open()) {
        std::string line;
        std::getline(file, line);
        std::getline(file, line);
        // WGS84 ellipsoid parameters
        const double a = 6378137.0;  // semi-major axis in meters
        const double f = 1.0 / 298.257223563;  // flattening

        // GeographicLib::Geocentric earth(a, f);
        // GeographicLib::LocalCartesian proj(0, 0, 0, earth);

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Point waypoint;

            double lat, lon, alt_ft;
            iss >> lat >> lon >> alt_ft;

            double x, y, z;
            //proj.Forward(lat, lon, alt_ft * 0.3048, x, y, z);


            //RCLCPP_INFO(this->get_logger(), "Reading waypoints from txt file: x=%f, y=%f, z=%f", lat, lon, alt_ft);
			geodetic_converter_.geodetic2Enu(lat, lon, alt_ft*.3048, &x, &y, &z);
            waypoint.x = x;
			waypoint.y = y;
			waypoint.z = z;
            
            waypoints.push_back(waypoint);
        }
        file.close();
    } else {
        //RCLCPP_ERROR(this->get_logger(), "Unable to open waypoints file");
    }

    return waypoints;
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
