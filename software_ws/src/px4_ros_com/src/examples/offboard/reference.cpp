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
#include "/home/maav/SUAS_24-25/software_ws/src/px4-ros2-interface-lib/px4_ros2_cpp/include/px4_ros2/odometry/local_position.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <Eigen/Dense>
#include <fstream>
// #include <geometry_msgs/PoseStamped.h>
#include "geodetic_conv.hpp"
#include <cmath>

// #include <GeographicLib/Geocentric.hpp>
// #include <GeographicLib/LocalCartesian.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Point {
	double east, north, up;
};

void print_point(Point & p) {
	std::cout << p.east << " " << p.north << " " << p.up << " " << std::endl;
}

const double start_coord_lat = 38.31633;
const double start_coord_long = -76.55578;
const double start_coord_alt = 142;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control"), curr_target(0)
	{
		//publishers
		geodetic_converter_.initialiseReference(start_coord_lat, start_coord_long, start_coord_alt);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		
		// // figuring out how to find location // node before
		// global_position_subscriber_ = this->create_subscription<VehicleGlobalPosition>(
        //     "/fmu/out/vehicle_global_position", 10,
        //     std::bind(&OffboardControl::global_position_callback, this, std::placeholders::_1));
		
		rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
		qos_profile.best_effort();
		global_position_subscriber_ = this->create_subscription<VehicleGlobalPosition>(
			"/fmu/out/vehicle_global_position", qos_profile,
			std::bind(&OffboardControl::global_position_callback, this, std::placeholders::_1));

		// setting initial reference to origin for now
		// getting list of waypoints
		std::string waypointFilePath = "/home/maav/SUAS_24-25/software_ws/src/waypoint_generation/way_points.txt";
		waypoints = read_waypoints(waypointFilePath, geodetic_converter_);
		std::cout << waypoints.size() << std::endl;

		auto node = rclcpp::Node::make_shared("vehicle_global_position_subscriber"); 

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

			// get east, north, up from our points function and then publish them
			//Point result = get_point();
			Point result = get_point_vel();

			publish_trajectory_setpoint(result.east, result.north, result.up);
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

	// list of waypoints
	std::vector<Point> waypoints;
	// std::vector<Point> long_lat_waypoints;
	int curr_target = 0;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	double current_x_ = 0.0;
	double current_y_ = 0.0;
	double current_z_ = 0.0; // Current position of the drone

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	int num_calls;

	geodetic_converter::GeodeticConverter geodetic_converter_;
	
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float east, float north, float up);
	// Point get_point();
	Point get_point_vel();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void global_position_callback(const VehicleGlobalPosition::SharedPtr msg);
	std::vector<Point> read_waypoints(const std::string& file_path, geodetic_converter::GeodeticConverter &geodetic_converter_);
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
	// We probably need velocity waypoints for fixed-wing flight
	if(curr_target == 0) {
		msg.position = true;
		msg.velocity = false;
	}
	else {
		msg.position = false;
		msg.velocity = true;
	}
	// msg.position = true;
	// msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

std::array<double, 3> OffboardControl::get_point_vel() {
    return {1.0, 0.0, 0.0};  // Example: Move forward at 1 m/s
}


Point OffboardControl::get_point() {
	// get distance between curr_pos and curr_target
	Point target_point = waypoints[curr_target];
	double east, north, up;
	std::cout << "Current gps " << current_x_ << " " <<
	current_y_ << " " << 
	current_z_ << std::endl;
	geodetic_converter_.geodetic2Enu(current_x_, current_y_, current_z_, &north, &east, &up);
	// because gazebo uses North East Down, we invert current_z_ ig
	double distance_from_target = sqrt(pow((east- target_point.east), 2) +
									   pow((north - target_point.north), 2) + 
									   pow((-current_z_ - target_point.up), 2));
	// if we're not close enough to target waypoint, keep going	
	std::cout << "Distance from target(m): " << distance_from_target << std::endl;
	std::cout << "Target point east, north, up ";
	print_point(target_point);
	std::cout << "Current east, north, up " << east << " " <<
									  north << " " << 
									  current_z_ << std::endl;


	const int waypoint_range = 10;
	if (distance_from_target > waypoint_range) {
		return target_point;
	}
	else {
		// if we've reached the last waypoint, go back to (0, 0, 0)
		if(curr_target == waypoints.size() - 1) {
			// Point origin = {0, 0, 0};
			// return origin;
			curr_target = 0;
			return waypoints[curr_target];
		}
		else {
			curr_target++;
			// Once we reach the first waypoint, we could switch to fixed wing here
			if(curr_target == 1) {
				publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, 4.0, 0.0);
			}
		}
		// std::cout << waypoints[curr_target].east << std::endl;
		return waypoints[curr_target];
	}

	// if(this->num_calls > 50)
	// {
	// 	this->num_calls+=1;
	// 	if(this->num_calls > 100)
	// 	{
	// 		this->num_calls = 0;
	// 	}
	// 	return {5.0, 5.0, -5.0};
	// }
	// else 
	// {
	// 	this->num_calls+=1;
	// 	return {0.0, 0.0, -5.0};
	// }
	// return {2.0, -100.0, 3.0};
};




void OffboardControl::global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    current_x_ = msg->lat; // Latitude
    current_y_ = msg->lon; // Longitude
    current_z_ = msg->alt; // Altitude
	// auto message = "east: " + std::to_string(current_x_) + " north: " + std::to_string(current_y_) + " up: " + std::to_string(current_z_);
	// RCLCPP_INFO(this->get_logger(), message.c_str());

    // RCLCPP_INFO(this->get_logger(), "Global Position Updated: [%f, %f, %f]", current_x_, current_y_, current_z_);
	
//	std::cout << "{PE:AASE}";
	// Point curr_pos;
	// curr_pos.east = current_x_;
	// curr_pos.north = current_y_;
	// curr_pos.up = current_z_;
	// return curr_pos;
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
// void OffboardControl::publish_trajectory_setpoint(float east, float north, float up)
// {
// 	TrajectorySetpoint msg{};
// 	msg.position = {east, north, up};
// 	msg.yaw = -3.14; // [-PI:PI]
// 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
// 	trajectory_setpoint_publisher_->publish(msg);
// }
void OffboardControl::publish_trajectory_setpoint(double vx, double vy, double vz)
{
	TrajectorySetpoint msg{};
	msg.velocity = {vx, vy, vz}; // Velocity command in m/s
	msg.yaw = NAN; // No yaw control
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
std::vector<Point> OffboardControl::read_waypoints(const std::string& file_path, geodetic_converter::GeodeticConverter &geodetic_converter_) {
    std::ifstream file(file_path);
    std::vector<Point> waypoints;

    if (file.is_open()) {
        std::string line;
        std::getline(file, line);
        std::getline(file, line);
        // WGS84 ellipsoid parameters
        // const double a = 6378137.0;  // semi-major axis in meters
        // const double f = 1.0 / 298.257223563;  // flattening

        // GeographicLib::Geocentric earth(a, f);
        // GeographicLib::LocalCartesian proj(0, 0, 0, earth);

		//get the first point, make that our origin
		// double origin_x, origin_y, origin_z;
		// if(std::getline(file, line)) {
		// 	double origin_lat, origin_lon, origin_alt_ft;
		// 	std::istringstream iss(line);
        //     iss >> origin_lat >> origin_lon >> origin_alt_ft;
		// 	geodetic_converter_.geodetic2Enu(origin_lat, origin_lon, origin_alt_ft*.3048, &origin_x, &origin_y, &origin_z);
        //     waypoints.push_back({0, 0, 0});  //our first xyz is at the origin which is 0, 0, 0
		// 	std::cout << "Added new point: " << waypoint.east << " " << waypoint.north << " " << waypoint.up << std::endl;

		// }

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Point waypoint;
            double lat, lon, alt_ft;
            iss >> lat >> lon >> alt_ft;
			double east, north, up;

            //RCLCPP_INFO(this->get_logger(), "Reading waypoints from txt file: east=%f, north=%f, up=%f", lat, lon, alt_ft);
			// long_lat_waypoints.push_back({lat - 38.31633, lon - (-76.55578), alt_ft*.3048});
			geodetic_converter_.geodetic2Enu(lat, lon, alt_ft*.3048, &north, &east, &up);
            waypoint.east = east;
			waypoint.north = north;
			// trying to give it the original altitude, we think we can just keep it this way
			waypoint.up = alt_ft*.3048*(-1);	
			std::cout << "Added new point: " << waypoint.east << " " << waypoint.north << " " << waypoint.up << std::endl;
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
