/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GAZEBO_PLUGINS_QUADCOPTERWORLDPLUGIN_HH_
#define GAZEBO_PLUGINS_QUADCOPTERWORLDPLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
//#include <boost/thread.hpp>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/sensors.hh>

#include <gazebo/physics/Base.hh>
#include "gazebo/transport/transport.hh"

#include "CommandMotorSpeed.pb.h"

#define MAX_MOTORS 255

namespace gazebo
{
  static const std::string kDefaultCmdPubTopic = "/gazebo/command/motor_speed";

/// \brief A servo packet.
struct ServoPacket
{
	/// \brief Flag to indicate the world should be reset
	int resetWorld;
	/// \brief Motor control signal [0,1].
	float motor[MAX_MOTORS];

};

/// \brief Flight Dynamics Model packet that is sent back to the Quadcopter
// NOTE: Because of struct padding and how this is serialized to bytes must be in multiple of 4? 8?
struct fdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity in NED frame
  double velocityXYZ[3];

  /// \brief Model position in NED frame
  double positionXYZ[3];

  double motorVelocity[4];
 // uint32_t iter;
  uint64_t status_code;
  
//  unsigned int seq;
};
class FlightControllerPlugin : public WorldPlugin
{
  /// \brief Constructor.
  public: FlightControllerPlugin();

  /// \brief Destructor.
  public: ~FlightControllerPlugin();

  // The function called when the plugin in loaded 
  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
	public: void ProcessSDF(sdf::ElementPtr _sdf);

	public: void LoopThread();
	

	public: bool Bind(const char *_address, const uint16_t _port);
  public: void MakeSockAddr(const char *_address, const uint16_t _port, struct sockaddr_in &_sockaddr);
  public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs);

  /// \brief Receive motor commands from Quadcopter
  private: bool ReceiveMotorCommand();

  /// \brief Send state to Quadcopter
  private: void SendState(bool motorCommandProcessed) const;
	private: void SoftReset();

  private: std::string robotNamespace;

	private: boost::thread callbackLoopThread;

	/// \brief How fast in Hertz the inner loop runs
	private: double loopRate;

	private: bool resetWithRandomAngularVelocity;
	private: int randomSeed;
	private: ignition::math::Vector2d rollLimit;
	private: ignition::math::Vector2d pitchLimit;
	private: ignition::math::Vector2d yawLimit;


	public: physics::WorldPtr world;
	
	/// \brief Pointer to the update event connection.
	public: event::ConnectionPtr updateConnection;

	/// \brief keep track of controller update sim-time.
	public: gazebo::common::Time lastControllerUpdateTime;

	/// \brief Controller update mutex.
	public: std::mutex mutex;

	/// \brief Socket handle
	public: int handle;

	public: struct sockaddr_in remaddr;

	public: socklen_t remaddrlen;

	/// \brief True if world should be reset
	public: bool resetWorld;
	/// \brief false before ardupilot controller is online
	/// to allow gazebo to continue without waiting
	public: bool aircraftOnline;

  private: int numActuators;

	/// \brief number of times ArduCotper skips update
	public: int connectionTimeoutCount;

	/// \brief number of times ArduCotper skips update
	/// before marking Quadcopter offline
	public: int connectionTimeoutMaxCount;

  private: std::string cmdPubTopic;
  private: transport::NodePtr nodeHandle;
  // Now define the communication channels with the digital twin
  // The architecure treats this world plugin as the flight controller
  // while all other aircraft components are now external and communicated
  // over protobufs
  private: transport::PublisherPtr cmdPub;

   // Subscribe to all possible sensors
  private: transport::SubscriberPtr sensorSub;
  private: cmd_msgs::msgs::CommandMotorSpeed cmdMsg;
  };
}
#endif
