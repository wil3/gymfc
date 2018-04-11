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

#define MAX_MOTORS 255

namespace gazebo
{
/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  float motorSpeed[MAX_MOTORS];

  /// \brief packet sequence to keep in sync
//  unsigned int seq;
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
  uint64_t iter;
  
//  unsigned int seq;
};
  class Rotor
 {
	 public: Rotor();

	  /// \brief rotor id
	  public: int id = 0;

	  /// \brief Max rotor propeller RPM.
	  public: double maxRpm = 838.0;

	  /// \brief Next command to be applied to the propeller
	  public: double cmd = 0;

	  /// \brief Velocity PID for motor control
	  public: common::PID pid;

	  /// \brief Control propeller joint.
	  public: std::string jointName;

	  /// \brief Control propeller joint.
	  public: physics::JointPtr joint;

	  /// \brief direction multiplier for this rotor
	  public: double multiplier = 1;

	  /// \brief unused coefficients
	  public: double rotorVelocitySlowdownSim = 10.0;
	  public: double frequencyCutoff = 5.0;
	  public: double samplingRate = 0.2;
	  public: ignition::math::OnePole<double> velocityFilter;

 };
  class QuadcopterWorldPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: QuadcopterWorldPlugin();

    /// \brief Destructor.
    public: ~QuadcopterWorldPlugin();

    // Documentation Inherited.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
	public: void processSDF(sdf::ElementPtr _sdf);

	public: void loop_thread();
	

	public: bool Bind(const char *_address, const uint16_t _port);
    public: void MakeSockAddr(const char *_address, const uint16_t _port, struct sockaddr_in &_sockaddr);
  	public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs);
    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    private: void ApplyMotorForces(const double _dt);

    /// \brief Reset PID Joint controllers.
    private: void ResetPIDs();

    /// \brief Receive motor commands from Quadcopter
    private: bool ReceiveMotorCommand();

    /// \brief Send state to Quadcopter
    private: void SendState() const;
	private: void softReset();

	private: boost::thread _callback_loop_thread;

	/// \brief How fast in Hertz the inner loop runs
	private: double loopRate;

	private: bool resetWithRandomAngularVelocity;
	private: int randomSeed;
	private: ignition::math::Vector2d rollLimit;
	private: ignition::math::Vector2d pitchLimit;
	private: ignition::math::Vector2d yawLimit;


	public: physics::WorldPtr _world;
	
	public: physics::ModelPtr _model;

	/// \brief Pointer to the update event connection.
	public: event::ConnectionPtr updateConnection;

	/// \brief array of propellers
	public: std::vector<Rotor> rotors;

	/// \brief keep track of controller update sim-time.
	public: gazebo::common::Time lastControllerUpdateTime;

	/// \brief Controller update mutex.
	public: std::mutex mutex;

	/// \brief Socket handle
	public: int handle;

	public: struct sockaddr_in remaddr;

	public: socklen_t remaddrlen;

	/// \brief Pointer to an IMU sensor
	public: sensors::ImuSensorPtr imuSensor;

	/// \brief false before ardupilot controller is online
	/// to allow gazebo to continue without waiting
	public: bool arduCopterOnline;

	/// \brief number of times ArduCotper skips update
	public: int connectionTimeoutCount;

	/// \brief number of times ArduCotper skips update
	/// before marking Quadcopter offline
	public: int connectionTimeoutMaxCount;
  };
}
#endif
