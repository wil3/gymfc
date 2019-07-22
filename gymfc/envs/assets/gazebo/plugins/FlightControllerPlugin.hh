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
#include "gazebo/common/Events.hh"

#include <gazebo/physics/Base.hh>
#include "gazebo/transport/transport.hh"

#include "MotorCommand.pb.h"
#include "EscSensor.pb.h"
#include "Imu.pb.h"
#include "State.pb.h"
#include "Action.pb.h"

#define ENV_SITL_PORT "GYMFC_SITL_PORT"
#define ENV_DIGITAL_TWIN_SDF "GYMFC_DIGITAL_TWIN_SDF"
#define ENV_NUM_MOTORS "GYMFC_NUM_MOTORS"
#define ENV_SUPPORTED_SENSORS "GYMFC_SUPPORTED_SENSORS"

namespace gazebo
{
  static const std::string kDefaultCmdPubTopic = "/aircraft/command/motor";
  static const std::string kDefaultImuSubTopic = "/aircraft/sensor/imu";
  static const std::string kDefaultEscSubTopic = "/aircraft/sensor/esc";
 // TODO Change link name to CoM
  const std::string DIGITAL_TWIN_ATTACH_LINK = "base_link";
  const std::string kTrainingRigModelName = "attitude_control_training_rig";

  const std::string kAircraftConfigFileName = "libAircraftConfigPlugin.so";

  typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
  typedef const boost::shared_ptr<const sensor_msgs::msgs::EscSensor> EscSensorPtr;

  /// \brief List of all supported sensors. The client must
  // tell us which ones it will use. The client must be aware of the 
  // digitial twin they are using and that it supports the corresponding 
  // sensors.
  enum Sensors {
    IMU,
    ESC,
    BATTERY
  };

class FlightControllerPlugin : public WorldPlugin
{
  /// \brief Constructor.
  public: FlightControllerPlugin();

  /// \brief Destructor.
  public: ~FlightControllerPlugin();

  /// \brief The function called when the plugin in loaded 
  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  private: void LoadVars();

  /// \brief Parse and process SDF 
	public: void ProcessSDF(sdf::ElementPtr _sdf);

  /// \brief Dynamically load the digitial twin from the location
  // specified by the environment variable.
  private: void LoadDigitalTwin();

  private: void ParseDigitalTwinSDF();

  /// \brief Main loop thread waiting for incoming UDP packets
	public: void LoopThread();
	
  /// \brief Bind to the specified port to receive UDP packets
	public: bool Bind(const char *_address, const uint16_t _port);

  /// \brief Helper to make a socket
  public: void MakeSockAddr(const char *_address, const uint16_t _port, struct sockaddr_in &_sockaddr);

  /// \brief Receive action including motor commands 
  private: bool ReceiveAction();

  /// \brief Initialize a single protobuf state that is 
  // reused throughout the simulation.
  private: void InitState();

  /// \brief Send current state  
  private: void SendState() const;

  /// \brief Reset the world time and model, differs from 
  // world reset such that the random number generator is not 
  // reset.
	private: void SoftReset();


  /// \brief Callback from the digital twin to recieve ESC sensor values
  // where each ESC/motor will be a separate message
  private: void EscSensorCallback(EscSensorPtr &_escSensor);

  /// \brief Callback from the digital twin to recieve IMU values
  private: void ImuCallback(ImuPtr &_imu);

  private: void CalculateCallbackCount();
  private: void ResetCallbackCount();

  // \brief Calling GetLink from a model will not traverse nested models
  // until found, this function will find a link name from the 
  // entire model
  private: physics::LinkPtr FindLinkByName(physics::ModelPtr _model, std::string _linkName);

  /// \brief Block until the sensors are within a certain threshold. Useful for 
  // flushing remote sensors at the beinning of a new task.
  private: void FlushSensors();

  /// \brief Block until all the callbacks for the supported sneors
  // are recieved. 
  private: void WaitForSensorsThenSend();

  private: bool SensorEnabled(Sensors _sensor);

  private: std::string robotNamespace;

  /// \brief Main loop thread for the server
	private: boost::thread callbackLoopThread;

  /// \brief Pointer to the world
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

	/// \brief number of times ArduCotper skips update
	public: int connectionTimeoutCount;

	/// \brief number of times ArduCotper skips update
	/// before marking Quadcopter offline
	public: int connectionTimeoutMaxCount;

  /// \brief File path to the digital twin SDF
  private: std::string digitalTwinSDF;

  private: std::string cmdPubTopic;
  private: std::string imuSubTopic;
  private: std::string escSubTopic;
  private: transport::NodePtr nodeHandle;
  // Now define the communication channels with the digital twin
  // The architecure treats this world plugin as the flight controller
  // while all other aircraft components are now external and communicated
  // over protobufs
  private: transport::PublisherPtr cmdPub;

   // Subscribe to all possible sensors
  private: transport::SubscriberPtr imuSub;
  private: std::vector<transport::SubscriberPtr> escSub;
  private: cmd_msgs::msgs::MotorCommand cmdMsg;

  /// \brief Current callback count incremented as sensors are pbulished
  private: int sensorCallbackCount;
  private: int numSensorCallbacks;

  private: boost::condition_variable callbackCondition;

  private: gymfc::msgs::State state;
  private: gymfc::msgs::Action action;
  private: std::vector<Sensors> supportedSensors;

  private: int numActuators;
  private: sdf::SDFPtr sdfElement;
  private: std::string centerOfThrustReferenceLinkName; 
  private: ignition::math::Vector3d cot;
  private: sdf::ElementPtr modelElement;


  private: gazebo::physics::JointPtr ballJoint;
  private: ignition::math::Vector3d ballJointForce;
  };
}
#endif
