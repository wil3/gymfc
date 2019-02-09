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
#include <functional>
#include <fcntl.h>
#include <cstdlib>


#ifdef _WIN32
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  using raw_type = char;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  using raw_type = void;
#endif

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#include <mutex>
#include <string>
#include <vector>
#include <sdf/sdf.hh>
#include <ignition/math/Filter.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/Base.hh>

#include "FlightControllerPlugin.hh"

#include "MotorCommand.pb.h"
#include "EscSensor.pb.h"
#include "Imu.pb.h"

#include "State.pb.h"

using namespace gazebo;
/// \brief Obtains a parameter from sdf.
/// \param[in] _sdf Pointer to the sdf object.
/// \param[in] _name Name of the parameter.
/// \param[out] _param Param Variable to write the parameter to.
/// \param[in] _default_value Default value, if the parameter not available.
/// \param[in] _verbose If true, gzerror if the parameter is not available.
/// \return True if the parameter was found in _sdf, false otherwise.
template<class T>
bool getSdfParam(sdf::ElementPtr _sdf, const std::string &_name,
  T &_param, const T &_defaultValue, const bool &_verbose = false)
{
  if (_sdf->HasElement(_name))
  {
    _param = _sdf->GetElement(_name)->Get<T>();
    return true;
  }

  _param = _defaultValue;
  if (_verbose)
  {
    gzerr << "[FlightControllerPlugin] Please specify a value for parameter ["
      << _name << "].\n";
  }
  return false;
}
bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

GZ_REGISTER_WORLD_PLUGIN(FlightControllerPlugin)

boost::mutex g_CallbackMutex;

FlightControllerPlugin::FlightControllerPlugin() 
{
  // socket
  this->handle = socket(AF_INET, SOCK_DGRAM /*SOCK_STREAM*/, 0);
  #ifndef _WIN32
  // Windows does not support FD_CLOEXEC
  fcntl(this->handle, F_SETFD, FD_CLOEXEC);
  #endif
  int one = 1;
  setsockopt(this->handle, IPPROTO_TCP, TCP_NODELAY,
      reinterpret_cast<const char *>(&one), sizeof(one));

  // Default port can be read in from an environment variable
  // This allows multiple instances to be run
  int port = 9002;
  if(const char* env_p =  std::getenv("SITL_PORT")){
		  port = std::stoi(env_p);
  }
  gzdbg << "Binding on port " << port << "\n";
  if (!this->Bind("127.0.0.1", port))
  {
    gzerr << "failed to bind with 127.0.0.1:" << port <<", aborting plugin.\n";
    return;
  }

  if(const char* env_p =  std::getenv(DIGITAL_TWIN_SDF_ENV)){
    this->digitalTwinSDF = env_p;
  } else {
    gzerr << "Could not load digital twin model from environment variable " << DIGITAL_TWIN_SDF_ENV << "\n";
    return;
  }

  if(const char* env_p =  std::getenv(NUM_MOTORS_ENV)){
    this->numActuators = std::stoi(env_p);
  } else {
    gzerr << "Environment variable " << NUM_MOTORS_ENV << " not set.\n";
    return;
  }

  //this->statePkt.escTemperature.reserve(this->numActuators);

  this->aircraftOnline = false;

  this->connectionTimeoutCount = 0;

  setsockopt(this->handle, SOL_SOCKET, SO_REUSEADDR,
     reinterpret_cast<const char *>(&one), sizeof(one));

  #ifdef _WIN32
  u_long on = 1;
  ioctlsocket(this->handle, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
  #else
  fcntl(this->handle, F_SETFL,
      fcntl(this->handle, F_GETFL, 0) | O_NONBLOCK);
  #endif

}
FlightControllerPlugin::~FlightControllerPlugin()
{
    // Tear down the transporter
    gazebo::transport::fini();

	  // Sleeps (pauses the destructor) until the thread has finished
	  this->callbackLoopThread.join();
}


void FlightControllerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{

  this->world = _world;
  this->ProcessSDF(_sdf);

  // IMU + ESC Sensor x motors
  this->numCallbacks = 1 + this->numActuators;

  this->nodeHandle = transport::NodePtr(new transport::Node());
  this->nodeHandle->Init(this->robotNamespace);

  //Subscribe to all the sensors
  this->imuSub = this->nodeHandle->Subscribe<sensor_msgs::msgs::Imu>(this->imuSubTopic, &FlightControllerPlugin::ImuCallback, this);

  //Each defined motor will have a unique index, since they are indpendent they must come in 
  //as separate messages
  for (unsigned int i = 0; i < this->numActuators; i++)
  {
    this->escSub[i] = this->nodeHandle->Subscribe<sensor_msgs::msgs::EscSensor>(this->escSubTopic + "/" + std::to_string(i) , &FlightControllerPlugin::EscSensorCallback, this);
  }
  //this->escSub = this->nodeHandle->Subscribe<sensor_msgs::msgs::EscSensor>("/aircraft/sensor/esc/0", &FlightControllerPlugin::EscSensorCallback, this);

  this->cmdPub = this->nodeHandle->Advertise<cmd_msgs::msgs::MotorCommand>(this->cmdPubTopic);

  /* 
  this->updateConnection = event::Events::ConnectWorldUpdateEnd(
       std::bind(&FlightControllerPlugin::UpdateEnd, this));
       */
  // Force pause because we drive the simulation steps
  this->world->SetPaused(TRUE);

  // Controller time control.
  this->lastControllerUpdateTime = 0;

  this->callbackLoopThread = boost::thread( boost::bind( &FlightControllerPlugin::LoopThread, this) );
}

/*  
void FlightControllerPlugin::UpdateEnd()
{
  gzdbg << "End update"<< std::endl;

}
*/

void FlightControllerPlugin::EscSensorCallback(EscSensorPtr &_escSensor)
{
  boost::mutex::scoped_lock lock(g_CallbackMutex);

  //this->statePkt.escTemperature[_escSensor->id()] = _escSensor->temperature();
  gzdbg << "Receiving esc sensor " << _escSensor->id() << std::endl;
  this->callbackCount++;
  this->callbackCondition.notify_all();

}
void FlightControllerPlugin::ImuCallback(ImuPtr &_imu)
{
  boost::mutex::scoped_lock lock(g_CallbackMutex);
  gzdbg << "Received IMU" << std::endl;

  this->statePkt.imuAngularVelocityRPY[0] = _imu->angular_velocity().x();
  this->statePkt.imuAngularVelocityRPY[1] = _imu->angular_velocity().y();
  this->statePkt.imuAngularVelocityRPY[2] = _imu->angular_velocity().z();

  this->statePkt.imuOrientationQuat[0] = _imu->orientation().w();
  this->statePkt.imuOrientationQuat[1] = _imu->orientation().x();
  this->statePkt.imuOrientationQuat[2] = _imu->orientation().y();
  this->statePkt.imuOrientationQuat[3] = _imu->orientation().z();

  this->statePkt.imuLinearAccelerationXYZ[0] = _imu->linear_acceleration().x();
  this->statePkt.imuLinearAccelerationXYZ[1] = _imu->linear_acceleration().y();
  this->statePkt.imuLinearAccelerationXYZ[2] = _imu->linear_acceleration().z();

  this->callbackCount++;
  this->callbackCondition.notify_all();

}
void FlightControllerPlugin::ProcessSDF(sdf::ElementPtr _sdf)
{
  this->cmdPubTopic = kDefaultCmdPubTopic;
  if (_sdf->HasElement("commandPubTopic")){
      this->cmdPubTopic = _sdf->GetElement("commandPubTopic")->Get<std::string>();
  }
  this->imuSubTopic = kDefaultImuSubTopic;
  if (_sdf->HasElement("imuSubTopic")){
      this->imuSubTopic = _sdf->GetElement("imuSubTopic")->Get<std::string>();
  }
  this->escSubTopic = kDefaultEscSubTopic;
  if (_sdf->HasElement("escSubTopicPrefix")){
      this->escSubTopic = _sdf->GetElement("escSubTopicPrefix")->Get<std::string>();
  }


  if (_sdf->HasElement("robotNamespace"))
    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

  // Missed update count before we declare aircraftOnline status false
  getSdfParam<int>(_sdf, "connectionTimeoutMaxCount",
    this->connectionTimeoutMaxCount, 10);
  getSdfParam<double>(_sdf, "loopRate",
    this->loopRate, 100);

}

void FlightControllerPlugin::SoftReset()
{
  this->world->ResetTime();
  this->world->ResetEntities(gazebo::physics::Base::BASE);
	this->world->ResetPhysicsStates();
}


physics::LinkPtr FlightControllerPlugin::FindLinkByName(physics::ModelPtr _model, std::string _linkName)
{
  for (auto link : _model->GetLinks())
  {
    gzdbg << "Link name: " << link->GetName() << std::endl;
    if (hasEnding(link->GetName(), _linkName))
    {
      return link;
    }

  }
  return NULL;

}
void FlightControllerPlugin::LoadDigitalTwin()
{
  gzdbg << "[fc] Inserting digital twin from, " << this->digitalTwinSDF << ".\n";
   // Load the root digital twin sdf file
  const std::string sdfPath(this->digitalTwinSDF);

  sdf::SDFPtr sdfElement(new sdf::SDF());
  sdf::init(sdfElement);
  if (!sdf::readFile(sdfPath, sdfElement))
  {
    gzerr << sdfPath << " is not a valid SDF file!" << std::endl;
    return;
  }

  // start parsing model
  const sdf::ElementPtr rootElement = sdfElement->Root();
  if (!rootElement->HasElement("model"))
  {
    gzerr << sdfPath << " is not a model SDF file!" << std::endl;
    return;
  }
  const sdf::ElementPtr modelElement = rootElement->GetElement("model");
  const std::string modelName = modelElement->Get<std::string>("name");
  gzdbg << "Found " << modelName << " model!" << std::endl;

  unsigned int startModelCount = this->world->ModelCount();
  //this->world->InsertModelFile(sdfElement);
  
  this->world->InsertModelSDF(*sdfElement);

  // TODO Better way to do this?
  // It appears the inserted model is not available in the world
  // right away, maybe due to message passing?
  // Poll until its there
  while (1)
  {
    unsigned int modelCount = this->world->ModelCount();
    if (modelCount >= startModelCount + 1)
    {
      break;
    } else {
      gazebo::common::Time::MSleep(1000);
    }
  }
  
  gzdbg << "Num models=" << this->world->ModelCount() << std::endl;
  for (unsigned int i=0; i<this->world->ModelCount(); i++)
  {
    gzdbg << "Model " << i << ":" << this->world->ModelByIndex(i)->GetScopedName() << std::endl;
  }

  // Now get a pointer to the model
  physics::ModelPtr model = this->world->ModelByName(modelName);
  if (!model){
    gzerr << "Could not access model " << modelName <<" from world"<<std::endl;
    return;
  }

  //Find the base link to attached to the world
  gzdbg << " Before get link\n";
  physics::LinkPtr digitalTwinCoMLink = FindLinkByName(model, DIGITAL_TWIN_ATTACH_LINK);
  if (!link)
  {
    gzerr << "Could not find link '" << DIGITAL_TWIN_ATTACH_LINK <<" from model " << modelName <<std::endl;
    return;
  } 
  else
  {
      gzdbg << " Link Found\n";
  }

  
  physics::ModelPtr trainingRigModel = this->world->ModelByName(kTrainingRigModelName);
  if (!trainingRigModel){
    gzerr << "Could not find training rig"<<std::endl;
    return;
  }

  


  // Create the actual ball link, connecting the digital twin to the sim world
  physics::JointPtr joint = trainingRigModel->CreateJoint("ball_joint", "ball", trainingRigModel->GetLink("pivot"), digitalTwinCoMLink);
  if (!joint)
  {
    gzerr << "Could not create joint"<<std::endl;
    return;
  }
  /*  
  joint->SetAnchor(0, ignition::math::Vector3d(0, 0, 0));
  joint->SetAnchor(1, ignition::math::Vector3d(0, 0, 0));
  joint->SetAnchor(2, ignition::math::Vector3d(0, 0, 0));
  */
  joint->Init();
  
  // This is actually great because we've removed the ground plane so there is no possible collision
  gzdbg << "Ball joint created\n";
}
void FlightControllerPlugin::LoopThread()
{


  this->LoadDigitalTwin();
  this->SoftReset();


	double msPeriod = 1000.0/this->loopRate;
  this->resetWorld = FALSE;
	while (1){

		std::lock_guard<std::mutex> lock(this->mutex);

		boost::this_thread::sleep(boost::posix_time::milliseconds(msPeriod));

		gazebo::common::Time curTime = this->world->SimTime();
		
		//Try reading from the socket, if a packet is
		//available update the rotors
    {
      boost::mutex::scoped_lock lock2(g_CallbackMutex);
      this->callbackCount = -1 * (1 + this->numActuators);
    }
		bool received = this->ReceiveMotorCommand();

		if (received){
			if (this->resetWorld)
			{
				// Cant do a full reset of the RNG gets reset as well
				this->SoftReset();
				double error = 0.0001;// About 0.006 deg/s
				double spR = 0.0;
				double spP = 0.0;
				double spY = 0.0;
				//Flush stale IMU values
        /*
				while (1)
				{
  						ignition::math::Vector3d rates = this->imuSensor->AngularVelocity();
						// Pitch and Yaw are negative
						if (std::abs(spR - rates.X()) > error || std::abs(spP + rates.Y()) > error || std::abs(spY + rates.Z()) > error){
							//gzdbg << "Gyro r=" << rates.X() << " p=" << rates.Y() << " y=" << rates.Z() << "\n";
							this->world->Step(1);
							if (!this->resetWithRandomAngularVelocity){//Only reset if trying to get to 0 rate 
								this->softReset();
							} 
							boost::this_thread::sleep(boost::posix_time::milliseconds(100));
						} else {
							  //gzdbg << "Target velocity reached! r=" << rates.X() << " p=" << rates.Y() << " y=" << rates.Z() << "\n";
							break;
						}
				}*/

				
  				if (this->world->SimTime().Double() != 0.0){
					gzerr << "Reset sent but clock did not reset, at " << this->world->SimTime().Double() << "\n";
				}
			}

			if (this->aircraftOnline)
			{
        cmd_msgs::msgs::MotorCommand cmd;
        gzdbg << "Sending motor commands to digital twin" << std::endl;
        for (unsigned int i = 0; i < this->numActuators; i++)
        {
          //gzdbg << i << "=" << this->motor[i] << std::endl;
          cmd.add_motor(this->motor[i]);
        }
				this->cmdPub->Publish(cmd);
			}
			this->lastControllerUpdateTime = curTime;
			if (!this->resetWorld)
			{
				this->world->Step(1);
			}
		} else {
			//gzerr << "Command not received t=" << this->_world->SimTime().Double() << "\n";
		}	

    this->statePkt.timestamp = this->world->SimTime().Double();

    if (received)
    {
      this->statePkt.status_code = 1;
    } 
    else 
    {
      this->statePkt.status_code = 0;
    }

		if (this->aircraftOnline && !this->resetWorld)
		{
      boost::mutex::scoped_lock lock2(g_CallbackMutex);
      while (this->callbackCount < 0)
      {
        this->callbackCondition.wait(lock2);
      }
      gzdbg << "Sending state"<<std::endl;
			this->SendState(received);
		} 
    else 
    {
			this->SendState(received);
    }

	}
}

  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
bool FlightControllerPlugin::Bind(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->handle, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->handle, 0);
      #ifdef _WIN32
      closesocket(this->handle);
      #else
      close(this->handle);
      #endif
      return false;
    }
    return true;
  }

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.
  void FlightControllerPlugin::MakeSockAddr(const char *_address, const uint16_t _port,
    struct sockaddr_in &_sockaddr)
  {
    memset(&_sockaddr, 0, sizeof(_sockaddr));

    #ifdef HAVE_SOCK_SIN_LEN
      _sockaddr.sin_len = sizeof(_sockaddr);
    #endif

    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
  }

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  ssize_t FlightControllerPlugin::Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;
	//struct sockaddr_in remaddr;
	//socklen_t addrlen = sizeof(this->remaddr);
	this->remaddrlen = sizeof(this->remaddr);
	int recvlen;

    FD_ZERO(&fds);
    FD_SET(this->handle, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(this->handle+1, &fds, NULL, NULL, &tv) != 1)
    {
        return -1;
    }

    #ifdef _WIN32
    return recv(this->handle, reinterpret_cast<char *>(_buf), _size, 0);
    #else
 	//return recv(this->handle, _buf, _size, 0);
    recvlen = recvfrom(this->handle, _buf, _size, 0, (struct sockaddr *)&this->remaddr, &this->remaddrlen);
	//:: sendto(this->handle, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, addrlen);
	return recvlen;
    #endif
  }


/////////////////////////////////////////////////
bool FlightControllerPlugin::ReceiveMotorCommand()
{

  bool commandProcessed = FALSE;
  ServoPacket pkt;
  int waitMs = 1;
  if (this->aircraftOnline)
  {
    // increase timeout for receive once we detect a packet from
    // ArduCopter FCS.
    waitMs = 5000;
  }
  else
  {
    // Otherwise skip quickly and do not set control force.
    waitMs = 1;
  }
  ssize_t recvSize = this->Recv(&pkt, sizeof(ServoPacket), waitMs);
  ssize_t expectedPktSize =
    sizeof(pkt.motor[0])*this->numActuators  + sizeof(pkt.resetWorld);
  if ((recvSize == -1) || (recvSize < expectedPktSize))
  {
    // didn't receive a packet
    if (recvSize != -1)
    {
      gzerr << "received bit size (" << recvSize << ") to small,"
            << " controller expected size (" << expectedPktSize << ").\n";
    }
	
	if (recvSize < expectedPktSize){
		//gzwarn << "Received size " << recvSize << " less than the expected size of " << expectedPktSize << "\n";
	}

    gazebo::common::Time::NSleep(100);
    if (this->aircraftOnline)
    {
      gzwarn << "Broken Quadcopter connection, count ["
             << this->connectionTimeoutCount
             << "/" << this->connectionTimeoutMaxCount
             << "]\n";
      if (++this->connectionTimeoutCount >
        this->connectionTimeoutMaxCount)
      {
        this->connectionTimeoutCount = 0;
        this->aircraftOnline = false;
        gzwarn << "Broken Quadcopter connection, resetting motor control.\n";
      }
    }
	commandProcessed = FALSE;
  }
  else
  {
    if (!this->aircraftOnline)
    {
      gzdbg << "Aircraft controller online detected.\n";
      // made connection, set some flags
      this->connectionTimeoutCount = 0;
      this->aircraftOnline = true;
    }

    //std::cout "Seq " << pkt.seq << "\n";

    // compute command based on requested motorSpeed
    gzdbg << "[fc] Received motor command: ";
    for (unsigned int i = 0; i < this->numActuators; i++)
    {
      if (i < MAX_MOTORS)
      {
          std::cout << pkt.motor[i] << " ";
         this->motor[i] = pkt.motor[i];
      }
      else
      {
        gzerr << "too many motors, skipping [" << i
              << " > " << MAX_MOTORS << "].\n";
      }
    }
	  commandProcessed = TRUE;
    std::cout << std::endl;

      if (pkt.resetWorld == 1) {
          this->resetWorld = TRUE;
      } else {
          this->resetWorld = FALSE;
      }
  }
  return commandProcessed;
}

/////////////////////////////////////////////////
void FlightControllerPlugin::SendState(bool motorCommandProcessed) const
{
  /* 
  StatePacket pkt;
  pkt.timestamp = 0.0;
  pkt.imuAngularVelocityRPY[0] = 0.0;
  pkt.imuAngularVelocityRPY[1] = 0.0;
  pkt.imuAngularVelocityRPY[2] = 0.0;

  pkt.imuLinearAccelerationXYZ[0] = 0.0;
  pkt.imuLinearAccelerationXYZ[1] = 0.0;
  pkt.imuLinearAccelerationXYZ[2] = 0.0;

  pkt.imuOrientationQuat[0] = 0.0;
  pkt.imuOrientationQuat[1] = 0.0;
  pkt.imuOrientationQuat[2] = 0.0;
  pkt.imuOrientationQuat[3] = 0.0;

  pkt.velocityXYZ[0] = 0.0;
  pkt.velocityXYZ[1] = 0.0;
  pkt.velocityXYZ[2] = 0.0;

  pkt.positionXYZ[0] = 0.0;
  pkt.positionXYZ[1] = 0.0;
  pkt.positionXYZ[2] = 0.0;
  pkt.motorVelocity[0] = 0.0;
  pkt.motorVelocity[1] = 0.0;
  pkt.motorVelocity[2] = 0.0;
  pkt.motorVelocity[3] = 0.0;

  pkt.status_code = 1;

  pkt.escTemperature.reserve(4);
  pkt.escTemperature[0] = 1;
  pkt.escTemperature[1] = 2;
  pkt.escTemperature[2] = 2;
  pkt.escTemperature[3] = 4;
  */

  GOOGLE_PROTOBUF_VERIFY_VERSION;

  gymfc::msgs::State state;
  state.set_sim_time(7.0f);
  state.set_test_string("hello world");
  std::string buf;
  state.SerializeToString(&buf);

  gzdbg << " Buf start" << buf.data() << "end" << std::endl;
  gzdbg << " Buf length " << buf.size() << std::endl;

  /*  
  ::sendto(this->handle,
           reinterpret_cast<raw_type *>(&pkt),
           sizeof(pkt), 0,
		   (struct sockaddr *)&this->remaddr, this->remaddrlen); 
       */
  ::sendto(this->handle,
           buf.data(),
           buf.size(), 0,
		   (struct sockaddr *)&this->remaddr, this->remaddrlen); 
}




