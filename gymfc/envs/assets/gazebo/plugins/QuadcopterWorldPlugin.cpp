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

#include "QuadcopterWorldPlugin.hh"


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
    gzerr << "[QuadcopterWorldPlugin] Please specify a value for parameter ["
      << _name << "].\n";
  }
  return false;
}

GZ_REGISTER_WORLD_PLUGIN(QuadcopterWorldPlugin)

QuadcopterWorldPlugin::QuadcopterWorldPlugin() 
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

  this->arduCopterOnline = false;

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
QuadcopterWorldPlugin::~QuadcopterWorldPlugin()
{
	  // Sleeps (pauses the destructor) until the thread has finished
	  _callback_loop_thread.join();
}
void QuadcopterWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{

  this->_world = _world;
  this->processSDF(_sdf);

  const	std::string modelName = "testbench";
  // Force pause because we drive the simulation steps
  this->_world->SetPaused(TRUE);

  // Controller time control.
  this->lastControllerUpdateTime = 0;

  gzlog << "Quadcopter ready to fly. The force will be with you" << "\n";
  _callback_loop_thread = boost::thread( boost::bind( &QuadcopterWorldPlugin::loop_thread,this ) );
}

void QuadcopterWorldPlugin::processSDF(sdf::ElementPtr _sdf)
{
  // Get model name
  std::string modelName;
  getSdfParam<std::string>(_sdf, "modelName", modelName, "");
  this->_model = this->_world->ModelByName(modelName);
  //TODO Better error handling
  if (!this->_model){
	  gzerr << "Cant find model " << modelName << ". Aborting plugin.\n";
	  return;
  }

  // per rotor
  if (_sdf->HasElement("rotor"))
  {
    sdf::ElementPtr rotorSDF = _sdf->GetElement("rotor");

    while (rotorSDF)
    {
      Rotor rotor;
      if (rotorSDF->HasAttribute("id"))
      {
        rotor.id = rotorSDF->GetAttribute("id")->Get(rotor.id);
      }
      else
      {
        rotor.id = this->rotors.size();
        gzwarn << "id attribute not specified, use order parsed ["
               << rotor.id << "].\n";
      }

      if (rotorSDF->HasElement("jointName"))
      {
        rotor.jointName = rotorSDF->Get<std::string>("jointName");
      }
      else
      {
        gzerr << "Please specify a jointName,"
          << " where the rotor is attached.\n";
      }

      // Get the pointer to the joint.
      rotor.joint = this->_model->GetJoint(rotor.jointName);
      if (rotor.joint == nullptr)
      {
        gzerr << "Couldn't find specified joint ["
            << rotor.jointName << "]. This plugin will not run.\n";
        return;
      }

      if (rotorSDF->HasElement("turningDirection"))
      {
        std::string turningDirection = rotorSDF->Get<std::string>(
            "turningDirection");
        // special cases mimic from rotors_gazebo_plugins
        if (turningDirection == "cw")
          rotor.multiplier = -1;
        else if (turningDirection == "ccw")
          rotor.multiplier = 1;
        else
        {
          gzdbg << "not string, check turningDirection as float\n";
          rotor.multiplier = rotorSDF->Get<double>("turningDirection");
        }
      }
      else
      {
        rotor.multiplier = 1;
        gzerr << "Please specify a turning"
          << " direction multiplier ('cw' or 'ccw'). Default 'ccw'.\n";
      }

      getSdfParam<double>(rotorSDF, "rotorVelocitySlowdownSim",
          rotor.rotorVelocitySlowdownSim, 1);

      if (ignition::math::equal(rotor.rotorVelocitySlowdownSim, 0.0))
      {
        gzerr << "rotor for joint [" << rotor.jointName
              << "] rotorVelocitySlowdownSim is zero,"
              << " aborting plugin.\n";
        return;
      }

      getSdfParam<double>(rotorSDF, "frequencyCutoff",
          rotor.frequencyCutoff, rotor.frequencyCutoff);
      getSdfParam<double>(rotorSDF, "samplingRate",
          rotor.samplingRate, rotor.samplingRate);

      // use ignition::math::Filter
      rotor.velocityFilter.Fc(rotor.frequencyCutoff, rotor.samplingRate);

      // initialize filter to zero value
      rotor.velocityFilter.Set(0.0);

      // note to use this
      // rotorVelocityFiltered = velocityFilter.Process(rotorVelocityRaw);

      // Overload the PID parameters if they are available.
      double param;
      getSdfParam<double>(rotorSDF, "vel_p_gain", param, rotor.pid.GetPGain());
      rotor.pid.SetPGain(param);

      getSdfParam<double>(rotorSDF, "vel_i_gain", param, rotor.pid.GetIGain());
      rotor.pid.SetIGain(param);

      getSdfParam<double>(rotorSDF, "vel_d_gain", param,  rotor.pid.GetDGain());
      rotor.pid.SetDGain(param);

      getSdfParam<double>(rotorSDF, "vel_i_max", param, rotor.pid.GetIMax());
      rotor.pid.SetIMax(param);

      getSdfParam<double>(rotorSDF, "vel_i_min", param, rotor.pid.GetIMin());
      rotor.pid.SetIMin(param);

      getSdfParam<double>(rotorSDF, "vel_cmd_max", param,
          rotor.pid.GetCmdMax());
      rotor.pid.SetCmdMax(param);

      getSdfParam<double>(rotorSDF, "vel_cmd_min", param,
          rotor.pid.GetCmdMin());
      rotor.pid.SetCmdMin(param);

      // set pid initial command
      rotor.pid.SetCmd(0.0);

      this->rotors.push_back(rotor);
      rotorSDF = rotorSDF->GetNextElement("rotor");
    }
  }

  // Get sensors
  std::string imuName;
  getSdfParam<std::string>(_sdf, "imuName", imuName, "imu_sensor");
  std::string imuScopedName = this->_world->Name()
      + "::" + this->_model->GetScopedName()
      + "::" + imuName;
  this->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
    (sensors::SensorManager::Instance()->GetSensor(imuScopedName));

  if (!this->imuSensor)
  {
    gzerr << "imu_sensor [" << imuScopedName
          << "] not found, abort Quadcopter plugin.\n" << "\n";
    return;
  }
  // Missed update count before we declare arduCopterOnline status false
  getSdfParam<int>(_sdf, "connectionTimeoutMaxCount",
    this->connectionTimeoutMaxCount, 10);
  getSdfParam<double>(_sdf, "loopRate",
    this->loopRate, 100);

  // Optional parameters to start the aircraft off in a spin
  if (_sdf->HasElement("resetState"))
  {
		sdf::ElementPtr resetStateSDF = _sdf->GetElement("resetState");
		if (resetStateSDF->HasElement("angularVelocity")){
			sdf::ElementPtr angularVelocitySDF = resetStateSDF->GetElement("angularVelocity");
			if (angularVelocitySDF->HasElement("random")){
				sdf::ElementPtr randomSDF = angularVelocitySDF->GetElement("random");
				if (randomSDF->HasElement("seed")){
						this->resetWithRandomAngularVelocity = TRUE;
						ignition::math::Rand::Seed(randomSDF->Get<int>("seed"));
						this->rollLimit = randomSDF->Get<ignition::math::Vector2d>("roll");
						this->pitchLimit = randomSDF->Get<ignition::math::Vector2d>("pitch");
						this->yawLimit = randomSDF->Get<ignition::math::Vector2d>("yaw");
				}
			}
		}
  } 
}

void QuadcopterWorldPlugin::softReset(){
    this->_world->ResetTime();
    this->_world->ResetEntities(gazebo::physics::Base::BASE);
	this->_world->ResetPhysicsStates();
}

void QuadcopterWorldPlugin::loop_thread()
{
	double msPeriod = 1000.0/this->loopRate;

	while (1){

		std::lock_guard<std::mutex> lock(this->mutex);

		boost::this_thread::sleep(boost::posix_time::milliseconds(msPeriod));

		gazebo::common::Time curTime = _world->SimTime();
		
		//Try reading from the socket, if a packet is
		//available update the rotors
		bool received = this->ReceiveMotorCommand();

		if (received){
			// Theres is an issue even after a reset 
			// the IMU isnt reset and sending old values
			if (this->_world->Iterations() == 0)
			{
				// Cant do a full reset of the RNG gets reset as well
				this->softReset();
				double error = 0.02;// About 1 deg/s
				double spR = 0.0;
				double spP = 0.0;
				double spY = 0.0;
				//Flush stale IMU values
				while (1)
				{
  						ignition::math::Vector3d rates = this->imuSensor->AngularVelocity();
						// Pitch and Yaw are negative
						if (std::abs(spR - rates.X()) > error || std::abs(spP + rates.Y()) > error || std::abs(spY + rates.Z()) > error){
							//gzdbg << "Gyro r=" << rates.X() << " p=" << rates.Y() << " y=" << rates.Z() << "\n";
							this->_world->Step(1);
							if (!this->resetWithRandomAngularVelocity){//Only reset if trying to get to 0 rate 
								this->softReset();
							} 
							boost::this_thread::sleep(boost::posix_time::milliseconds(100));
						} else {
							  //gzdbg << "Target velocity reached! r=" << rates.X() << " p=" << rates.Y() << " y=" << rates.Z() << "\n";
							break;
						}
				}
				
			}
		} 
		if (this->arduCopterOnline)
		{
			this->ApplyMotorForces((curTime - this->lastControllerUpdateTime).Double());
		}
		this->lastControllerUpdateTime = curTime;
		if (received)
		{
			this->_world->Step(1);
		}
		if (this->arduCopterOnline)
		{
			this->SendState();
		}

	}
}

  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
bool QuadcopterWorldPlugin::Bind(const char *_address, const uint16_t _port)
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
  void QuadcopterWorldPlugin::MakeSockAddr(const char *_address, const uint16_t _port,
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
  ssize_t QuadcopterWorldPlugin::Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;
	//struct sockaddr_in remaddr;
	//socklen_t addrlen = sizeof(this->remaddr);
	this->remaddrlen = sizeof(this->remaddr);
	int recvlen;
	char buf[] = "hi";

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
void QuadcopterWorldPlugin::ResetPIDs()
{
  // Reset velocity PID for rotors
  for (size_t i = 0; i < this->rotors.size(); ++i)
  {
    this->rotors[i].cmd = 0;
    // this->rotors[i].pid.Reset();
  }
}

/////////////////////////////////////////////////
void QuadcopterWorldPlugin::ApplyMotorForces(const double _dt)
{
  // update velocity PID for rotors and apply force to joint
  for (size_t i = 0; i < this->rotors.size(); ++i)
  {
    double velTarget = this->rotors[i].multiplier *
      this->rotors[i].cmd /
      this->rotors[i].rotorVelocitySlowdownSim;
    double vel = this->rotors[i].joint->GetVelocity(0);
    double error = vel - velTarget;
    double force = this->rotors[i].pid.Update(error, _dt);
    this->rotors[i].joint->SetForce(0, force);
  }
}

/////////////////////////////////////////////////
bool QuadcopterWorldPlugin::ReceiveMotorCommand()
{
  // Added detection for whether ArduCopter is online or not.
  // If ArduCopter is detected (receive of fdm packet from someone),
  // then socket receive wait time is increased from 1ms to 1 sec
  // to accomodate network jitter.
  // If ArduCopter is not detected, receive call blocks for 1ms
  // on each call.
  // Once ArduCopter presence is detected, it takes this many
  // missed receives before declaring the FCS offline.

  bool commandProcessed = FALSE;
  ServoPacket pkt;
  int waitMs = 1;
  if (this->arduCopterOnline)
  {
    // increase timeout for receive once we detect a packet from
    // ArduCopter FCS.
    waitMs = 1000;
  }
  else
  {
    // Otherwise skip quickly and do not set control force.
    waitMs = 1;
  }
  ssize_t recvSize = this->Recv(&pkt, sizeof(ServoPacket), waitMs);
  ssize_t expectedPktSize =
    sizeof(pkt.motorSpeed[0])*this->rotors.size();//  + sizeof(pkt.seq);
  if ((recvSize == -1) || (recvSize < expectedPktSize))
  {
    // didn't receive a packet
    if (recvSize != -1)
    {
      gzerr << "received bit size (" << recvSize << ") to small,"
            << " controller expected size (" << expectedPktSize << ").\n";
    }

    gazebo::common::Time::NSleep(100);
    if (this->arduCopterOnline)
    {
      gzwarn << "Broken Quadcopter connection, count ["
             << this->connectionTimeoutCount
             << "/" << this->connectionTimeoutMaxCount
             << "]\n";
      if (++this->connectionTimeoutCount >
        this->connectionTimeoutMaxCount)
      {
        this->connectionTimeoutCount = 0;
        this->arduCopterOnline = false;
        gzwarn << "Broken Quadcopter connection, resetting motor control.\n";
        this->ResetPIDs();
      }
    }
	commandProcessed = FALSE;
  }
  else
  {
    if (!this->arduCopterOnline)
    {
      gzdbg << "Quadcopter controller online detected.\n";
      // made connection, set some flags
      this->connectionTimeoutCount = 0;
      this->arduCopterOnline = true;
    }

    //std::cout "Seq " << pkt.seq << "\n";

    // compute command based on requested motorSpeed
    for (unsigned i = 0; i < this->rotors.size(); ++i)
    {
      if (i < MAX_MOTORS)
      {
        // std::cout << i << ": " << pkt.motorSpeed[i] << "\n";
        this->rotors[i].cmd = this->rotors[i].maxRpm *
          pkt.motorSpeed[i];
      }
      else
      {
        gzerr << "too many motors, skipping [" << i
              << " > " << MAX_MOTORS << "].\n";
      }
	  commandProcessed = TRUE;
    }
  }
  return commandProcessed;
}

/////////////////////////////////////////////////
void QuadcopterWorldPlugin::SendState() const
{
  // send_fdm
  fdmPacket pkt;

  pkt.timestamp = this->_world->SimTime().Double();
  pkt.iter = static_cast<uint64_t> (this->_world->Iterations());

  for (size_t i = 0; i < this->rotors.size(); ++i)
  {
    pkt.motorVelocity[i] = this->rotors[i].joint->GetVelocity(0);
  }
  // asssumed that the imu orientation is:
  //   x forward
  //   y right
  //   z down

  // get linear acceleration in body frame
  ignition::math::Vector3d linearAccel =
    this->imuSensor->LinearAcceleration();

  // copy to pkt
  pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
  pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
  pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
  // gzerr << "lin accel [" << linearAccel << "]\n";

  // get angular velocity in body frame
  ignition::math::Vector3d angularVel =
    this->imuSensor->AngularVelocity();

  // copy to pkt
  pkt.imuAngularVelocityRPY[0] = angularVel.X();
  pkt.imuAngularVelocityRPY[1] = angularVel.Y();
  pkt.imuAngularVelocityRPY[2] = angularVel.Z();

 //if (pkt.iter == 0){
  //}


  // get inertial pose and velocity
  // position of the quadrotor in world frame
  // this position is used to calcualte bearing and distance
  // from starting location, then use that to update gps position.
  // The algorithm looks something like below (from ardupilot helper
  // libraries):
  //   bearing = to_degrees(atan2(position.y, position.x));
  //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
  //   (self.latitude, self.longitude) = util.gps_newpos(
  //    self.home_latitude, self.home_longitude, bearing, distance)
  // where xyz is in the NED directions.
  // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
  // around.
  // orientation of the quadrotor in world NED frame -
  // assuming the world NED frame has xyz mapped to NED,
  // imuLink is NED - z down

  // gazeboToNED brings us from gazebo model: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  ignition::math::Pose3d gazeboToNED(0, 0, 0, IGN_PI, 0, 0);

  // model world pose brings us to model, x-forward, y-left, z-up
  // adding gazeboToNED gets us to the x-forward, y-right, z-down
  ignition::math::Pose3d worldToModel = gazeboToNED +
    this->_model->WorldPose();

  // get transform from world NED to Model frame
  ignition::math::Pose3d NEDToModel = worldToModel - gazeboToNED;

  // gzerr << "ned to model [" << NEDToModel << "]\n";

  // N
  pkt.positionXYZ[0] = NEDToModel.Pos().X();

  // E
  pkt.positionXYZ[1] = NEDToModel.Pos().Y();

  // D
  pkt.positionXYZ[2] = NEDToModel.Pos().Z();

  // imuOrientationQuat is the rotation from world NED frame
  // to the quadrotor frame.
  pkt.imuOrientationQuat[0] = NEDToModel.Rot().W();
  pkt.imuOrientationQuat[1] = NEDToModel.Rot().X();
  pkt.imuOrientationQuat[2] = NEDToModel.Rot().Y();
  pkt.imuOrientationQuat[3] = NEDToModel.Rot().Z();

  // gzdbg << "imu [" << worldToModel.rot.GetAsEuler() << "]\n";
  // gzdbg << "ned [" << gazeboToNED.rot.GetAsEuler() << "]\n";
  // gzdbg << "rot [" << NEDToModel.rot.GetAsEuler() << "]\n";

  // Get NED velocity in body frame *
  // or...
  // Get model velocity in NED frame
  ignition::math::Vector3d velGazeboWorldFrame =
    this->_model->GetLink()->WorldLinearVel();
  ignition::math::Vector3d velNEDFrame =
    gazeboToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
  pkt.velocityXYZ[0] = velNEDFrame.X();
  pkt.velocityXYZ[1] = velNEDFrame.Y();
  pkt.velocityXYZ[2] = velNEDFrame.Z();

  //pkt.iter = 6;

  /*  
  char b[sizeof(pkt)];
  memcpy(b, &pkt, sizeof(pkt));


  gzdbg << " size of pkt " << sizeof(pkt) << "\n";
  ::sendto(this->handle,
		  b,
           sizeof(pkt), 0,
		   (struct sockaddr *)&this->remaddr, this->remaddrlen); 

  //struct sockaddr_in sockaddr;
  //this->MakeSockAddr("127.0.0.1", 9003, sockaddr);
  */
  ::sendto(this->handle,
           reinterpret_cast<raw_type *>(&pkt),
           sizeof(pkt), 0,
		   (struct sockaddr *)&this->remaddr, this->remaddrlen); 
   //        (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}

  /// \brief Constructor
Rotor::Rotor()
{
    // most of these coefficients are not used yet.
	/*
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;
	*/

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
}




