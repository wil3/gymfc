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

  // Force pause because we drive the simulation steps
  this->_world->SetPaused(TRUE);

  // Controller time control.
  this->lastControllerUpdateTime = 0;

  _callback_loop_thread = boost::thread( boost::bind( &QuadcopterWorldPlugin::loop_thread,this ) );
}

void QuadcopterWorldPlugin::processSDF(sdf::ElementPtr _sdf)
{

  // Missed update count before we declare arduCopterOnline status false
  getSdfParam<int>(_sdf, "connectionTimeoutMaxCount",
    this->connectionTimeoutMaxCount, 10);
  getSdfParam<double>(_sdf, "loopRate",
    this->loopRate, 100);

  // Optional parameters to start the aircraft off in a spin
  this->resetWithRandomAngularVelocity = FALSE;
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
    this->resetWorld = FALSE;
	while (1){

		std::lock_guard<std::mutex> lock(this->mutex);

		boost::this_thread::sleep(boost::posix_time::milliseconds(msPeriod));

		gazebo::common::Time curTime = _world->SimTime();
		
		//Try reading from the socket, if a packet is
		//available update the rotors
		bool received = this->ReceiveMotorCommand();

		if (received){
			if (this->resetWorld)
			{
				// Cant do a full reset of the RNG gets reset as well
				this->softReset();
				double error = 0.0001;// About 0.006 deg/s
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

				
  				if (this->_world->SimTime().Double() != 0.0){
					gzerr << "Reset sent but clock did not reset, at " << this->_world->SimTime().Double() << "\n";
				}
			}

			if (this->arduCopterOnline)
			{
				this->ApplyMotorForces((curTime - this->lastControllerUpdateTime).Double());
			}
			this->lastControllerUpdateTime = curTime;
			if (!this->resetWorld)
			{
				this->_world->Step(1);
			}
		} else {
			//gzerr << "Command not received t=" << this->_world->SimTime().Double() << "\n";
		}	
		if (this->arduCopterOnline)
		{
			this->SendState(received);
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
    waitMs = 5000;
  }
  else
  {
    // Otherwise skip quickly and do not set control force.
    waitMs = 1;
  }
  ssize_t recvSize = this->Recv(&pkt, sizeof(ServoPacket), waitMs);
  ssize_t expectedPktSize =
    sizeof(pkt.motorSpeed[0])*this->rotors.size()  + sizeof(pkt.resetWorld);
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
      if (pkt.resetWorld == 1) {
          this->resetWorld = TRUE;
      } else {
          this->resetWorld = FALSE;
      }
  }
  return commandProcessed;
}

/////////////////////////////////////////////////
void QuadcopterWorldPlugin::SendState(bool motorCommandProcessed) const
{
  // send_fdm
  fdmPacket pkt;

  pkt.timestamp = this->_world->SimTime().Double();

  if (motorCommandProcessed){
	  pkt.status_code = 1;
  } else {
	  pkt.status_code = 0;
  }

  ::sendto(this->handle,
           reinterpret_cast<raw_type *>(&pkt),
           sizeof(pkt), 0,
		   (struct sockaddr *)&this->remaddr, this->remaddrlen); 
   //        (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}




