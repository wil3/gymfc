
#include <functional>
#include <fcntl.h>

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
#include "AircraftModelPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AircraftModelPlugin)

AircraftModelPlugin::AircraftModelPlugin()
{
}


/////////////////////////////////////////////////
AircraftModelPlugin::~AircraftModelPlugin()
{
}

/////////////////////////////////////////////////
void AircraftModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "AircraftModelPlugin _model pointer is null");
    GZ_ASSERT(_sdf, "AircraftModelPlugin _sdf pointer is null");
    gzdbg << " Model plugin loaded!\n";
 // this->dataPtr->model = _model;
    this->test = "Sample string";
    this->processSDF(_sdf);
}
void AircraftModelPlugin::OnUpdate()
{
}


void AircraftModelPlugin::processSDF(sdf::ElementPtr _sdf)
{

    /* 
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
*/

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
}
