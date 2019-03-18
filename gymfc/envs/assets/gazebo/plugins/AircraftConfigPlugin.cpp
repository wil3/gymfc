#include <gazebo/gazebo.hh>
/**
 * This is a dummy plugin to be able to dynamically load in 
 * details about the aircraft model through the SDF.
 */

namespace gazebo
{
  class AircraftConfigPlugin : public ModelPlugin
  {
    public: AircraftConfigPlugin() : ModelPlugin()
            {}

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
            {}
  };
  GZ_REGISTER_MODEL_PLUGIN(AircraftConfigPlugin)
}
