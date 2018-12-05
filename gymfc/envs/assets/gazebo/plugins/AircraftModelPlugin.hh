
#ifndef GAZEBO_PLUGINS_GYMFC_MODEL_PLUGIN_HH_
#define GAZEBO_PLUGINS_GYMFC_MODEL_PLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  // Forward declare private data class
  class AircraftPluginPrivate;

  class GAZEBO_VISIBLE AircraftModelPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: AircraftModelPlugin();

    /// \brief Destructor.
    public: ~AircraftModelPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    private: void OnUpdate();
  };
}
#endif
