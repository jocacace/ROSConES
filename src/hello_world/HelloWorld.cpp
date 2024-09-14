#include <string>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>

namespace hello_world
{

  class HelloWorld:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
  {
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
  };
}

IGNITION_ADD_PLUGIN(
    hello_world::HelloWorld,
    gz::sim::System,
    hello_world::HelloWorld::ISystemPostUpdate)

using namespace hello_world;


void HelloWorld::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";

  ignmsg << msg << std::endl;
}



