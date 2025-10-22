// #include <gazebo/gazebo.hh>
// #include <gazebo/common/SystemPaths.hh>
// #include <gazebo/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>

namespace actor_plugin
{
    class ActorMovementPlugin : public gz::sim::System, public gz::sim::ISystemPreUpdate
    {
        private:
            gz::sim::Entity actorEntity;
            
        public:
            void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
            {
                if (_info.paused)
                {
                    return;
                }
                
                if (!this->actorEntity)
                {
                    auto actorEntities = _ecm.EntitiesByComponents(gz::sim::components::Actor());
                    if (!actorEntities.empty())
                    {
                        this->actorEntity = actorEntities.front();
                    }
                    return;
                }

                auto *poseComp = _ecm.Component<gz::sim::components::Pose>(this->actorEntity);
                if (!poseComp)
                {
                    gzerr << "Actor does not have a pose component" << std::endl;
                    return;
                }

                auto currentPose = poseComp->Data();
                double movementSpeed = 0.01;
                currentPose.Pos().X() += movementSpeed * _info.dt.count();

                _ecm.SetComponentData<gz::sim::components::Pose>(this->actorEntity, currentPose);
            }
    };

    GZ_ADD_PLUGIN(ActorMovementPlugin, gz::sim::System, ActorMovementPlugin::ISystemPreUpdate)
}