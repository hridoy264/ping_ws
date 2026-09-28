// SPDX-License-Identifier: Apache-2.0
// Gazebo Fortress system for a deliberately simplified launcher boundary model.
// Balls begin beyond the wheels: this does not simulate hose feeding, deformation,
// wheel/ball contact, slip, spin transfer, or the Magnus force.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/uint32.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

namespace pingpong
{
namespace sim = ignition::gazebo;
namespace math = ignition::math;
namespace components = ignition::gazebo::components;

class BallLauncher final : public sim::System,
    public sim::ISystemConfigure, public sim::ISystemPreUpdate
{
public:
  void Configure(const sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      sim::EntityComponentManager &_ecm, sim::EventManager &_events) override
  {
    this->model = sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      ignerr << "BallLauncher must be attached to a model.\n";
      return;
    }

    const auto cfg = _sdf->Clone();
    this->wheelRadius = cfg->Get<double>("wheel_radius", 0.0325).first;
    this->efficiency = cfg->Get<double>("efficiency", 0.75).first;
    this->ballRadius = cfg->Get<double>("ball_radius", 0.02).first;
    this->ballMass = cfg->Get<double>("ball_mass", 0.0027).first;
    this->ballLifetime = cfg->Get<double>("ball_lifetime", 15.0).first;
    this->minShotInterval = cfg->Get<double>("min_shot_interval", 0.25).first;
    this->dragCoefficient = cfg->Get<double>("drag_coefficient", 0.47).first;
    this->airDensity = cfg->Get<double>("air_density", 1.225).first;
    this->muzzlePose = cfg->Get<math::Pose3d>("muzzle_pose",
        math::Pose3d(0.113, 0, 0, 0, 0, 0)).first;
    const int maxBalls = cfg->Get<int>("max_balls", 30).first;
    if (!Positive(this->wheelRadius) || !Positive(this->efficiency) ||
        this->efficiency > 1.0 || !Positive(this->ballRadius) ||
        !Positive(this->ballMass) || !Positive(this->ballLifetime) ||
        !Nonnegative(this->minShotInterval) ||
        !Nonnegative(this->dragCoefficient) || !Nonnegative(this->airDensity) ||
        !this->muzzlePose.Pos().IsFinite() ||
        !this->muzzlePose.Rot().IsFinite() || maxBalls < 1 || maxBalls > 1000)
    {
      ignerr << "BallLauncher has invalid physical parameters; max_balls must "
             << "be 1..1000 and efficiency must be in (0,1].\n";
      return;
    }
    this->maxBalls = static_cast<std::size_t>(maxBalls);

    const auto headName = cfg->Get<std::string>("head_link", "head_link").first;
    this->head = this->model.LinkByName(_ecm, headName);
    if (this->head == sim::kNullEntity)
    {
      ignerr << "BallLauncher cannot find head link " << headName << ".\n";
      return;
    }
    std::vector<std::string> jointNames;
    if (cfg->HasElement("wheel_joint"))
    {
      auto element = cfg->GetElement("wheel_joint");
      while (element)
      {
        jointNames.push_back(element->Get<std::string>());
        element = element->GetNextElement("wheel_joint");
      }
    }
    else
    {
      jointNames = {"wheel_1_joint", "wheel_2_joint", "wheel_3_joint"};
    }
    if (jointNames.size() != 3)
    {
      ignerr << "BallLauncher requires exactly three wheel_joint entries.\n";
      return;
    }
    for (const auto &name : jointNames)
    {
      const auto joint = this->model.JointByName(_ecm, name);
      if (joint == sim::kNullEntity)
      {
        ignerr << "BallLauncher cannot find wheel joint " << name << ".\n";
        return;
      }
      this->wheelJoints.push_back(joint);
      // Physics only reports joint velocities when a consumer requests them.
      if (!_ecm.Component<components::JointVelocity>(joint))
        _ecm.CreateComponent(joint, components::JointVelocity());
    }
    sim::Link(this->head).EnableVelocityChecks(_ecm);

    // Balls must be children of the world, not nested in the fixed launcher.
    auto ancestor = _entity;
    while (ancestor != sim::kNullEntity)
    {
      if (_ecm.Component<components::World>(ancestor))
      {
        this->world = ancestor;
        break;
      }
      const auto parent = _ecm.Component<components::ParentEntity>(ancestor);
      ancestor = parent ? parent->Data() : sim::kNullEntity;
    }
    // In Fortress, model plugins can configure before the model's ParentEntity
    // component is attached. Each server ECM represents one world, which already
    // exists at this point, so resolve it directly as a fallback.
    if (this->world == sim::kNullEntity)
      this->world = _ecm.EntityByComponents(components::World());
    if (this->world == sim::kNullEntity)
    {
      ignerr << "BallLauncher cannot resolve its world.\n";
      return;
    }
    this->creator = std::make_unique<sim::SdfEntityCreator>(_ecm, _events);
    this->fireTopic = cfg->Get<std::string>("fire_topic", "/pingpong/fire").first;
    this->countPub = this->node.Advertise<ignition::msgs::UInt32>(
        cfg->Get<std::string>("shot_count_topic", "/pingpong/shot_count").first);
    this->statusPub = this->node.Advertise<ignition::msgs::StringMsg>(
        cfg->Get<std::string>("status_topic", "/pingpong/ball_status").first);
    if (!this->countPub || !this->statusPub ||
        !this->node.Subscribe(this->fireTopic, &BallLauncher::OnFire, this))
    {
      ignerr << "BallLauncher could not create transport topics.\n";
      return;
    }
    this->configured = true;
    this->PublishCount();
    this->Status("ready: simplified launch model; spin and contact transfer disabled");
  }

  void PreUpdate(const sim::UpdateInfo &_info,
      sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;
    const double now = std::chrono::duration<double>(_info.simTime).count();
    if (now < this->lastSimTime)
      this->Reset(_ecm);
    this->lastSimTime = now;
    if (now - this->lastCountTime >= 1.0)
    {
      this->PublishCount();
      this->lastCountTime = now;
    }
    if (_info.paused)
      return;

    this->UpdateBalls(now, _ecm);
    if (!this->fireRequested.exchange(false))
      return;
    if (now - this->lastShotTime < this->minShotInterval)
    {
      this->Status("rejected: shot interval has not elapsed");
      return;
    }

    double meanRate = 0.0;
    for (const auto joint : this->wheelJoints)
    {
      const auto velocity = _ecm.Component<components::JointVelocity>(joint);
      if (!velocity || velocity->Data().empty() ||
          !std::isfinite(velocity->Data()[0]) || velocity->Data()[0] <= 0.1)
      {
        this->Status("rejected: all three measured wheel rates must exceed 0.1 rad/s");
        return;
      }
      meanRate += velocity->Data()[0] / 3.0;
    }
    const double speed = this->efficiency * this->wheelRadius * meanRate;
    const math::Pose3d pose = sim::worldPose(this->head, _ecm) * this->muzzlePose;
    // Include head translation and its tangential velocity at the muzzle.
    const auto inheritedVelocity = sim::Link(this->head).WorldLinearVelocity(
        _ecm, this->muzzlePose.Pos());
    const auto velocity = pose.Rot().RotateVector(math::Vector3d(speed, 0, 0)) +
        inheritedVelocity.value_or(math::Vector3d::Zero);
    if (!velocity.IsFinite() || !pose.Pos().IsFinite())
    {
      this->Status("rejected: invalid head state");
      return;
    }
    if (this->SpawnBall(now, pose, velocity, _ecm))
      this->lastShotTime = now;
  }

private:
  enum class Stage {Spawned, VelocitySet, Flying};
  struct Ball
  {
    sim::Entity model;
    sim::Entity link;
    math::Vector3d initialWorldVelocity;
    double born;
    Stage stage{Stage::Spawned};
  };

  static bool Positive(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  static bool Nonnegative(double value)
  {
    return std::isfinite(value) && value >= 0.0;
  }

  void OnFire(const ignition::msgs::Boolean &_msg)
  {
    // One true message requests one shot; false does nothing. Requests received
    // in the same physics tick coalesce. The transport thread never touches ECM.
    if (_msg.data())
      this->fireRequested.store(true);
  }

  void PublishCount()
  {
    ignition::msgs::UInt32 message;
    message.set_data(this->shotCount);
    this->countPub.Publish(message);
  }

  void Status(const std::string &_text)
  {
    ignition::msgs::StringMsg message;
    message.set_data(_text);
    this->statusPub.Publish(message);
  }

  bool SpawnBall(double _now, const math::Pose3d &_pose,
      const math::Vector3d &_velocity, sim::EntityComponentManager &_ecm)
  {
    // Table-tennis balls are approximated as thin hollow spheres: I = 2mr²/3.
    const double inertia = (2.0 / 3.0) * this->ballMass *
        this->ballRadius * this->ballRadius;
    const std::string name = "pingpong_ball_" +
        std::to_string(this->model.Entity()) + "_" + std::to_string(++this->serial);
    std::ostringstream xml;
    xml << std::setprecision(17)
        << "<sdf version='1.8'><model name='" << name << "'><pose>"
        << _pose << "</pose><link name='ball_link'><inertial><mass>"
        << this->ballMass << "</mass><inertia><ixx>" << inertia
        << "</ixx><iyy>" << inertia << "</iyy><izz>" << inertia
        << "</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
        << "<collision name='ball_collision'><geometry><sphere><radius>"
        << this->ballRadius << "</radius></sphere></geometry><surface>"
        << "<friction><ode><mu>0.3</mu><mu2>0.3</mu2></ode></friction>"
        << "<bounce><restitution_coefficient>0.8</restitution_coefficient>"
        << "<threshold>0.01</threshold></bounce></surface></collision>"
        << "<visual name='ball_visual'><geometry><sphere><radius>"
        << this->ballRadius << "</radius></sphere></geometry><material>"
        << "<ambient>1 0.45 0.04 1</ambient><diffuse>1 0.45 0.04 1</diffuse>"
        << "</material></visual></link></model></sdf>";
    sdf::Root root;
    const auto errors = root.LoadSdfString(xml.str());
    if (!errors.empty() || !root.Model())
    {
      ignerr << "BallLauncher failed to parse its generated ball SDF.\n";
      for (const auto &error : errors)
        ignerr << error.Message() << '\n';
      this->Status("failed: ball SDF creation");
      return false;
    }
    const auto entity = this->creator->CreateEntities(root.Model());
    if (entity == sim::kNullEntity)
    {
      this->Status("failed: ball entity creation");
      return false;
    }
    this->creator->SetParent(entity, this->world);
    const auto link = sim::Model(entity).LinkByName(_ecm, "ball_link");
    if (link == sim::kNullEntity)
    {
      this->creator->RequestRemoveEntity(entity);
      this->Status("failed: missing spawned ball link");
      return false;
    }
    sim::Link(link).EnableVelocityChecks(_ecm);
    while (this->balls.size() >= this->maxBalls)
    {
      this->creator->RequestRemoveEntity(this->balls.front().model);
      this->balls.pop_front();
    }
    this->balls.push_back({entity, link, _velocity, _now, Stage::Spawned});
    return true;
  }

  void UpdateBalls(double _now, sim::EntityComponentManager &_ecm)
  {
    for (auto it = this->balls.begin(); it != this->balls.end(); )
    {
      sim::Link link(it->link);
      if (!link.Valid(_ecm))
      {
        it = this->balls.erase(it);
        continue;
      }
      if (_now - it->born > this->ballLifetime)
      {
        this->creator->RequestRemoveEntity(it->model);
        it = this->balls.erase(it);
        continue;
      }
      if (it->stage == Stage::Spawned)
      {
        // Let the Physics system construct the newly spawned body first. The
        // Fortress setters take LINK-frame vectors, not world-frame vectors.
        const auto orientation = sim::worldPose(it->link, _ecm).Rot();
        link.SetLinearVelocity(_ecm,
            orientation.RotateVectorReverse(it->initialWorldVelocity));
        link.SetAngularVelocity(_ecm, math::Vector3d::Zero);
        it->stage = Stage::VelocitySet;
      }
      else
      {
        if (it->stage == Stage::VelocitySet)
        {
          // Fortress retains these command components and zeros their data after
          // every step. Remove them or the next step would stop the ball.
          _ecm.RemoveComponent<components::LinearVelocityCmd>(it->link);
          _ecm.RemoveComponent<components::AngularVelocityCmd>(it->link);
          const auto measured = link.WorldLinearVelocity(_ecm);
          const double expectedSpeed = it->initialWorldVelocity.Length();
          if (!measured || !measured->IsFinite() || expectedSpeed < 1e-9 ||
              measured->Dot(it->initialWorldVelocity) <
              0.5 * expectedSpeed * expectedSpeed)
          {
            this->Status("failed: physics did not apply ball launch velocity");
            this->creator->RequestRemoveEntity(it->model);
            it = this->balls.erase(it);
            continue;
          }
          it->stage = Stage::Flying;
          ++this->shotCount;
          this->PublishCount();
          this->Status("launched: " + std::to_string(this->shotCount));
        }
        // Quadratic drag in still air. Gravity and collision response remain
        // with Gazebo's physics engine; no trajectory teleporting is performed.
        if (this->dragCoefficient > 0.0 && this->airDensity > 0.0)
        {
          const auto velocity = link.WorldLinearVelocity(_ecm);
          if (velocity && velocity->IsFinite())
          {
            constexpr double pi = 3.14159265358979323846;
            const double drag = -0.5 * this->airDensity * this->dragCoefficient *
                pi * this->ballRadius * this->ballRadius * velocity->Length();
            link.AddWorldForce(_ecm, drag * (*velocity));
          }
        }
      }
      ++it;
    }
  }

  void Reset(sim::EntityComponentManager &_ecm)
  {
    // Fortress has no ISystemReset interface. A simulation-time rewind is the
    // available reset signal. A full server restart also starts fresh state.
    for (const auto &ball : this->balls)
    {
      if (sim::Link(ball.link).Valid(_ecm))
        this->creator->RequestRemoveEntity(ball.model);
    }
    this->balls.clear();
    this->fireRequested.store(false);
    this->lastShotTime = -std::numeric_limits<double>::infinity();
    this->lastCountTime = 0.0;
    this->shotCount = 0;
    this->PublishCount();
    this->Status("reset: balls removed and shot count cleared");
  }

  sim::Model model;
  sim::Entity world{sim::kNullEntity};
  sim::Entity head{sim::kNullEntity};
  std::vector<sim::Entity> wheelJoints;
  std::unique_ptr<sim::SdfEntityCreator> creator;
  std::deque<Ball> balls;
  math::Pose3d muzzlePose;
  double wheelRadius{0.0325};
  double efficiency{0.75};
  double ballRadius{0.02};
  double ballMass{0.0027};
  double ballLifetime{15.0};
  double minShotInterval{0.25};
  double dragCoefficient{0.47};
  double airDensity{1.225};
  double lastShotTime{-std::numeric_limits<double>::infinity()};
  double lastSimTime{0.0};
  double lastCountTime{0.0};
  std::size_t maxBalls{30};
  std::uint64_t serial{0};
  std::uint32_t shotCount{0};
  bool configured{false};
  std::string fireTopic;
  std::atomic<bool> fireRequested{false};
  ignition::transport::Node::Publisher countPub;
  ignition::transport::Node::Publisher statusPub;
  // Destroy Node before callback state so subscription teardown cannot race it.
  ignition::transport::Node node;
};
}  // namespace pingpong

IGNITION_ADD_PLUGIN(pingpong::BallLauncher,
    ignition::gazebo::System,
    pingpong::BallLauncher::ISystemConfigure,
    pingpong::BallLauncher::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(pingpong::BallLauncher, "pingpong::BallLauncher")
