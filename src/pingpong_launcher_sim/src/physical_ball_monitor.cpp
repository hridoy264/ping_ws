// SPDX-License-Identifier: Apache-2.0
// Gazebo Fortress observer: reads persistent physical ball poses only.
// No entities, poses, velocities, forces, or physics components are written.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_set>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/uint32.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

namespace pingpong
{
namespace sim = ignition::gazebo;
namespace math = ignition::math;
namespace components = ignition::gazebo::components;

class PhysicalBallMonitor final : public sim::System,
    public sim::ISystemConfigure, public sim::ISystemPostUpdate
{
public:
  void Configure(const sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      sim::EntityComponentManager &_ecm, sim::EventManager &) override
  {
    this->launcher = sim::Model(_entity);
    if (!this->launcher.Valid(_ecm) || !_sdf)
    {
      ignerr << "PhysicalBallMonitor must be attached to a launcher model.\n";
      return;
    }
    const auto cfg = _sdf->Clone();
    // These datums belong to the new CAD model: guessing them can count a
    // crossing through a plane that is unrelated to the real mouth.
    if (!cfg->HasElement("head_link") || !cfg->HasElement("muzzle_pose") ||
        !cfg->HasElement("aperture_radius"))
    {
      ignerr << "PhysicalBallMonitor requires head_link, muzzle_pose, and "
             << "aperture_radius from the physical model.\n";
      return;
    }
    this->headName = cfg->Get<std::string>("head_link");
    this->muzzlePose = cfg->Get<math::Pose3d>("muzzle_pose");
    this->apertureRadius = cfg->Get<double>("aperture_radius");
    this->ballPrefix = cfg->Get<std::string>("ball_name_prefix", "inventory_ball_").first;
    this->ballLinkName = cfg->Get<std::string>("ball_link", "ball_link").first;
    this->planeX = cfg->Get<double>("muzzle_plane_x", 0.0).first;
    this->ballRadius = cfg->Get<double>("ball_radius", 0.02).first;
    this->armingDistance = cfg->Get<double>("arming_distance", 0.002).first;
    this->minForwardSpeed = cfg->Get<double>("min_forward_speed", 0.01).first;
    this->maxBallSpeed = cfg->Get<double>("max_ball_speed", 100.0).first;
    this->maxSampleGap = cfg->Get<double>("max_sample_gap", 0.05).first;
    this->publishRate = cfg->Get<double>("publish_rate", 5.0).first;
    const int expected = cfg->Get<int>("expected_ball_count", 100).first;
    this->contactTopic = cfg->Get<std::string>("contact_topic", "").first;
    if (this->headName.empty() || this->ballPrefix.empty() ||
        this->ballLinkName.empty() || !std::isfinite(this->planeX) ||
        !this->muzzlePose.Pos().IsFinite() || !this->muzzlePose.Rot().IsFinite() ||
        !Positive(this->ballRadius) || !Positive(this->apertureRadius) ||
        this->apertureRadius <= this->ballRadius || !Positive(this->armingDistance) ||
        !Nonnegative(this->minForwardSpeed) || !Positive(this->maxBallSpeed) ||
        this->maxBallSpeed <= this->minForwardSpeed || !Positive(this->maxSampleGap) ||
        !Positive(this->publishRate) || expected < 1 || expected > 100000)
    {
      ignerr << "PhysicalBallMonitor has invalid observation parameters.\n";
      return;
    }
    this->expectedBallCount = static_cast<std::size_t>(expected);
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
    if (this->world == sim::kNullEntity)
      this->world = _ecm.EntityByComponents(components::World());
    if (this->world == sim::kNullEntity)
    {
      ignerr << "PhysicalBallMonitor cannot resolve its world.\n";
      return;
    }
    this->head = this->launcher.LinkByName(_ecm, this->headName);
    this->countPub = this->node.Advertise<ignition::msgs::UInt32>(
        cfg->Get<std::string>("shot_count_topic", "/pingpong/physical_shot_count").first);
    this->inventoryPub = this->node.Advertise<ignition::msgs::StringMsg>(
        cfg->Get<std::string>("inventory_topic", "/pingpong/physical_ball_inventory").first);
    if (!this->countPub || !this->inventoryPub ||
        !this->node.Subscribe(cfg->Get<std::string>("reset_topic",
            "/pingpong/physical_monitor/reset").first,
            &PhysicalBallMonitor::OnReset, this) ||
        (!this->contactTopic.empty() && !this->node.Subscribe(
            this->contactTopic, &PhysicalBallMonitor::OnContacts, this)))
    {
      ignerr << "PhysicalBallMonitor could not create its transport interfaces.\n";
      return;
    }
    this->configured = true;
    this->PublishCount();
  }

  void PostUpdate(const sim::UpdateInfo &_info,
      const sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;
    const double now = std::chrono::duration<double>(_info.simTime).count();
    const bool rewound = this->haveSimTime && now < this->lastSimTime;
    const bool reset = this->resetRequested.exchange(false) || rewound;
    if (reset)
      this->ResetAccounting();
    this->haveSimTime = true;
    this->lastSimTime = now;
    if (this->head == sim::kNullEntity || !_ecm.HasEntity(this->head))
      this->head = this->launcher.LinkByName(_ecm, this->headName);
    this->headAvailable = this->head != sim::kNullEntity &&
        _ecm.HasEntity(this->head) && _ecm.Component<components::Pose>(this->head);
    math::Pose3d mouth;
    if (this->headAvailable)
    {
      mouth = sim::worldPose(this->head, _ecm) * this->muzzlePose;
      this->headAvailable = mouth.Pos().IsFinite() && mouth.Rot().IsFinite();
    }
    for (auto &entry : this->balls)
    {
      entry.second.present = false;
      entry.second.poseAvailable = false;
      entry.second.localPoseAvailable = false;
      entry.second.phase = "missing";
    }

    // Inventory world-child MODEL identities, not visuals or collision names.
    // Balls remain in ECM after launch and are never removed by this observer.
    std::map<std::string, sim::Entity> discovered;
    std::unordered_set<std::string> duplicates;
    _ecm.Each<components::Model, components::Name, components::ParentEntity>(
        [&](const sim::Entity &_ballEntity, const components::Model *,
            const components::Name *_name, const components::ParentEntity *_parent)
        {
          const auto &name = _name->Data();
          if (_parent->Data() != this->world || !this->MatchesPrefix(name))
            return true;
          if (!discovered.emplace(name, _ballEntity).second)
            duplicates.insert(name);
          return true;
        });
    this->duplicateIdentities = duplicates.size();
    const auto countBefore = this->shotCount;
    for (const auto &entry : discovered)
    {
      auto &ball = this->balls[entry.first];
      ball.present = true;
      ball.counted = this->shotIdentities.count(entry.first) != 0;
      const bool modelChanged = ball.model != entry.second;
      if (modelChanged)
      {
        if (ball.model != sim::kNullEntity)
          ++this->identityReplacements;
        ball.model = entry.second;
        ball.hasPrevious = false;
        ball.armed = false;
      }
      const auto link = sim::Model(entry.second).LinkByName(_ecm, this->ballLinkName);
      if (ball.link != link)
      {
        if (!modelChanged && ball.link != sim::kNullEntity && link != sim::kNullEntity)
          ++this->identityReplacements;
        ball.hasPrevious = false;
        ball.armed = false;
        ball.link = link;
      }
      if (duplicates.count(entry.first))
      {
        ball.phase = "duplicate_identity";
        ball.hasPrevious = false;
        continue;
      }
      if (ball.link == sim::kNullEntity || !_ecm.HasEntity(ball.link) ||
          !_ecm.Component<components::Pose>(ball.link))
      {
        ball.phase = "missing_link";
        ball.hasPrevious = false;
        continue;
      }
      const auto pose = sim::worldPose(ball.link, _ecm);
      if (!pose.Pos().IsFinite() || !pose.Rot().IsFinite())
      {
        ball.phase = "invalid_pose";
        ball.hasPrevious = false;
        continue;
      }
      ball.worldPosition = pose.Pos();
      ball.poseAvailable = true;
      ball.lastSeen = now;
      if (!this->headAvailable)
      {
        ball.phase = "head_unavailable";
        ball.hasPrevious = false;
        continue;
      }
      ball.localPosition = mouth.Rot().RotateVectorReverse(pose.Pos() - mouth.Pos());
      if (!ball.localPosition.IsFinite())
      {
        ball.phase = "invalid_pose";
        ball.hasPrevious = false;
        continue;
      }
      ball.localPoseAvailable = true;
      const double dt = now - ball.previousTime;
      if (!_info.paused && !ball.counted && ball.hasPrevious &&
          dt > 0.0 && dt <= this->maxSampleGap)
      {
        const auto worldStep = ball.worldPosition - ball.previousWorld;
        const double relativeDx = ball.localPosition.X() - ball.previousLocal.X();
        const double worldSpeed = worldStep.Length()/dt;
        const double forwardWorldSpeed = worldStep.Dot(
            mouth.Rot().RotateVector(math::Vector3d(1.0, 0.0, 0.0)))/dt;
        // Actual ball progress as well as relative mouth progress is required:
        // moving the head across a stationary ball must not create a shot.
        if (worldSpeed > this->maxBallSpeed)
        {
          ++this->discontinuousSamples;
          ball.armed = false;
        }
        else if (ball.armed && ball.previousLocal.X() <= this->planeX &&
            ball.localPosition.X() > this->planeX &&
            relativeDx/dt > this->minForwardSpeed &&
            forwardWorldSpeed > this->minForwardSpeed)
        {
          const double alpha = (this->planeX - ball.previousLocal.X())/relativeDx;
          const auto crossing = ball.previousLocal +
              alpha*(ball.localPosition - ball.previousLocal);
          const double usableRadius = this->apertureRadius - this->ballRadius;
          if (crossing.Y()*crossing.Y() + crossing.Z()*crossing.Z() <=
              usableRadius*usableRadius)
          {
            if (this->shotCount != std::numeric_limits<std::uint32_t>::max())
            {
              this->shotIdentities.insert(entry.first);
              ++this->shotCount;
              ball.counted = true;
              ball.crossingTime = ball.previousTime + alpha*dt;
              ball.crossingPosition = crossing;
            }
            else
              this->countOverflow = true;
          }
          else
            ++this->outsideApertureCrossings;
        }
      }
      if (ball.localPosition.X() <= this->planeX - this->armingDistance)
        ball.armed = true;
      ball.phase = ball.counted ? "counted" :
          (ball.localPosition.X() <= this->planeX ? "upstream" : "downstream_uncounted");
      ball.previousWorld = ball.worldPosition;
      ball.previousLocal = ball.localPosition;
      ball.previousTime = now;
      // Pause, a missing pose, or a reset establishes a new baseline; none is
      // bridged by an invented crossing when subsequent measurements resume.
      ball.hasPrevious = !_info.paused;
    }
    for (auto &entry : this->balls)
    {
      if (!entry.second.present)
        entry.second.hasPrevious = false;
    }
    if (reset || countBefore != this->shotCount ||
        now - this->lastPublishTime >= 1.0/this->publishRate)
    {
      this->PublishCount();
      this->PublishInventory(now, _info.paused);
      this->lastPublishTime = now;
    }
  }

private:
  struct Ball
  {
    sim::Entity model{sim::kNullEntity};
    sim::Entity link{sim::kNullEntity};
    math::Vector3d worldPosition{math::Vector3d::Zero};
    math::Vector3d localPosition{math::Vector3d::Zero};
    math::Vector3d previousWorld{math::Vector3d::Zero};
    math::Vector3d previousLocal{math::Vector3d::Zero};
    math::Vector3d crossingPosition{math::Vector3d::Zero};
    double previousTime{0.0};
    double lastSeen{0.0};
    double crossingTime{0.0};
    bool present{false};
    bool poseAvailable{false};
    bool localPoseAvailable{false};
    bool hasPrevious{false};
    bool armed{false};
    bool counted{false};
    std::string phase{"unobserved"};
  };

  static bool Positive(double value)
  { return std::isfinite(value) && value > 0.0; }
  static bool Nonnegative(double value)
  { return std::isfinite(value) && value >= 0.0; }
  bool MatchesPrefix(const std::string &name) const
  { return name.compare(0, this->ballPrefix.size(), this->ballPrefix) == 0; }

  std::string ContactBallName(const std::string &collision) const
  {
    std::size_t first = 0;
    while (first < collision.size())
    {
      const auto end = collision.find("::", first);
      const auto part = collision.substr(first, end == std::string::npos ? end : end-first);
      if (this->MatchesPrefix(part))
        return part;
      if (end == std::string::npos)
        break;
      first = end+2;
    }
    return {};
  }

  void OnReset(const ignition::msgs::Boolean &_message)
  {
    if (_message.data())
      this->resetRequested.store(true);
  }

  void OnContacts(const ignition::msgs::Contacts &_message)
  {
    // Transport callbacks only accumulate received contact telemetry. They
    // never access ECM or infer impacts from ball positions.
    std::lock_guard<std::mutex> lock(this->contactMutex);
    ++this->contactMessages;
    this->receivedContactPairs += static_cast<std::uint64_t>(_message.contact_size());
    for (const auto &contact : _message.contact())
    {
      const auto one = this->ContactBallName(contact.collision1().name());
      const auto two = this->ContactBallName(contact.collision2().name());
      if (one.empty() && two.empty())
        continue;
      ++this->ballContactPairs;
      if (!one.empty())
        ++this->contactsPerBall[one];
      if (!two.empty() && two != one)
        ++this->contactsPerBall[two];
    }
  }

  void ResetAccounting()
  {
    this->balls.clear();
    this->shotIdentities.clear();
    this->shotCount = 0;
    this->countOverflow = false;
    this->outsideApertureCrossings = 0;
    this->discontinuousSamples = 0;
    this->identityReplacements = 0;
    this->lastPublishTime = -std::numeric_limits<double>::infinity();
    ++this->resetEpoch;
    std::lock_guard<std::mutex> lock(this->contactMutex);
    this->contactMessages = 0;
    this->receivedContactPairs = 0;
    this->ballContactPairs = 0;
    this->contactsPerBall.clear();
  }

  static std::string JsonString(const std::string &value)
  {
    std::ostringstream result;
    result << '"';
    for (const unsigned char c : value)
    {
      switch (c)
      {
        case '"': result << "\\\""; break;
        case '\\': result << "\\\\"; break;
        case '\n': result << "\\n"; break;
        case '\r': result << "\\r"; break;
        case '\t': result << "\\t"; break;
        default:
          if (c < 0x20)
            result << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                   << static_cast<unsigned>(c) << std::dec;
          else
            result << static_cast<char>(c);
      }
    }
    result << '"';
    return result.str();
  }

  static void JsonVector(std::ostringstream &out, const math::Vector3d &value)
  { out << '[' << value.X() << ',' << value.Y() << ',' << value.Z() << ']'; }

  void PublishCount()
  {
    ignition::msgs::UInt32 message;
    message.set_data(this->shotCount);
    this->countPub.Publish(message);
  }

  void PublishInventory(double now, bool paused)
  {
    std::size_t present = 0, missing = 0, unavailable = 0, countedPresent = 0;
    std::map<std::string, std::size_t> phases;
    for (const auto &entry : this->balls)
    {
      const auto &ball = entry.second;
      ball.present ? ++present : ++missing;
      if (ball.present && !ball.poseAvailable)
        ++unavailable;
      if (ball.present && ball.counted)
        ++countedPresent;
      ++phases[ball.phase];
    }
    std::uint64_t messages, pairs, ballPairs;
    std::map<std::string, std::uint64_t> contacts;
    {
      std::lock_guard<std::mutex> lock(this->contactMutex);
      messages = this->contactMessages;
      pairs = this->receivedContactPairs;
      ballPairs = this->ballContactPairs;
      contacts = this->contactsPerBall;
    }
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << std::boolalpha
        << "{\"schema\":\"physical_ball_monitor/v1\",\"sim_time\":" << now
        << ",\"paused\":" << paused << ",\"reset_epoch\":" << this->resetEpoch
        << ",\"shot_count\":" << this->shotCount
        << ",\"count_semantics\":\"unique observed forward muzzle crossings since reset\""
        << ",\"expected_ball_count\":" << this->expectedBallCount
        << ",\"present_count\":" << present << ",\"ever_seen_count\":" << this->balls.size()
        << ",\"missing_seen_count\":" << missing
        << ",\"unaccounted_expected_count\":" <<
            (present < this->expectedBallCount ? this->expectedBallCount-present : 0)
        << ",\"counted_present_count\":" << countedPresent
        << ",\"unavailable_pose_count\":" << unavailable
        << ",\"duplicate_identity_count\":" << this->duplicateIdentities
        << ",\"identity_replacement_count\":" << this->identityReplacements
        << ",\"inventory_matches_expected\":" <<
            (present == this->expectedBallCount && missing == 0 && unavailable == 0 &&
             this->duplicateIdentities == 0)
        << ",\"head_available\":" << this->headAvailable
        << ",\"outside_aperture_crossings\":" << this->outsideApertureCrossings
        << ",\"discontinuous_samples\":" << this->discontinuousSamples
        << ",\"count_overflow\":" << this->countOverflow
        << ",\"contact_topic\":" << JsonString(this->contactTopic)
        << ",\"contact_telemetry_configured\":" << !this->contactTopic.empty()
        << ",\"contact_messages_received\":" << messages
        << ",\"contact_pairs_received\":" << pairs
        << ",\"ball_contact_pairs_received\":" << ballPairs
        << ",\"contact_coverage_verified\":false,\"phases\":{";
    bool first = true;
    for (const auto &phase : phases)
    {
      if (!first) out << ',';
      first = false;
      out << JsonString(phase.first) << ':' << phase.second;
    }
    out << "},\"balls\":[";
    first = true;
    for (const auto &entry : this->balls)
    {
      if (!first) out << ',';
      first = false;
      const auto &ball = entry.second;
      out << "{\"identity\":" << JsonString(entry.first)
          << ",\"model_entity\":" << ball.model << ",\"link_entity\":" << ball.link
          << ",\"present\":" << ball.present << ",\"phase\":" << JsonString(ball.phase)
          << ",\"counted\":" << ball.counted << ",\"armed\":" << ball.armed
          << ",\"last_seen_sim_time\":" << ball.lastSeen << ",\"world_position\":";
      if (ball.present && ball.poseAvailable) JsonVector(out, ball.worldPosition);
      else out << "null";
      out << ",\"muzzle_local_position\":";
      if (ball.present && ball.localPoseAvailable)
        JsonVector(out, ball.localPosition);
      else out << "null";
      out << ",\"crossing_sim_time\":";
      if (ball.counted) out << ball.crossingTime;
      else out << "null";
      out << ",\"crossing_muzzle_local_position\":";
      if (ball.counted) JsonVector(out, ball.crossingPosition);
      else out << "null";
      out << ",\"contact_pair_observations\":" << contacts[entry.first] << '}';
    }
    out << "]}";
    ignition::msgs::StringMsg message;
    message.set_data(out.str());
    this->inventoryPub.Publish(message);
  }

  sim::Model launcher;
  sim::Entity world{sim::kNullEntity};
  sim::Entity head{sim::kNullEntity};
  math::Pose3d muzzlePose;
  std::string headName;
  std::string ballPrefix;
  std::string ballLinkName;
  std::string contactTopic;
  double apertureRadius{0.0}, ballRadius{0.02}, planeX{0.0};
  double armingDistance{0.002}, minForwardSpeed{0.01}, maxBallSpeed{100.0};
  double maxSampleGap{0.05}, publishRate{5.0}, lastSimTime{0.0};
  double lastPublishTime{-std::numeric_limits<double>::infinity()};
  std::size_t expectedBallCount{100}, duplicateIdentities{0};
  std::uint32_t shotCount{0};
  std::uint64_t resetEpoch{0}, identityReplacements{0}, outsideApertureCrossings{0};
  std::uint64_t discontinuousSamples{0};
  bool configured{false}, haveSimTime{false}, headAvailable{false}, countOverflow{false};
  std::map<std::string, Ball> balls;
  std::unordered_set<std::string> shotIdentities;
  std::atomic<bool> resetRequested{false};
  std::mutex contactMutex;
  std::uint64_t contactMessages{0}, receivedContactPairs{0}, ballContactPairs{0};
  std::map<std::string, std::uint64_t> contactsPerBall;
  ignition::transport::Node::Publisher countPub;
  ignition::transport::Node::Publisher inventoryPub;
  // Node is destroyed first so callbacks cannot outlive their state.
  ignition::transport::Node node;
};
}  // namespace pingpong

IGNITION_ADD_PLUGIN(pingpong::PhysicalBallMonitor,
    ignition::gazebo::System,
    pingpong::PhysicalBallMonitor::ISystemConfigure,
    pingpong::PhysicalBallMonitor::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(pingpong::PhysicalBallMonitor, "pingpong::PhysicalBallMonitor")
