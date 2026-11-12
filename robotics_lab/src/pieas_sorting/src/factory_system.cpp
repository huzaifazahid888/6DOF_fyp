/* Conveyor traction and a proximity-gated suction surrogate for Gazebo Harmonic.
 * Perception never reads object identities or poses from this plugin. The world
 * state is used only for conveyor indexing, attachment and drop verification.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

namespace pieas
{
using namespace gz;
using namespace gz::sim;

class FactorySystem : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemReset
{
    struct Part
    {
        Entity model{kNullEntity};
        Entity link{kNullEntity};
        bool removedFromBelt{false};
    };
    transport::Node node;
    transport::Node::Publisher statePublisher;
    std::mutex mutex;
    std::deque<std::string> commands;
    std::chrono::steady_clock::time_point heartbeat{};
    std::vector<Part> parts;
    Entity parent{kNullEntity}, joint{kNullEntity};
    int held{-1}, lastReleased{-1}, count{6};
    bool running{false};
    double bx{0.36}, top{0.12}, halfWidth{0.045}, ymin{-1.15}, ymax{0.12};
    double stopY{0.0}, speed{0.025}, height{0.03}, lastPublish{-1};
    std::string ack, error;
    math::Pose3d toolOffset{0.00152937072616,-0.0000956811660029,0.195355912714,
                          3.14159265359,2.0650148258e-13,1.57046745796};

    void OnCommand(const msgs::StringMsg &message)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (commands.size() < 16) commands.push_back(message.data());
    }
    void OnHeartbeat(const msgs::Boolean &message)
    {
        if (!message.data()) return;
        std::lock_guard<std::mutex> lock(mutex);
        heartbeat = std::chrono::steady_clock::now();
    }
    bool Alive()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return std::chrono::steady_clock::now() - heartbeat < std::chrono::seconds(2);
    }
    bool Resolve(EntityComponentManager &ecm)
    {
        if (parent == kNullEntity)
        {
            Entity arm = ecm.EntityByComponents(components::Model(), components::Name("pieas_arm"));
            if (arm == kNullEntity) return false;
            parent = Model(arm).LinkByName(ecm,"rg2_link");
        }
        if (parent == kNullEntity) return false;
        for (int i=0; i<count; ++i)
        {
            if (parts[i].link != kNullEntity) continue;
            std::ostringstream name;
            name << "part_" << std::setw(3) << std::setfill('0') << i;
            parts[i].model = ecm.EntityByComponents(components::Model(),components::Name(name.str()));
            if (parts[i].model == kNullEntity) return false;
            parts[i].link = Model(parts[i].model).LinkByName(ecm,"body");
            if (parts[i].link == kNullEntity) return false;
            Link(parts[i].link).EnableVelocityChecks(ecm);
        }
        return true;
    }
    int Front(const EntityComponentManager &ecm) const
    {
        int front=-1;
        double largest=ymin-1;
        for (int i=0; i<count; ++i)
        {
            if (parts[i].removedFromBelt) continue;
            auto p=worldPose(parts[i].link,ecm).Pos();
            if (std::abs(p.X()-bx)>halfWidth || p.Y()<ymin || p.Y()>ymax ||
                std::abs(p.Z()-(top+height/2))>0.015) continue;
            if (p.Y()>largest) { front=i; largest=p.Y(); }
        }
        return front;
    }
    void Process(const std::string &message, EntityComponentManager &ecm)
    {
        auto split=message.find('|');
        if (split==std::string::npos) return;
        const auto id=message.substr(0,split), command=message.substr(split+1);
        if (id==ack) return; // Retry of last command is idempotent.
        ack=id; error.clear();
        if (command=="stop") { running=false; return; }
        if (command=="run")
        {
            if (!Alive()) { error="heartbeat_missing"; return; }
            if (held>=0) { error="payload_still_attached"; return; }
            running=true; return;
        }
        if (command=="release")
        {
            if (held<0) { error="nothing_attached"; return; }
            ecm.RequestRemoveEntity(joint);
            joint=kNullEntity; lastReleased=held; held=-1;
            return;
        }
        if (command!="grip") { error="unknown_command"; return; }
        if (!Alive() || running || held>=0) { error="grip_interlock"; return; }
        const int candidate=Front(ecm);
        if (candidate<0) { error="no_part_at_station"; return; }
        const auto object=worldPose(parts[candidate].link,ecm);
        const auto tool=worldPose(parent,ecm)*toolOffset;
        const auto contact=object.Pos()+math::Vector3d(0,0,height/2);
        const auto offset=tool.Pos()-contact;
        const auto up=tool.Rot().RotateVector(math::Vector3d::UnitZ);
        auto velocity=Link(parts[candidate].link).WorldLinearVelocity(ecm);
        if (std::hypot(offset.X(),offset.Y())>0.008 || std::abs(offset.Z())>0.005 ||
            up.Z()<0.98 || !velocity || velocity->Length()>0.008)
        {
            error="suction_pose_or_speed_outside_tolerance"; return;
        }
        // A real physics fixed joint: carried objects retain mass and collisions.
        // This approximates a vacuum seal; it is not a force/contact grasp model.
        joint=ecm.CreateEntity();
        ecm.CreateComponent(joint,components::DetachableJoint({parent,parts[candidate].link,"fixed"}));
        held=candidate; parts[candidate].removedFromBelt=true;
    }
public:
    void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                   EntityComponentManager &, EventManager &) override
    {
        bx=sdf->Get<double>("belt_x"); top=sdf->Get<double>("belt_top");
        halfWidth=sdf->Get<double>("belt_half_width"); ymin=sdf->Get<double>("belt_y_min");
        ymax=sdf->Get<double>("belt_y_max"); stopY=sdf->Get<double>("stop_y");
        speed=sdf->Get<double>("speed"); count=sdf->Get<int>("part_count");height=sdf->Get<double>("part_height");
        parts.resize(count);
        node.Subscribe("/factory/command",&FactorySystem::OnCommand,this);
        node.Subscribe("/factory/heartbeat",&FactorySystem::OnHeartbeat,this);
        statePublisher=node.Advertise<msgs::StringMsg>("/factory/state");
    }
    void Reset(const UpdateInfo &,EntityComponentManager &ecm) override
    {
        if (joint!=kNullEntity) ecm.RequestRemoveEntity(joint);
        joint=parent=kNullEntity; held=lastReleased=-1; running=false;
        parts.assign(count,Part{});lastPublish=-1;ack.clear();error="world_reset_restart_sorter";
        std::lock_guard<std::mutex> lock(mutex);
        commands.clear();heartbeat={};
    }
    void PreUpdate(const UpdateInfo &info,EntityComponentManager &ecm) override
    {
        if (info.paused || !Resolve(ecm)) return;
        std::deque<std::string> pending;
        { std::lock_guard<std::mutex> lock(mutex); pending.swap(commands); }
        for (const auto &message: pending) Process(message,ecm);
        if (!Alive()) running=false;
        const int front=Front(ecm);
        double targetSpeed=0;
        if (running && front>=0)
        {
            const double remaining=stopY-worldPose(parts[front].link,ecm).Pos().Y();
            targetSpeed=std::clamp(3*remaining,0.0,speed);
            if (remaining<=0.001) running=false;
        }
        else if (front<0) running=false;
        for (auto &part: parts)
        {
            if (part.removedFromBelt) continue;
            auto p=worldPose(part.link,ecm).Pos();
            if (std::abs(p.X()-bx)>halfWidth || p.Y()<ymin || p.Y()>ymax ||
                std::abs(p.Z()-(top+height/2))>0.015) continue;
            Link link(part.link);
            auto v=link.WorldLinearVelocity(ecm);
            if (!v) continue;
            // Bounded tangential traction. Gravity/contact remain physics-driven.
            double fx=std::clamp(0.025*(40*(-4*(p.X()-bx)-v->X())),-0.15,0.15);
            double fy=std::clamp(0.025*40*((running?targetSpeed:0)-v->Y()),-0.15,0.15);
            link.AddWorldForce(ecm,math::Vector3d(fx,fy,0));
        }
        const double now=std::chrono::duration<double>(info.simTime).count();
        if (now-lastPublish<0.05) return;
        lastPublish=now;
        bool atStation=front>=0 && std::abs(worldPose(parts[front].link,ecm).Pos().Y()-stopY)<0.004;
        std::ostringstream s;
        s << std::setprecision(10) << "{\"sim_time\":" << now << ",\"running\":" << (running?"true":"false")
          << ",\"at_station\":" << (atStation?"true":"false") << ",\"held\":" << (held>=0?"true":"false")
          << ",\"ack\":\"" << ack << "\",\"error\":\"" << error << "\",\"release_xyz\":";
        // Ground truth is for drop verification only, never perception or IK.
        if (lastReleased>=0)
        {
            auto p=worldPose(parts[lastReleased].link,ecm).Pos();
            s << '[' << p.X() << ',' << p.Y() << ',' << p.Z() << ']';
        }
        else s << "null";
        s << '}';
        msgs::StringMsg message;message.set_data(s.str());statePublisher.Publish(message);
    }
};
}
GZ_ADD_PLUGIN(pieas::FactorySystem,gz::sim::System,
              pieas::FactorySystem::ISystemConfigure,
              pieas::FactorySystem::ISystemPreUpdate,
              pieas::FactorySystem::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(pieas::FactorySystem,"pieas::FactorySystem")
