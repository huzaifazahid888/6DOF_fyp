/* Simulation backend: fixed stem attachment, checked suction, contact monitoring.
 * No fruit labels or pickup coordinates are published to perception or the task.
 * Detachment is an explicit surrogate, not a calibrated stem-fracture model.
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
#include <gz/msgs/contacts.pb.h>
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
class OrchardSystem : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemReset
{
    struct Fruit
    {
        Entity link{kNullEntity};
        Entity stem{kNullEntity};
        bool picked{false};
    };
    transport::Node node;
    transport::Node::Publisher publisher;
    std::mutex mutex;
    std::deque<std::string> commands;
    std::deque<msgs::Entity> contactEntities;
    std::chrono::steady_clock::time_point heartbeat{};
    std::vector<Fruit> fruits;
    Entity tool{kNullEntity}, branch{kNullEntity}, grip{kNullEntity};
    int held{-1}, released{-1}, count{3};
    double radius{.016}, lastPublish{-1};
    bool treeContact{false}, resetFault{false};
    std::string ack, error;
    math::Pose3d offset{.00152937072616,-.0000956811660029,.195355912714,
                        3.14159265359,2.0650148258e-13,1.57046745796};
    void OnCommand(const msgs::StringMsg &m)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (commands.size()<16) commands.push_back(m.data());
    }
    void OnHeartbeat(const msgs::Boolean &m)
    {
        if (!m.data()) return;
        std::lock_guard<std::mutex> lock(mutex);
        heartbeat=std::chrono::steady_clock::now();
    }
    void OnContacts(const msgs::Contacts &m)
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto &c:m.contact())
        {
            if (contactEntities.size() >= 512)
            {
                // A contact backlog must not silently hide a collision.
                treeContact = true;
                break;
            }
            contactEntities.push_back(c.collision1());
            contactEntities.push_back(c.collision2());
        }
    }
    void CheckContacts(EntityComponentManager &ecm)
    {
        std::deque<msgs::Entity> contacts;
        {
            std::lock_guard<std::mutex> lock(mutex);
            contacts.swap(contactEntities);
        }
        for (const auto &contact : contacts)
        {
            const auto entity = entityFromMsg(ecm, contact);
            const auto name = scopedName(entity, ecm, "::", false);
            bool prohibited = name.find("pieas_arm::") != std::string::npos;
            for (int i = 0; i < count; ++i)
            {
                if (!fruits[i].picked) continue;
                std::ostringstream fruitName;
                fruitName << "apple_" << std::setw(3) << std::setfill('0') << i << "::";
                prohibited = prohibited || name.find(fruitName.str()) != std::string::npos;
            }
            if (prohibited)
            {
                std::lock_guard<std::mutex> lock(mutex);
                treeContact = true;
            }
        }
    }
    bool Alive()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return std::chrono::steady_clock::now()-heartbeat<std::chrono::seconds(2);
    }
    bool Damaged()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return treeContact;
    }
    Entity LinkOf(const std::string &name, const std::string &link, EntityComponentManager &ecm)
    {
        auto model=ecm.EntityByComponents(components::Model(),components::Name(name));
        return model==kNullEntity?kNullEntity:Model(model).LinkByName(ecm,link);
    }
    bool Resolve(EntityComponentManager &ecm)
    {
        if (tool==kNullEntity) tool=LinkOf("pieas_arm","rg2_link",ecm);
        if (branch==kNullEntity) branch=LinkOf("tree_branch","body",ecm);
        if (tool==kNullEntity || branch==kNullEntity) return false;
        Link(tool).EnableVelocityChecks(ecm);
        for (int i=0;i<count;++i)
        {
            if (fruits[i].link!=kNullEntity) continue;
            std::ostringstream name;name<<"apple_"<<std::setw(3)<<std::setfill('0')<<i;
            auto link=LinkOf(name.str(),"body",ecm);
            if (link==kNullEntity) return false;
            fruits[i].link=link;
            Link(link).EnableVelocityChecks(ecm);
            fruits[i].stem=ecm.CreateEntity();
            ecm.CreateComponent(fruits[i].stem,components::DetachableJoint({branch,link,"fixed"}));
        }
        return true;
    }
    void Process(const std::string &m,EntityComponentManager &ecm)
    {
        auto split=m.find('|');if (split==std::string::npos) return;
        const auto id=m.substr(0,split),cmd=m.substr(split+1);
        if (ack==id) return;
        ack=id;error.clear();
        if (cmd=="stop") return;
        if (resetFault || Damaged()) { error="tree_contact_or_reset_requires_restart";return; }
        if (!Alive()) { error="heartbeat_missing";return; }
        if (cmd=="release")
        {
            if (held<0) { error="nothing_attached";return; }
            ecm.RequestRemoveEntity(grip);grip=kNullEntity;released=held;held=-1;return;
        }
        if (cmd!="grip" || held>=0) { error="invalid_grip_command";return; }
        auto pose=worldPose(tool,ecm)*offset;
        auto velocity=Link(tool).WorldLinearVelocity(ecm);
        auto angular=Link(tool).WorldAngularVelocity(ecm);
        const auto lever=worldPose(tool,ecm).Rot().RotateVector(offset.Pos());
        if (!velocity || !angular || (*velocity+angular->Cross(lever)).Length()>.012 || angular->Length()>.05 || pose.Rot().RotateVector(math::Vector3d::UnitZ).Z()<.98)
        { error="tool_moving_or_tilted";return; }
        int chosen=-1;double nearest=1.;
        for (int i=0;i<count;++i)
        {
            if (fruits[i].picked) continue;
            auto delta=pose.Pos()-(worldPose(fruits[i].link,ecm).Pos()+math::Vector3d(0,0,radius));
            if (std::hypot(delta.X(),delta.Y())<.006 && std::abs(delta.Z())<.005 && delta.Length()<nearest)
            { chosen=i;nearest=delta.Length(); }
        }
        if (chosen<0) { error="no_fruit_at_measured_tool_pose";return; }
        auto &f=fruits[chosen];
        // Stem removal and attachment preserve the simulated pose and mass.
        ecm.RequestRemoveEntity(f.stem);f.stem=kNullEntity;
        grip=ecm.CreateEntity();ecm.CreateComponent(grip,components::DetachableJoint({tool,f.link,"fixed"}));
        f.picked=true;held=chosen;
    }
public:
    void Configure(const Entity &,const std::shared_ptr<const sdf::Element> &sdf,
                   EntityComponentManager &,EventManager &) override
    {
        count=sdf->Get<int>("fruit_count");radius=sdf->Get<double>("fruit_radius");fruits.resize(count);
        node.Subscribe("/orchard/command",&OrchardSystem::OnCommand,this);
        node.Subscribe("/orchard/heartbeat",&OrchardSystem::OnHeartbeat,this);
        node.Subscribe("/orchard/tree_contacts",&OrchardSystem::OnContacts,this);
        publisher=node.Advertise<msgs::StringMsg>("/orchard/state");
    }
    void Reset(const UpdateInfo &,EntityComponentManager &ecm) override
    {
        if (grip!=kNullEntity) ecm.RequestRemoveEntity(grip);
        for (auto &f:fruits) if(f.stem!=kNullEntity) ecm.RequestRemoveEntity(f.stem);
        fruits.assign(count,Fruit{});tool=branch=grip=kNullEntity;held=released=-1;
        lastPublish=-1;resetFault=true;error="world_reset_restart_required";
        std::lock_guard<std::mutex> lock(mutex);commands.clear();contactEntities.clear();heartbeat={};
    }
    void PreUpdate(const UpdateInfo &info,EntityComponentManager &ecm) override
    {
        if (info.paused || !Resolve(ecm)) return;
        CheckContacts(ecm);
        std::deque<std::string> pending;
        { std::lock_guard<std::mutex> lock(mutex);pending.swap(commands); }
        for (const auto &m:pending) Process(m,ecm);
        double now=std::chrono::duration<double>(info.simTime).count();
        if (now-lastPublish<.05) return;
        lastPublish=now;
        std::ostringstream s;s<<std::setprecision(10)<<"{\"sim_time\":"<<now
            <<",\"running\":false,\"at_station\":true,\"held\":"<<(held>=0?"true":"false")
            <<",\"removed\":"<<std::count_if(fruits.begin(),fruits.end(),[](const Fruit &f){return f.picked;})
            <<",\"tree_contact\":"<<(Damaged()?"true":"false")
            <<",\"reset_fault\":"<<(resetFault?"true":"false")
            <<",\"ack\":\""<<ack<<"\",\"error\":\""<<error<<"\",\"release_xyz\":";
        if (released>=0)
        {
            auto p=worldPose(fruits[released].link,ecm).Pos();s<<'['<<p.X()<<','<<p.Y()<<','<<p.Z()<<']';
        }
        else s<<"null";
        s<<'}';msgs::StringMsg message;message.set_data(s.str());publisher.Publish(message);
    }
};
}
GZ_ADD_PLUGIN(pieas::OrchardSystem,gz::sim::System,pieas::OrchardSystem::ISystemConfigure,
              pieas::OrchardSystem::ISystemPreUpdate,pieas::OrchardSystem::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(pieas::OrchardSystem,"pieas::OrchardSystem")

