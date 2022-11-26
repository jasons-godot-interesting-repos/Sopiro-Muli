#pragma once

#include "aabb.h"
#include "collision.h"
#include "collision_filter.h"
#include "common.h"
#include "edge.h"
#include "settings.h"

namespace muli
{

class World;
class Collider;
class Shape;
struct Node;
struct ContactEdge;
struct JointEdge;

// Children: Polygon, Circle
class RigidBody final
{
public:
    enum Type
    {
        static_body = 0,
        kinematic_body,
        dynamic_body,
    };

    enum Flag
    {
        flag_island = 1 << 0,
        flag_sleeping = 1 << 1,
        flag_fixed_rotation = 1 << 2,
    };

    RigidBody(RigidBody::Type type);
    ~RigidBody() noexcept;

    RigidBody(const RigidBody&) = delete;
    RigidBody& operator=(const RigidBody&) = delete;

    RigidBody(RigidBody&& other) noexcept = delete;
    RigidBody& operator=(RigidBody&& other) = delete;

    bool TestPoint(const Vec2& q) const;
    Vec2 GetClosestPoint(const Vec2& q) const;
    bool RayCast(const RayCastInput& input, RayCastOutput* output) const;

    const Transform& GetTransform() const;
    void SetTransform(const Vec2& pos, float angle);
    const Vec2& GetPosition() const;
    void SetPosition(const Vec2& pos);
    void SetPosition(float x, float y);
    const Rotation& GetRotation() const;
    void SetRotation(const Rotation& rotation);
    void SetRotation(float angle);
    float GetAngle() const;
    void Translate(const Vec2& d);
    void Translate(float dx, float dy);
    void Rotate(float a);

    void AddForce(const Vec2& localPosition, const Vec2& force);
    void Awake();

    float GetMass() const;
    float GetInverseMass() const;
    float GetInertia() const;
    float GetInverseInertia() const;

    const Vec2& GetLinearVelocity() const;
    void SetLinearVelocity(const Vec2& linearVelocity);
    void SetLinearVelocity(float vx, float vy);
    float GetAngularVelocity() const;
    void SetAngularVelocity(float angularVelocity);

    const Vec2& GetForce() const;
    void SetForce(const Vec2& force);
    float GetTorque() const;
    void SetTorque(float torque);

    Type GetType() const;
    bool IsSleeping() const;
    void SetFixedRotation(bool fixed);
    bool IsRotationFixed() const;

    uint32 GetID() const;
    uint32 GetIslandID() const;
    RigidBody* GetPrev() const;
    RigidBody* GetNext() const;
    World* GetWorld() const;

    Collider* AddCollider(Shape* shape);
    Collider* GetColliderList() const;

    void SetCollisionFilter(const CollisionFilter& filter) const;
    void SetFriction(float friction) const;
    void SetRestitution(float restitution) const;
    void SetSurfaceSpeed(float surfaceSpeed) const;

    // Callbacks
    std::function<void(RigidBody*)> OnDestroy = nullptr;
    uint32 userFlag;

protected:
    friend class World;
    friend class Island;

    friend class AABBTree;
    friend class BroadPhase;
    friend class ContactManager;

    friend class Contact;
    friend class ContactSolver;
    friend class BlockSolver;
    friend class PositionSolver;

    friend class Joint;
    friend class GrabJoint;
    friend class RevoluteJoint;
    friend class DistanceJoint;
    friend class AngleJoint;
    friend class WeldJoint;
    friend class LineJoint;
    friend class PrismaticJoint;
    friend class PulleyJoint;
    friend class MotorJoint;

    friend class Collider;

    // Center of mass in local space = (0, 0)
    Transform transform;

    Vec2 force;   // N
    float torque; // N⋅m

    Vec2 linearVelocity;   // m/s
    float angularVelocity; // rad/s

    float mass; // kg
    float invMass;
    float inertia; // kg⋅m²
    float invInertia;

    Type type;

    uint16 flag;

    Collider* colliderList;
    int32 colliderCount;

    void ResetMassData();

private:
    World* world;
    uint32 id;
    uint32 islandID;

    ContactEdge* contactList;
    JointEdge* jointList;

    float resting;

    RigidBody* prev;
    RigidBody* next;

    void NotifyForceUpdate() const;
};

inline const Transform& RigidBody::GetTransform() const
{
    return transform;
}

inline void RigidBody::SetTransform(const Vec2& _pos, float _angle)
{
    transform.position = _pos;
    transform.rotation = _angle;
}

inline const Vec2& RigidBody::GetPosition() const
{
    return transform.position;
}

inline void RigidBody::SetPosition(const Vec2& _pos)
{
    transform.position = _pos;
}

inline void RigidBody::SetPosition(float x, float y)
{
    transform.position = Vec2{ x, y };
}

inline const Rotation& RigidBody::GetRotation() const
{
    return transform.rotation;
}

inline void RigidBody::SetRotation(const Rotation& _rotation)
{
    transform.rotation = _rotation;
}

inline void RigidBody::SetRotation(float _angle)
{
    transform.rotation = _angle;
}

inline float RigidBody::GetAngle() const
{
    return transform.rotation.angle;
}

inline void RigidBody::Translate(const Vec2& d)
{
    transform.position += d;
}

inline void RigidBody::Translate(float dx, float dy)
{
    transform.position.x += dx;
    transform.position.y += dy;
}

inline void RigidBody::Rotate(float a)
{
    transform.rotation += a;
}

inline float RigidBody::GetMass() const
{
    return mass;
}

inline float RigidBody::GetInverseMass() const
{
    return invMass;
}

inline float RigidBody::GetInertia() const
{
    return inertia;
}

inline float RigidBody::GetInverseInertia() const
{
    return invInertia;
}

inline void RigidBody::Awake()
{
    resting = 0.0f;
    flag &= ~Flag::flag_sleeping;
}

inline void RigidBody::AddForce(const Vec2& localPosition, const Vec2& f)
{
    if (type != dynamic_body)
    {
        return;
    }

    force += f;
    torque += Cross(transform * localPosition - transform.position, f);
    NotifyForceUpdate();
}

inline const Vec2& RigidBody::GetLinearVelocity() const
{
    return linearVelocity;
}

inline void RigidBody::SetLinearVelocity(const Vec2& _linearVelocity)
{
    if (type == static_body)
    {
        return;
    }

    linearVelocity = _linearVelocity;
}

inline void RigidBody::SetLinearVelocity(float vx, float vy)
{
    if (type == static_body)
    {
        return;
    }

    linearVelocity.Set(vx, vy);
}

inline float RigidBody::GetAngularVelocity() const
{
    return angularVelocity;
}

inline void RigidBody::SetAngularVelocity(float _angularVelocity)
{
    if (type == static_body)
    {
        return;
    }

    angularVelocity = _angularVelocity;
}

inline const Vec2& RigidBody::GetForce() const
{
    return force;
}

inline void RigidBody::SetForce(const Vec2& _force)
{
    if (type != dynamic_body)
    {
        return;
    }

    force = _force;
    NotifyForceUpdate();
}

inline float RigidBody::GetTorque() const
{
    return torque;
}

inline void RigidBody::SetTorque(float _torque)
{
    if (type != dynamic_body)
    {
        return;
    }

    torque = _torque;
    NotifyForceUpdate();
}

inline RigidBody::Type RigidBody::GetType() const
{
    return type;
}

inline bool RigidBody::IsSleeping() const
{
    return flag & Flag::flag_sleeping;
}

inline void RigidBody::SetFixedRotation(bool fixed)
{
    if (fixed)
    {
        flag |= flag_fixed_rotation;
    }
    else
    {
        flag &= ~flag_fixed_rotation;
    }

    ResetMassData();
}

inline bool RigidBody::IsRotationFixed() const
{
    return (flag & flag_fixed_rotation) == flag_fixed_rotation;
}

inline uint32 RigidBody::GetID() const
{
    return id;
}

inline uint32 RigidBody::GetIslandID() const
{
    return islandID;
}

inline RigidBody* RigidBody::GetPrev() const
{
    return prev;
}

inline RigidBody* RigidBody::GetNext() const
{
    return next;
}

inline World* RigidBody::GetWorld() const
{
    return world;
}

inline Collider* RigidBody::GetColliderList() const
{
    return colliderList;
}

} // namespace muli