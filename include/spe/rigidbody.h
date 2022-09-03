#pragma once

#include "aabb.h"
#include "common.h"
#include "entity.h"
#include "settings.h"

namespace spe
{

struct Node;
struct ContactEdge;
struct JointEdge;

// Children: Polygon, Circle
class RigidBody : public Entity
{
    friend class World;
    friend class Island;

    friend class Contact;
    friend class ContactSolver;
    friend class BlockSolver;

    friend class Joint;
    friend class GrabJoint;
    friend class RevoluteJoint;
    friend class DistanceJoint;

    friend class AABBTree;
    friend class BroadPhase;
    friend class ContactManager;

public:
    enum Shape : uint8_t
    {
        ShapeCircle,
        ShapePolygon,
        ShapeEdge,
    };

    enum Type : uint8_t
    {
        Static,
        Dynamic,
    };

    RigidBody(RigidBody::Type _type, RigidBody::Shape _shape);
    ~RigidBody() noexcept;

    RigidBody(const RigidBody&) = delete;
    RigidBody& operator=(const RigidBody&) = delete;

    RigidBody(RigidBody&& _other) noexcept;
    RigidBody& operator=(RigidBody&& _other) = delete;

    virtual void SetDensity(float d) = 0;
    virtual void SetMass(float m) = 0;
    virtual float GetArea() const = 0;
    virtual AABB GetAABB() const = 0;

    void Awake();

    float GetDensity() const;
    float GetMass() const;
    float GetInverseMass() const;
    float GetInertia() const;
    float GetInverseInertia() const;
    float GetFriction() const;
    void SetFriction(float _friction);
    float GetRestitution() const;
    void SetRestitution(float _restitution);
    float GetSurfaceSpeed() const;
    void SetSurfaceSpeed(float _surfaceSpeed);
    glm::vec2 GetLinearVelocity() const;
    void SetLinearVelocity(glm::vec2 _linearVelocity);
    float GetAngularVelocity() const;
    void SetAngularVelocity(float _angularVelocity);
    glm::vec2 GetForce() const;
    void SetForce(glm::vec2 _force);
    float GetTorque() const;
    void SetTorque(float _torque);
    Type GetType() const;
    Shape GetShape() const;
    bool IsSleeping() const;

    uint32_t GetID() const;
    uint32_t GetIslandID() const;
    const Node* GetNode() const;
    RigidBody* GetPrev() const;
    RigidBody* GetNext() const;

    // Callbacks
    std::function<void(RigidBody*)> OnDestroy = nullptr;

protected:
    // Center of mass in local space = (0, 0)
    glm::vec2 force{ 0.0f }; // N
    float torque = 0.0f;     // N⋅m

    glm::vec2 linearVelocity{ 0.0f }; // m/s
    float angularVelocity = 0.0f;     // rad/s

    float density; // kg/m²
    float mass;    // kg
    float invMass;
    float inertia; // kg⋅m²
    float invInertia;

    float friction = DEFAULT_FRICTION;
    float restitution = DEFAULT_RESTITUTION;
    float surfaceSpeed = DEFAULT_SURFACESPEED; // m/s (Tangential speed)

    Shape shape;
    Type type;

private:
    bool moved = false;

    World* world = nullptr;
    uint32_t id = 0;
    uint32_t islandID = 0;

    ContactEdge* contactList = nullptr;
    JointEdge* jointList = nullptr;

    float resting = 0.0f;
    bool sleeping = false;

    Node* node = nullptr;
    RigidBody* prev = nullptr;
    RigidBody* next = nullptr;
};

inline float RigidBody::GetDensity() const
{
    return density;
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

inline const Node* RigidBody::GetNode() const
{
    return node;
}

inline void RigidBody::Awake()
{
    resting = 0.0f;
    sleeping = false;
}

inline float RigidBody::GetFriction() const
{
    return friction;
}

inline void RigidBody::SetFriction(float _friction)
{
    friction = std::move(_friction);
}

inline float RigidBody::GetRestitution() const
{
    return restitution;
}

inline void RigidBody::SetRestitution(float _restitution)
{
    restitution = std::move(_restitution);
}

inline float RigidBody::GetSurfaceSpeed() const
{
    return surfaceSpeed;
}

inline void RigidBody::SetSurfaceSpeed(float _surfaceSpeed)
{
    surfaceSpeed = std::move(_surfaceSpeed);
}

inline glm::vec2 RigidBody::GetLinearVelocity() const
{
    return linearVelocity;
}

inline void RigidBody::SetLinearVelocity(glm::vec2 _linearVelocity)
{
    linearVelocity = std::move(_linearVelocity);
}

inline float RigidBody::GetAngularVelocity() const
{
    return angularVelocity;
}

inline void RigidBody::SetAngularVelocity(float _angularVelocity)
{
    angularVelocity = std::move(_angularVelocity);
}

inline glm::vec2 RigidBody::GetForce() const
{
    return force;
}

inline void RigidBody::SetForce(glm::vec2 _force)
{
    force = std::move(_force);
}

inline float RigidBody::GetTorque() const
{
    return torque;
}

inline void RigidBody::SetTorque(float _torque)
{
    torque = std::move(_torque);
}

inline RigidBody::Type RigidBody::GetType() const
{
    return type;
}

inline RigidBody::Shape RigidBody::GetShape() const
{
    return shape;
}

inline bool RigidBody::IsSleeping() const
{
    return sleeping;
}

inline uint32_t RigidBody::GetID() const
{
    return id;
}

inline uint32_t RigidBody::GetIslandID() const
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

} // namespace spe