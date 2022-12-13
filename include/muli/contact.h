#pragma once

#include "block_solver.h"
#include "collision.h"
#include "constraint.h"
#include "contact_solver.h"
#include "position_solver.h"

namespace muli
{

inline float MixFriction(float f1, float f2)
{
    return Sqrt(f1 * f2);
}

inline float MixRestitution(float r1, float r2)
{
    return Max(r1, r2);
}

class RigidBody;
class Contact;

struct ContactEdge
{
    RigidBody* other;
    Contact* contact;
    ContactEdge* prev;
    ContactEdge* next;
};

class Contact : Constraint
{
public:
    Contact(Collider* colliderA, Collider* colliderB, const WorldSettings& settings);
    ~Contact() noexcept = default;

    Collider* GetColliderA() const;
    Collider* GetColliderB() const;

    const Contact* GetNext() const;
    const Contact* GetPrev() const;

    const ContactManifold& GetContactManifold() const;
    float GetFriction() const;
    float GetRestitution() const;
    float GetSurfaceSpeed() const;
    bool IsTouching() const;

    void SetEnabled(bool enabled);
    bool IsEnabled() const;

private:
    friend class World;
    friend class Island;
    friend class ContactManager;
    friend class BroadPhase;
    friend class ContactSolver;
    friend class BlockSolver;
    friend class PositionSolver;

    enum
    {
        flag_enabled = 1,
        flag_touching = 1 << 1,
        flag_island = 1 << 2,
        flag_toi = 1 << 3,
    };

    void Update();
    virtual void Prepare() override;
    virtual void SolveVelocityConstraint() override;
    virtual bool SolvePositionConstraint() override;

    DetectionFunction* collisionDetectionFunction;

    RigidBody* b1; // Reference body
    RigidBody* b2; // Incident body

    Collider* colliderA;
    Collider* colliderB;

    Contact* prev;
    Contact* next;

    ContactEdge nodeA;
    ContactEdge nodeB;

    float restitution;
    float friction;
    float surfaceSpeed;

    ContactManifold manifold;

    ContactSolver normalSolvers[MAX_CONTACT_POINT];
    ContactSolver tangentSolvers[MAX_CONTACT_POINT];
    PositionSolver positionSolvers[MAX_CONTACT_POINT];
    BlockSolver blockSolver;

    // Impulse for position correction
    Vec2 cLinearImpulseA;
    float cAngularImpulseA;
    Vec2 cLinearImpulseB;
    float cAngularImpulseB;

    uint16 flag;

    int32 toiCount;
    float toi;
};

inline Collider* Contact::GetColliderA() const
{
    return colliderA;
}

inline Collider* Contact::GetColliderB() const
{
    return colliderB;
}

inline const Contact* Contact::GetPrev() const
{
    return prev;
}

inline const Contact* Contact::GetNext() const
{
    return next;
}

inline const ContactManifold& Contact::GetContactManifold() const
{
    return manifold;
}

inline float Contact::GetFriction() const
{
    return friction;
}

inline float Contact::GetRestitution() const
{
    return restitution;
}

inline float Contact::GetSurfaceSpeed() const
{
    return surfaceSpeed;
}

inline bool Contact::IsTouching() const
{
    return (flag & flag_touching) == flag_touching;
}

inline void Contact::SetEnabled(bool enabled)
{
    if (enabled)
    {
        flag |= flag_enabled;
    }
    else
    {
        flag &= ~flag_enabled;
    }
}

inline bool Contact::IsEnabled() const
{
    return (flag & flag_enabled) == flag_enabled;
}

} // namespace muli
