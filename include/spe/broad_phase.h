#pragma once

#include "aabb.h"
#include "aabbtree.h"
#include "common.h"

namespace spe
{

class BroadPhase
{
    friend class World;

public:
    BroadPhase(World& _world, float _aabbMargin = DEFAULT_AABB_MARGIN, float _velocityMultiplier = DEFAULT_VELOCITY_MULTIPLIER);
    ~BroadPhase() noexcept;

    void UpdateDynamicTree(float dt);
    void FindContacts(std::function<void(RigidBody* bodyA, RigidBody* bodyB)> callback) const;
    bool TestOverlap(RigidBody* bodyA, RigidBody* bodyB) const;

    void Add(RigidBody* body);
    void Remove(RigidBody* body);
    void Reset();

private:
    World& world;
    AABBTree tree;

    float aabbMargin;
    float velocityMultiplier;
};

inline BroadPhase::BroadPhase(World& _world, float _aabbMargin, float _velocityMultiplier)
    : world{ _world }
    , aabbMargin{ _aabbMargin }
    , velocityMultiplier{ _velocityMultiplier }
{
}

inline BroadPhase::~BroadPhase()
{
    Reset();
}

inline void BroadPhase::Reset()
{
    tree.Reset();
}

inline void BroadPhase::Add(RigidBody* body)
{
    AABB fatAABB = body->GetAABB();
    fatAABB.min -= aabbMargin;
    fatAABB.max += aabbMargin;

    tree.Insert(body, fatAABB);
}

inline void BroadPhase::Remove(RigidBody* body)
{
    tree.Remove(body);
}

inline bool BroadPhase::TestOverlap(RigidBody* bodyA, RigidBody* bodyB) const
{
    return TestOverlapAABB(bodyA->node->aabb, bodyB->node->aabb);
}

} // namespace spe