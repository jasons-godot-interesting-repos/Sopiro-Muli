#include "muli/world.h"
#include "muli/capsule.h"
#include "muli/circle.h"
#include "muli/island.h"
#include "muli/polygon.h"

namespace muli
{

World::World(const WorldSettings& simulationSettings)
    : settings{ simulationSettings }
    , contactManager{ this }
    , bodyList{ nullptr }
    , bodyListTail{ nullptr }
    , jointList{ nullptr }
    , bodyCount{ 0 }
    , jointCount{ 0 }
    , numIslands{ 0 }
    , sleepingBodies{ 0 }
{
}

World::~World() noexcept
{
    Reset();
}

void World::Step(float dt)
{
    settings.DT = dt;
    settings.INV_DT = 1.0f / dt;

    contactManager.Step(dt);

    // Build the constraint island
    Island island{ this, bodyCount, contactManager.contactCount, jointCount };

    uint32 restingBodies = 0;
    uint32 islandID = 0;
    sleepingBodies = 0;

    // Use stack allocator to avoid per-frame allocation
    RigidBody** stack = (RigidBody**)stackAllocator.Allocate(bodyCount * sizeof(RigidBody*));
    uint32 stackPointer;

    // Perform a DFS(Depth First Search) on the constraint graph
    // After building island, each island can be solved in parallel because they are independent of each other
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        if (b->type == RigidBody::Type::static_body)
        {
            continue;
        }

        if (b->flag & RigidBody::Flag::flag_sleeping || b->flag & RigidBody::Flag::flag_island)
        {
            sleepingBodies += (b->flag & RigidBody::Flag::flag_sleeping) > 0;
            continue;
        }

        stackPointer = 0;
        stack[stackPointer++] = b;

        ++islandID;
        while (stackPointer > 0)
        {
            RigidBody* t = stack[--stackPointer];

            if (t->type == RigidBody::Type::static_body || (t->flag & RigidBody::Flag::flag_island))
            {
                continue;
            }

            t->flag |= RigidBody::Flag::flag_island;
            t->islandID = islandID;
            island.Add(t);

            for (ContactEdge* ce = t->contactList; ce; ce = ce->next)
            {
                if (ce->other->flag & RigidBody::Flag::flag_island)
                {
                    continue;
                }

                if (ce->contact->touching)
                {
                    island.Add(ce->contact);
                    stack[stackPointer++] = ce->other;
                }
            }

            for (JointEdge* je = t->jointList; je; je = je->next)
            {
                if (je->other == t)
                {
                    island.Add(je->joint);
                    t->Awake();
                }

                if (je->other->flag & RigidBody::Flag::flag_island)
                {
                    continue;
                }

                island.Add(je->joint);
                stack[stackPointer++] = je->other;
            }

            if (t->resting > settings.SLEEPING_TRESHOLD)
            {
                restingBodies++;
            }
        }

        island.sleeping = settings.SLEEPING && (restingBodies == island.bodyCount);
        island.Solve();
        island.Clear();
        restingBodies = 0;
    }

    stackAllocator.Free(stack, bodyCount);

    numIslands = islandID;

    for (RigidBody* b : destroyBufferBody)
    {
        Destroy(b);
    }
    for (Joint* j : destroyBufferJoint)
    {
        Destroy(j);
    }

    destroyBufferBody.clear();
    destroyBufferJoint.clear();
}

void World::Reset()
{
    RigidBody* b = bodyList;
    while (b)
    {
        RigidBody* b0 = b;
        b = b->next;
        Destroy(b0);
    }

    Joint* j = jointList;
    while (j)
    {
        Joint* j0 = j;
        j = j->next;
        Destroy(j0);
    }

    bodyList = nullptr;
    bodyListTail = nullptr;
    bodyCount = 0;

    jointList = nullptr;
    jointCount = 0;

    muliAssert(blockAllocator.GetBlockCount() == 0);

    destroyBufferBody.clear();
    destroyBufferJoint.clear();
}

void World::Destroy(RigidBody* body)
{
    muliAssert(body->world == this);

    Collider* c = body->colliderList;
    while (c)
    {
        Collider* c0 = c;
        c = c->next;
        body->DestoryCollider(c0);
    }

    JointEdge* je = body->jointList;
    while (je)
    {
        JointEdge* je0 = je;
        je = je->next;
        je0->other->Awake();

        Destroy(je0->joint);
    }

    if (body->next) body->next->prev = body->prev;
    if (body->prev) body->prev->next = body->next;
    if (body == bodyList) bodyList = body->next;
    if (body == bodyListTail) bodyListTail = body->prev;

    FreeBody(body);
    --bodyCount;
}

void World::Destroy(const std::vector<RigidBody*>& bodies)
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        Destroy(bodies[i]);
    }
}

void World::BufferDestroy(RigidBody* body)
{
    destroyBufferBody.push_back(body);
}

void World::BufferDestroy(const std::vector<RigidBody*>& bodies)
{
    std::unordered_set<RigidBody*> destroyed;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        RigidBody* b = bodies[i];

        if (destroyed.find(b) != destroyed.end())
        {
            destroyed.insert(b);
            BufferDestroy(b);
        }
    }
}

void World::Destroy(Joint* joint)
{
    RigidBody* bodyA = joint->bodyA;
    RigidBody* bodyB = joint->bodyB;

    // Remove from the world
    if (joint->prev) joint->prev->next = joint->next;
    if (joint->next) joint->next->prev = joint->prev;
    if (joint == jointList) jointList = joint->next;

    // Remove from bodyA
    if (joint->nodeA.prev) joint->nodeA.prev->next = joint->nodeA.next;
    if (joint->nodeA.next) joint->nodeA.next->prev = joint->nodeA.prev;
    if (&joint->nodeA == bodyA->jointList) bodyA->jointList = joint->nodeA.next;

    // Remove from bodyB
    if (joint->bodyA != joint->bodyB)
    {
        if (joint->nodeB.prev) joint->nodeB.prev->next = joint->nodeB.next;
        if (joint->nodeB.next) joint->nodeB.next->prev = joint->nodeB.prev;
        if (&joint->nodeB == bodyB->jointList) bodyB->jointList = joint->nodeB.next;
    }

    FreeJoint(joint);
    --jointCount;
}

void World::Destroy(const std::vector<Joint*>& joints)
{
    std::unordered_set<Joint*> destroyed;

    for (size_t i = 0; i < joints.size(); ++i)
    {
        Joint* j = joints[i];

        if (destroyed.find(j) != destroyed.end())
        {
            destroyed.insert(j);
            Destroy(j);
        }
    }
}

void World::BufferDestroy(Joint* joint)
{
    destroyBufferJoint.push_back(joint);
}

void World::BufferDestroy(const std::vector<Joint*>& joints)
{
    for (uint32 i = 0; i < joints.size(); ++i)
    {
        BufferDestroy(joints[i]);
    }
}

std::vector<Collider*> World::Query(const Vec2& point) const
{
    std::vector<Collider*> res;
    res.reserve(8);

    contactManager.broadPhase.tree.Query(point, [&](Collider* collider) -> bool {
        if (collider->TestPoint(point))
        {
            res.push_back(collider);
        }

        return true;
    });

    return res;
}

std::vector<Collider*> World::Query(const AABB& aabb) const
{
    std::vector<Collider*> res;
    res.reserve(8);

    Vec2 vertices[4] = { aabb.min, { aabb.max.x, aabb.min.y }, aabb.max, { aabb.min.x, aabb.max.y } };
    Polygon box{ vertices, 4, false, 0.0f };
    Transform t{ identity };

    contactManager.broadPhase.tree.Query(aabb, [&](Collider* collider) -> bool {
        if (DetectCollision(collider->shape, collider->body->transform, &box, t))
        {
            res.push_back(collider);
        }

        return true;
    });

    return res;
}

void World::Query(const Vec2& point, WorldQueryCallback* callback)
{
    struct TempCallback
    {
        Vec2 point;
        WorldQueryCallback* callback;

        bool QueryCallback(Collider* collider)
        {
            if (collider->TestPoint(point))
            {
                return callback->OnQuery(collider);
            }

            return true;
        }
    } tempCallback;

    tempCallback.point = point;
    tempCallback.callback = callback;

    contactManager.broadPhase.tree.Query(point, &tempCallback);
}

void World::Query(const AABB& aabb, WorldQueryCallback* callback)
{
    Vec2 vertices[4] = { aabb.min, { aabb.max.x, aabb.min.y }, aabb.max, { aabb.min.x, aabb.max.y } };
    Polygon box{ vertices, 4, false, 0.0f };

    struct TempCallback
    {
        Polygon region;
        WorldQueryCallback* callback;
        Transform t{ identity };

        TempCallback(const Polygon& box)
            : region{ box }
        {
        }

        bool QueryCallback(Collider* collider)
        {
            if (DetectCollision(collider->shape, collider->body->transform, &region, t))
            {
                return callback->OnQuery(collider);
            }

            return true;
        }
    } tempCallback(box);

    tempCallback.callback = callback;

    contactManager.broadPhase.tree.Query(aabb, &tempCallback);
}

void World::RayCastAny(
    const Vec2& from,
    const Vec2& to,
    const std::function<float(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback)
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;

    contactManager.broadPhase.tree.RayCast(input, [&](const RayCastInput& input, Collider* collider) -> float {
        RayCastOutput output;

        bool hit = collider->RayCast(input, &output);
        if (hit)
        {
            float fraction = output.fraction;
            Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

            return callback(collider, point, output.normal, fraction);
        }

        return input.maxFraction;
    });
}

bool World::RayCastClosest(
    const Vec2& from,
    const Vec2& to,
    const std::function<void(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback)
{
    bool hit = false;
    Collider* closestCollider;
    Vec2 closestPoint;
    Vec2 closestNormal;
    float closestFraction;

    RayCastAny(from, to, [&](Collider* collider, const Vec2& point, const Vec2& normal, float fraction) -> float {
        hit = true;
        closestCollider = collider;
        closestPoint = point;
        closestNormal = normal;
        closestFraction = fraction;

        return fraction;
    });

    if (hit)
    {
        callback(closestCollider, closestPoint, closestNormal, closestFraction);
        return true;
    }

    return false;
}

void World::RayCastAny(const Vec2& from, const Vec2& to, RayCastAnyCallback* callback)
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;

    struct TempCallback
    {
        RayCastAnyCallback* callback;

        float RayCastCallback(const RayCastInput& input, Collider* collider)
        {
            RayCastOutput output;

            bool hit = collider->RayCast(input, &output);
            if (hit)
            {
                float fraction = output.fraction;
                Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

                return callback->OnHit(collider, point, output.normal, fraction);
            }

            return input.maxFraction;
        }
    } anyCallback;

    anyCallback.callback = callback;

    contactManager.broadPhase.tree.RayCast(input, &anyCallback);
}

bool World::RayCastClosest(const Vec2& from, const Vec2& to, RayCastClosestCallback* callback)
{
    struct TempCallback : public RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestFraction;

        float OnHit(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;

            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, &tempCallback);

    if (tempCallback.hit)
    {
        callback->OnHit(tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal,
                        tempCallback.closestFraction);
        return true;
    }

    return false;
}

RigidBody* World::CreateEmptyBody(RigidBody::Type type)
{
    void* mem = blockAllocator.Allocate(sizeof(RigidBody));
    RigidBody* body = new (mem) RigidBody(type);

    body->world = this;

    if (bodyList == nullptr && bodyListTail == nullptr)
    {
        bodyList = body;
        bodyListTail = body;
    }
    else
    {
        bodyListTail->next = body;
        body->prev = bodyListTail;
        bodyListTail = body;
    }

    ++bodyCount;

    return body;
}

RigidBody* World::CreateCircle(float radius, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(type);

    Circle circle{ radius };
    b->CreateCollider(&circle);
    return b;
}

RigidBody* World::CreateCapsule(float length, float radius, bool horizontal, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(type);

    Capsule capsule{ length, radius, horizontal };
    b->CreateCollider(&capsule);
    return b;
}

RigidBody* World::CreateCapsule(
    const Vec2& p1, const Vec2& p2, float radius, RigidBody::Type type, bool resetPosition, float density)
{
    RigidBody* b = CreateEmptyBody(type);

    Vec2 center = (p1 + p2) * 0.5f;
    Capsule capsule{ p1, p2, radius, true };
    b->CreateCollider(&capsule);
    if (resetPosition == false)
    {
        b->Translate(center);
    }
    return b;
}

RigidBody* World::CreatePolygon(
    const std::vector<Vec2>& vertices, RigidBody::Type type, bool resetPosition, float radius, float density)
{
    RigidBody* b = CreateEmptyBody(type);

    Polygon polygon(vertices.data(), static_cast<int32>(vertices.size()), true, radius);
    b->CreateCollider(&polygon);

    Vec2 center{ 0.0f };
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        center += vertices[i];
    }
    center *= 1.0f / vertices.size();

    if (resetPosition == false)
    {
        b->Translate(center);
    }

    return b;
}

RigidBody* World::CreateBox(float width, float height, RigidBody::Type type, float radius, float density)
{
    RigidBody* b = CreateEmptyBody(type);

    Vec2 vertices[4] = { Vec2{ 0, 0 }, Vec2{ width, 0 }, Vec2{ width, height }, Vec2{ 0, height } };
    Polygon box{ vertices, 4, true, radius };
    b->CreateCollider(&box);
    return b;
}

RigidBody* World::CreateBox(float size, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size, size, type, radius, density);
}

RigidBody* World::CreateRandomConvexPolygon(float length, int32 vertexCount, RigidBody::Type type, float radius, float density)
{
    if (vertexCount < 3)
    {
        vertexCount = LinearRand(6, 12);
    }

    std::vector<float> angles;
    angles.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        angles.push_back(LinearRand(0.0f, 1.0f) * (MULI_PI * 2.0f - MULI_EPSILON));
    }

    std::sort(angles.begin(), angles.end());

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        vertices.emplace_back(Cos(angles[i]) * length, Sin(angles[i]) * length);
    }

    RigidBody* b = CreateEmptyBody(type);

    Polygon polygon{ vertices.data(), vertexCount, true, radius };
    b->CreateCollider(&polygon);
    return b;
}

RigidBody* World::CreateRegularPolygon(
    float length, int32 vertexCount, float initial_angle, RigidBody::Type type, float radius, float density)
{
    if (vertexCount < 3)
    {
        vertexCount = LinearRand(3, 12);
    }

    float angleStart = initial_angle;
    float angle = MULI_PI * 2.0f / vertexCount;

    if (vertexCount % 2 == 0)
    {
        angleStart += angle / 2.0f;
    }

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        float currentAngle = angleStart + angle * i;

        Vec2 corner = Vec2{ Cos(currentAngle), Sin(currentAngle) };
        corner *= length * Sqrt(2.0f);

        vertices.push_back(corner);
    }

    RigidBody* b = CreateEmptyBody(type);

    Polygon polygon{ vertices.data(), vertexCount, true, radius };
    b->CreateCollider(&polygon);
    return b;
}

GrabJoint* World::CreateGrabJoint(
    RigidBody* body, const Vec2& anchor, const Vec2& target, float frequency, float dampingRatio, float jointMass)
{
    if (body->world != this)
    {
        throw std::runtime_error("You should register the rigid bodies before registering the joint");
    }

    void* mem = blockAllocator.Allocate(sizeof(GrabJoint));
    GrabJoint* gj = new (mem) GrabJoint(body, anchor, target, settings, frequency, dampingRatio, jointMass);

    AddJoint(gj);
    return gj;
}

RevoluteJoint* World::CreateRevoluteJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, float frequency, float dampingRatio, float jointMass)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        throw std::runtime_error("You should register the rigid bodies before registering the joint");
    }

    void* mem = blockAllocator.Allocate(sizeof(RevoluteJoint));
    RevoluteJoint* rj = new (mem) RevoluteJoint(bodyA, bodyB, anchor, settings, frequency, dampingRatio, jointMass);

    AddJoint(rj);
    return rj;
}

DistanceJoint* World::CreateDistanceJoint(RigidBody* bodyA,
                                          RigidBody* bodyB,
                                          const Vec2& anchorA,
                                          const Vec2& anchorB,
                                          float length,
                                          float frequency,
                                          float dampingRatio,
                                          float jointMass)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        throw std::runtime_error("You should register the rigid bodies before registering the joint");
    }

    void* mem = blockAllocator.Allocate(sizeof(DistanceJoint));
    DistanceJoint* dj =
        new (mem) DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, settings, frequency, dampingRatio, jointMass);

    AddJoint(dj);
    return dj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA, RigidBody* bodyB, float length, float frequency, float dampingRatio, float jointMass)
{
    return CreateDistanceJoint(bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), length, frequency, dampingRatio,
                               jointMass);
}

AngleJoint* World::CreateAngleJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio, float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(AngleJoint));
    AngleJoint* aj = new (mem) AngleJoint(bodyA, bodyB, settings, frequency, dampingRatio, jointMass);

    AddJoint(aj);
    return aj;
}

WeldJoint* World::CreateWeldJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio, float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(WeldJoint));
    WeldJoint* wj = new (mem) WeldJoint(bodyA, bodyB, (bodyA->GetPosition() + bodyB->GetPosition()) * 0.5f, settings, frequency,
                                        dampingRatio, jointMass);

    AddJoint(wj);
    return wj;
}

LineJoint* World::CreateLineJoint(
    RigidBody* bodyA, RigidBody* bodyB, Vec2 anchor, Vec2 dir, float frequency, float dampingRatio, float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(LineJoint));
    LineJoint* lj = new (mem) LineJoint(bodyA, bodyB, anchor, dir, settings, frequency, dampingRatio, jointMass);

    AddJoint(lj);
    return lj;
}

LineJoint* World::CreateLineJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio, float jointMass)
{
    return CreateLineJoint(bodyA, bodyB, bodyA->GetPosition(), (bodyB->GetPosition() - bodyA->GetPosition()).Normalized(),
                           frequency, dampingRatio, jointMass);
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, const Vec2& dir, float frequency, float dampingRatio, float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(PrismaticJoint));
    PrismaticJoint* pj = new (mem) PrismaticJoint(bodyA, bodyB, anchor, dir, settings, frequency, dampingRatio, jointMass);

    AddJoint(pj);
    return pj;
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio, float jointMass)
{
    return CreatePrismaticJoint(bodyA, bodyB, bodyB->GetPosition(), (bodyB->GetPosition() - bodyA->GetPosition()).Normalized(),
                                frequency, dampingRatio, jointMass);
}

PulleyJoint* World::CreatePulleyJoint(RigidBody* bodyA,
                                      RigidBody* bodyB,
                                      const Vec2& anchorA,
                                      const Vec2& anchorB,
                                      const Vec2& groundAnchorA,
                                      const Vec2& groundAnchorB,
                                      float ratio,
                                      float frequency,
                                      float dampingRatio,
                                      float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(PulleyJoint));
    PulleyJoint* pj = new (mem) PulleyJoint(bodyA, bodyB, anchorA, anchorB, groundAnchorA, groundAnchorB, settings, ratio,
                                            frequency, dampingRatio, jointMass);

    AddJoint(pj);
    return pj;
}

MotorJoint* World::CreateMotorJoint(RigidBody* bodyA,
                                    RigidBody* bodyB,
                                    const Vec2& anchor,
                                    float maxForce,
                                    float maxTorque,
                                    float frequency,
                                    float dampingRatio,
                                    float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(MotorJoint));
    MotorJoint* mj =
        new (mem) MotorJoint(bodyA, bodyB, anchor, settings, maxForce, maxTorque, frequency, dampingRatio, jointMass);

    AddJoint(mj);
    return mj;
}

void World::AddJoint(Joint* joint)
{
    // Insert into the world
    joint->prev = nullptr;
    joint->next = jointList;
    if (jointList != nullptr)
    {
        jointList->prev = joint;
    }
    jointList = joint;

    // Connect to island graph

    // Connect joint edge to body A
    joint->nodeA.joint = joint;
    joint->nodeA.other = joint->bodyB;

    joint->nodeA.prev = nullptr;
    joint->nodeA.next = joint->bodyA->jointList;
    if (joint->bodyA->jointList != nullptr)
    {
        joint->bodyA->jointList->prev = &joint->nodeA;
    }
    joint->bodyA->jointList = &joint->nodeA;

    // Connect joint edge to body B
    if (joint->bodyA != joint->bodyB)
    {
        joint->nodeB.joint = joint;
        joint->nodeB.other = joint->bodyA;

        joint->nodeB.prev = nullptr;
        joint->nodeB.next = joint->bodyB->jointList;
        if (joint->bodyB->jointList != nullptr)
        {
            joint->bodyB->jointList->prev = &joint->nodeB;
        }
        joint->bodyB->jointList = &joint->nodeB;
    }

    ++jointCount;
}

void World::FreeBody(RigidBody* body)
{
    body->~RigidBody();
    blockAllocator.Free(body, sizeof(RigidBody));
}

void World::FreeJoint(Joint* joint)
{
    joint->~Joint();

    switch (joint->type)
    {
    case Joint::Type::grab_joint:
        blockAllocator.Free(joint, sizeof(GrabJoint));
        break;
    case Joint::Type::revolute_joint:
        blockAllocator.Free(joint, sizeof(RevoluteJoint));
        break;
    case Joint::Type::distance_joint:
        blockAllocator.Free(joint, sizeof(DistanceJoint));
        break;
    case Joint::Type::angle_joint:
        blockAllocator.Free(joint, sizeof(AngleJoint));
        break;
    case Joint::Type::weld_joint:
        blockAllocator.Free(joint, sizeof(WeldJoint));
        break;
    case Joint::Type::line_joint:
        blockAllocator.Free(joint, sizeof(LineJoint));
        break;
    case Joint::Type::prismatic_joint:
        blockAllocator.Free(joint, sizeof(PrismaticJoint));
        break;
    case Joint::Type::pulley_joint:
        blockAllocator.Free(joint, sizeof(PulleyJoint));
        break;
    case Joint::Type::motor_joint:
        blockAllocator.Free(joint, sizeof(MotorJoint));
        break;
    default:
        muliAssert(false);
        break;
    }
}

} // namespace muli