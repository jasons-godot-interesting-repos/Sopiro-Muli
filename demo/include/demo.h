#pragma once

#include "common.h"

namespace spe
{

static void demo1(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    RigidBody* box = world.CreateBox(0.4f);
    box->SetPosition(0.0f, 5.0f);
    box->SetAngularVelocity(glm::linearRand(-12.0f, 12.0f));
}

static void demo2(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    float start = 0.5f;
    float size = 0.3f;
    float gap = 0.25f;

    // float error = 0.015f;
    float error = 0.0f;

    for (uint32_t i = 0; i < 20; i++)
    {
        RigidBody* b = world.CreateBox(size);
        b->SetPosition(glm::linearRand(-error, error), start + i * (size + gap));
    }
}

static void demo3(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    int32_t rows = 15;
    float boxSize = 0.4f;
    float xGap = 0.0625f * boxSize / 0.5f;
    float yGap = 0.125f * boxSize / 0.5f;
    float xStart = -(rows - 1.0f) * (boxSize + xGap) / 2.0f;
    float yStart = 0.2f + boxSize / 2.0f + yGap;

    for (int y = 0; y < rows; y++)
    {
        for (int x = 0; x < rows - y; x++)
        {
            RigidBody* b = world.CreateBox(boxSize);
            b->SetPosition(xStart + y * (boxSize + xGap) / 2.0f + x * (boxSize + xGap), yStart + y * (boxSize + yGap));
        }
    }
}

static void demo4(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    Box* b = world.CreateBox(0.3f);
    b->SetPosition(-3.0f, 5.0f);

    world.CreateRevoluteJoint(ground, b, glm::vec2(0.0f, 5.0f), -1.0f);
}

static void demo5(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = false;

    Box* g = world.CreateBox(0.3f, 6, RigidBody::Type::Static);
    g->SetPosition(0.0f, 3.6f);

    Box* b = world.CreateBox(0.3f);
    b->SetPosition(3.0f, 3.6f + 2.0f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 1.0f, 0.05f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->SetPosition(3.0f, 3.6f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 1.0f, 0.2f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->SetPosition(3.0f, 3.6f - 2.0f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 1.0f, 0.7f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->SetPosition(-3.0f, 3.6f + 2.0f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 0.5f, 0.2f, b->GetMass());

    // Reduce the amplitude by half every second
    float halfLife = 1.0f;
    float frequency = -glm::log(0.5f) / (halfLife * glm::pi<float>() * 2.0f);

    b = world.CreateBox(0.3f);
    b->SetPosition(-3.0f, 3.6f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, frequency, 1.0f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->SetPosition(-3.0f, 3.6f - 2.0f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 2.0f, 0.01f, b->GetMass());
}

static void demo6(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    int rows = 12;
    float size = 0.25f;
    float xGap = 0.2f;
    float yGap = 0.15f;
    float xStart = -(rows - 1) * (size + xGap) / 2.0f;
    float yStart = 1.0f;

    for (int y = 0; y < rows; y++)
    {
        for (int x = 0; x < rows - y; x++)
        {
            Polygon* b = world.CreateRandomConvexPolygon(size, 6);
            b->SetPosition(xStart + y * (size + xGap) / 2 + x * (size + xGap), yStart + y * (size + yGap));
            b->SetLinearVelocity(b->GetPosition() * glm::linearRand(0.5f, 0.7f));
            b->SetFriction(glm::linearRand(0.2f, 1.0f));
        }
    }

    Box* pillar = world.CreateBox(0.25f, 4.0f, RigidBody::Type::Static);
    pillar->SetPosition(xStart - 0.2f, 3.0f);

    pillar = world.CreateBox(0.25f, 4.0f, RigidBody::Type::Static);
    pillar->SetPosition(-(xStart - 0.2f), 3.0f);
}

static void demo7(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    Box* seesaw = world.CreateBox(6.0f, 0.1f);
    seesaw->SetPosition(0.0f, 0.45f);
    seesaw->SetMass(10.0f);

    world.CreateRevoluteJoint(ground, seesaw, seesaw->GetPosition(), -1);

    RigidBody* b = world.CreateCircle(0.2f);
    b->SetPosition(-2.5f, 1.0f);

    b = world.CreateBox(0.2f);
    b->SetPosition(-2.8f, 1.0f);
    b->SetMass(1.0f);

    b = world.CreateBox(0.5f);
    b->SetPosition(2.5f, 5.0f);
    b->SetMass(30.0f);
}

static void demo8(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    Box* b = world.CreateBox(6.0f, 0.1f, RigidBody::Type::Static);
    b->SetPosition(-0.6f, 5.0f);
    b->SetRotation(-0.15f);
    b->SetFriction(1.0f);

    b = world.CreateBox(6.0f, 0.1f, RigidBody::Type::Static);
    b->SetPosition(0.0f, 3.0f);
    b->SetRotation(0.15f);
    b->SetFriction(1.0f);

    b = world.CreateBox(6.0f, 0.1f, RigidBody::Type::Static);
    b->SetPosition(-0.6f, 1.0f);
    b->SetRotation(-0.15f);
    b->SetFriction(1.0f);

    b = world.CreateBox(0.1f, 1.1f, RigidBody::Type::Static);
    b->SetPosition(3.1f, 4.3f);
    b = world.CreateBox(0.1f, 1.1f, RigidBody::Type::Static);
    b->SetPosition(-3.7f, 2.3f);

    float xStart = -4.5f;
    float yStart = 7.0f;
    float gap = 0.30f;
    float size = 0.30f;

    std::array<float, 5> frictions = { 0.51f, 0.31f, 0.21f, 0.11f, 0.0f };

    for (uint32_t i = 0; i < frictions.size(); i++)
    {
        b = world.CreateBox(size, size);
        b->SetPosition(xStart + (size + gap) * i, yStart);
        b->SetFriction(frictions[i]);
        b->SetLinearVelocity({ 2.0f, 0.0f });
    }
}

static void demo9(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    int count = 11;
    float gap = 0.5f;
    float size = 0.3f;

    float xStart = -(count - 1) / 2 * gap;
    float yStart = 6.0f;

    bool r = glm::linearRand(0.0f, 1.0f) > 0.5f;

    RigidBody* b;
    for (int i = 0; i < count; i++)
    {
        if (r)
            b = world.CreateBox(size);
        else
            b = world.CreateCircle(size / 2.0f);
        b->SetPosition(xStart + gap * i, yStart);
        float attenuation = (count - i) / (float)count;
        b->SetRestitution(1.0f - attenuation * attenuation);
    }
}

static void demo10(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    float xStart = 0.0f;
    float yStart = 5.0f;
    float sizeW = 0.3f;
    float sizeH = 0.15f;
    float gap = 0.1f;

    RigidBody* b1 = world.CreateBox(sizeW, sizeH);
    b1->SetMass(1.0);
    b1->SetPosition(xStart - (gap + sizeW), yStart);

    Joint* j = world.CreateRevoluteJoint(ground, b1, { xStart, yStart }, -1.0f);

    bool t = glm::linearRand<float>(0.0f, 1.0f) > 0.5;

    for (int i = 1; i < 12; i++)
    {
        RigidBody* b2 = world.CreateBox(sizeW, sizeH);
        b2->SetMass(1.0f);
        b2->SetPosition(xStart - (gap + sizeW) * (i + 1), yStart);

        if (t)
        {
            j = world.CreateRevoluteJoint(b1, b2, { xStart - (sizeW + gap) / 2 - (gap + sizeW) * i, yStart }, 15.0f, 0.5f);
        }
        else
        {
            j = world.CreateDistanceJoint(b1, b2, b1->GetPosition() - glm::vec2{ sizeW / 2, 0 },
                                          b2->GetPosition() + glm::vec2{ sizeW / 2, 0 });
        }

        b1 = b2;
    }
}

static void demo11(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    float groundStart = 0.2f;

    float xStart = -5.0f;
    float yStart = 4.0f;
    float gap = 0.1f;

    float pillarWidth = 0.3f;
    float sizeX = 0.5f;
    float sizeY = sizeX * 0.25f;

    Box* pillar = world.CreateBox(pillarWidth, yStart, RigidBody::Type::Static);
    pillar->SetPosition(xStart, yStart / 2 + 0.2f);

    Box* b1 = world.CreateBox(sizeX, sizeY);
    b1->SetMass(10.0f);
    b1->SetPosition(xStart + sizeX / 2 + pillarWidth / 2 + gap, yStart + groundStart);

    Joint* j;

    bool revoluteBridge = glm::linearRand<float>(0.0f, 1.0f) > 0.5;
    float frequency = 30.0f;

    if (revoluteBridge)
    {
        j = world.CreateRevoluteJoint(pillar, b1, pillar->GetPosition() + glm::vec2(pillarWidth, yStart) / 2.0f, frequency, 1.0f);
    }
    else
    {
        j = world.CreateDistanceJoint(pillar, b1, pillar->GetPosition() + glm::vec2(pillarWidth / 2.0f, yStart / 2.0f),
                                      b1->GetPosition() + glm::vec2(-sizeX / 2, 0.03), -1.0f, frequency, 1.0f);
        j = world.CreateDistanceJoint(pillar, b1, pillar->GetPosition() + glm::vec2(pillarWidth / 2.0f, yStart / 2.0f),
                                      b1->GetPosition() + glm::vec2(-sizeX / 2, -0.03), -1.0f, frequency, 1.0f);
    }

    for (int i = 1; i + 1 < xStart * -2 / (sizeX + gap); i++)
    {
        Box* b2 = world.CreateBox(sizeX, sizeY);
        b2->SetMass(10.0f);
        b2->SetPosition(xStart + sizeX / 2.0f + pillarWidth / 2.0f + gap + (gap + sizeX) * i, yStart + groundStart);

        if (revoluteBridge)
        {
            j = world.CreateRevoluteJoint(b1, b2, (b1->GetPosition() + b2->GetPosition()) / 2.0f, frequency, 1.0f);
        }
        else
        {
            j = world.CreateDistanceJoint(b1, b2, b1->GetPosition() + glm::vec2(sizeX / 2.0f, 0.03f),
                                          b2->GetPosition() + glm::vec2(-sizeX / 2.0f, 0.03f), -1.0f, frequency, 1.0f);
            j = world.CreateDistanceJoint(b1, b2, b1->GetPosition() + glm::vec2(sizeX / 2.0f, -0.03f),
                                          b2->GetPosition() + glm::vec2(-sizeX / 2.0f, -0.03f), -1.0f, frequency, 1.0f);
        }

        b1 = b2;
    }

    pillar = world.CreateBox(pillarWidth, yStart, RigidBody::Type::Static);
    pillar->SetPosition(-xStart, yStart / 2.0f + 0.2f);

    if (revoluteBridge)
    {
        j = world.CreateRevoluteJoint(pillar, b1, pillar->GetPosition() + glm::vec2(-pillarWidth, yStart) / 2.0f, frequency,
                                      1.0f);
    }
    else
    {
        j = world.CreateDistanceJoint(pillar, b1, pillar->GetPosition() + glm::vec2(-pillarWidth / 2.0f, yStart / 2.0f),
                                      b1->GetPosition() + glm::vec2(sizeX / 2, 0.03), -1, frequency, 1.0f);
        j = world.CreateDistanceJoint(pillar, b1, pillar->GetPosition() + glm::vec2(-pillarWidth / 2.0f, yStart / 2.0f),
                                      b1->GetPosition() + glm::vec2(sizeX / 2, -0.03), -1, frequency, 1.0f);
    }

    Camera& camera = game.GetCamera();
    camera.position = glm::vec2{ 0, 3.6f + 1.8f };
    camera.scale = glm::vec2{ 1.5f, 1.5f };
}

static void demo12(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, RigidBody::Type::Static);

    float xStart = -3.0f;
    float yStart = 1.0f;
    float size = 0.3f;
    float gap = 0.3f;

    int rows = 10;

    for (int i = 0; i < rows; i++)
    {
        for (int j = i; j < rows; j++)
        {
            Circle* c = world.CreateCircle(size);
            c->SetMass((1 + i) + (1 + i) * j + 2.0f);
            c->SetPosition(xStart + (gap + size * 2) * i, yStart + (gap + size * 2) * j);
        }
    }
}

static void demo13(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;

    float size = 15.0f;
    float wallSize = 0.4f;

    RigidBody* wall = world.CreateBox(size, wallSize, RigidBody::Type::Static);
    wall->SetPosition(0.0f, -size / 2.0f);
    wall = world.CreateBox(size, wallSize, RigidBody::Type::Static);
    wall->SetPosition(0.0f, size / 2.0f);
    wall = world.CreateBox(wallSize, size, RigidBody::Type::Static);
    wall->SetPosition(-size / 2.0f, 0.0f);
    wall = world.CreateBox(wallSize, size, RigidBody::Type::Static);
    wall->SetPosition(size / 2.0f, 0.0f);

    float r = 0.2f;

    for (int i = 0; i < 1000; i++)
    {
        RigidBody* b = world.CreateCircle(r);
        b->SetPosition(glm::linearRand<float>(0.0f, size - wallSize) - (size - wallSize) / 2.0f,
                       glm::linearRand<float>(0.0f, size - wallSize) - (size - wallSize) / 2.0f);
        b->SetRotation(glm::linearRand<float>(0.0f, glm::pi<float>() * 2.0f));
    }

    Camera& c = game.GetCamera();
    c.position = { 0.0f, 0.0f };
    c.scale = { 3.f, 3.f };
}

static void demo14(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;

    float size = 15.0f;
    float wallSize = 0.4f;

    RigidBody* wall = world.CreateBox(size, wallSize, RigidBody::Type::Static);
    wall->SetPosition(0.0f, -size / 2.0f);
    wall = world.CreateBox(size, wallSize, RigidBody::Type::Static);
    wall->SetPosition(0.0f, size / 2.0f);
    wall = world.CreateBox(wallSize, size, RigidBody::Type::Static);
    wall->SetPosition(-size / 2.0f, 0.0f);
    wall = world.CreateBox(wallSize, size, RigidBody::Type::Static);
    wall->SetPosition(size / 2.0f, 0.0f);

    float r = 0.38f;

    for (int i = 0; i < 1000; i++)
    {
        RigidBody* b = world.CreateBox(r);
        b->SetPosition(glm::linearRand<float>(0.0f, size - wallSize) - (size - wallSize) / 2.0f,
                       glm::linearRand<float>(0.0f, size - wallSize) - (size - wallSize) / 2.0f);
    }

    Camera& c = game.GetCamera();
    c.position = { 0.0f, 0.0f };
    c.scale = { 3.f, 3.f };
}

static void demo15(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;

    float size = 15.0f;
    float wallSize = 0.4f;

    RigidBody* wall = world.CreateBox(size, wallSize, RigidBody::Type::Static);
    wall->SetPosition(0.0f, -size / 2.0f);
    wall = world.CreateBox(size, wallSize, RigidBody::Type::Static);
    wall->SetPosition(0.0f, size / 2.0f);
    wall = world.CreateBox(wallSize, size, RigidBody::Type::Static);
    wall->SetPosition(-size / 2.0f, 0.0f);
    wall = world.CreateBox(wallSize, size, RigidBody::Type::Static);
    wall->SetPosition(size / 2.0f, 0.0f);

    float r = 0.28f;

    for (int i = 0; i < 1000; i++)
    {
        RigidBody* b = world.CreateRandomConvexPolygon(r, 7);
        b->SetPosition(glm::linearRand<float>(0.0f, size - wallSize) - (size - wallSize) / 2.0f,
                       glm::linearRand<float>(0.0f, size - wallSize) - (size - wallSize) / 2.0f);
    }

    Camera& c = game.GetCamera();
    c.position = { 0.0f, 0.0f };
    c.scale = { 3.f, 3.f };
}

static void demo16(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;

    float size = 15.0f;
    float wallSize = 0.4f;

    RigidBody* wall = world.CreateBox(size, wallSize, RigidBody::Type::Static);
    wall->SetPosition(0.0f, -size / 2.0f);
    wall = world.CreateBox(size, wallSize, RigidBody::Type::Static);
    wall->SetPosition(0.0f, size / 2.0f);
    wall = world.CreateBox(wallSize, size, RigidBody::Type::Static);
    wall->SetPosition(-size / 2.0f, 0.0f);
    wall = world.CreateBox(wallSize, size, RigidBody::Type::Static);
    wall->SetPosition(size / 2.0f, 0.0f);

    Camera& c = game.GetCamera();
    c.position = { 0.0f, 0.0f };
    c.scale = { 3.f, 3.f };
}

std::vector<std::pair<std::string, std::function<void(Game&, World&, Settings&)>>> get_demos()
{
    decltype(get_demos()) demos;
    demos.reserve(12);

    demos.push_back({ "Single Box", demo1 });
    demos.push_back({ "Box stacking", demo2 });
    demos.push_back({ "Pyramid", demo3 });
    demos.push_back({ "Single pendulum", demo4 });
    demos.push_back({ "Springs", demo5 });
    demos.push_back({ "Random convex shapes", demo6 });
    demos.push_back({ "Seesaw", demo7 });
    demos.push_back({ "Friction test", demo8 });
    demos.push_back({ "Restitution test", demo9 });
    demos.push_back({ "Multi pendulum", demo10 });
    demos.push_back({ "Suspension bridge", demo11 });
    demos.push_back({ "Circle stacking", demo12 });
    demos.push_back({ "1000 circles", demo13 });
    demos.push_back({ "1000 boxes", demo14 });
    demos.push_back({ "1000 convex shapes", demo15 });
    demos.push_back({ "test", demo16 });

    return demos;
}

} // namespace spe