#pragma once

#include "shape.h"

#define MAX_LOCAL_POLYGON_VERTICES 8

namespace muli
{

class Polygon : public Shape
{
public:
    Polygon(const Vec2* vertices, int32 vertexCount, bool resetPosition = false, float radius = DEFAULT_RADIUS);
    Polygon(std::initializer_list<Vec2> vertices, bool resetPosition = false, float radius = DEFAULT_RADIUS);
    Polygon(float width, float height, float radius = DEFAULT_RADIUS, const Vec2& position = Vec2{ 0.0f }, float angle = 0.0f);
    Polygon(float size, float radius = DEFAULT_RADIUS, const Vec2& position = Vec2{ 0.0f }, float angle = 0.0f);
    ~Polygon();

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual ContactPoint Support(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;
    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

    const Vec2* GetVertices() const;
    const Vec2* GetNormals() const;
    int32 GetVertexCount() const;
    float GetArea() const;

protected:
    virtual Shape* Clone(Allocator* allocator) const override;

    Vec2* vertices;
    Vec2* normals;
    int32 vertexCount;

private:
    Vec2 localVertices[MAX_LOCAL_POLYGON_VERTICES];
    Vec2 localNormals[MAX_LOCAL_POLYGON_VERTICES];
};

inline const Vec2* Polygon::GetVertices() const
{
    return vertices;
}

inline const Vec2* Polygon::GetNormals() const
{
    return normals;
}

inline int32 Polygon::GetVertexCount() const
{
    return vertexCount;
}

inline float Polygon::GetArea() const
{
    return area;
}

} // namespace muli
