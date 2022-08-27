#include "spe/polygon.h"

namespace spe
{

Polygon::Polygon(std::vector<glm::vec2> _vertices, BodyType _type, bool _resetPosition, float _density) :
    RigidBody(std::move(_type)),
    vertices{ std::move(_vertices) }
{
    glm::vec2 centerOfMass{ 0.0f };
    size_t count = vertices.size();

    for (size_t i = 0; i < count; i++)
    {
        centerOfMass += vertices[i];
    }

    centerOfMass /= count;

    float _area = 0;

    vertices[0] -= centerOfMass;
    radius = glm::length(vertices[0]);

    for (uint32_t i = 1; i < count; i++)
    {
        vertices[i] -= centerOfMass;
        radius = glm::max(radius, glm::length(vertices[i]));
        _area += glm::cross(vertices[i - 1], vertices[i]);
    }
    _area += glm::cross(vertices[count - 1], vertices[0]);

    area = glm::abs(_area) / 2.0f;

    if (type == Dynamic)
    {
        assert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = calculate_convex_polygon_inertia(vertices, mass, area);
        invInertia = 1.0f / inertia;
    }

    if (!_resetPosition)
    {
        Translate(centerOfMass);
    }

    shape = BodyShape::ShapePolygon;
}

}