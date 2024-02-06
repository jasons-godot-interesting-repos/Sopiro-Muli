#include "muli/distance.h"
#include "muli/collision.h"
#include "muli/util.h"

namespace muli
{

extern SupportPoint CSOSupport(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Vec2& dir);

float GetClosestFeatures(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ClosestFeatures* features)
{
    GJKResult gjkResult;

    bool collide = GJK(a, tfA, b, tfB, &gjkResult);
    if (collide == true)
    {
        return 0.0f;
    }

    Simplex& simplex = gjkResult.simplex;
    muliAssert(simplex.count < max_simplex_vertex_count);

    features->count = simplex.count;
    for (int32 i = 0; i < features->count; ++i)
    {
        features->featuresA[i] = simplex.vertices[i].pointA;
        features->featuresB[i] = simplex.vertices[i].pointB;
    }

    return gjkResult.distance;
}

float ComputeDistance(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, Vec2* pointA, Vec2* pointB)
{
    GJKResult gjkResult;

    bool collide = GJK(a, tfA, b, tfB, &gjkResult);
    if (collide == true)
    {
        return 0.0f;
    }

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    if (gjkResult.distance < radii)
    {
        return 0.0f;
    }

    Simplex& simplex = gjkResult.simplex;

    muliAssert(simplex.count < max_simplex_vertex_count);

    simplex.GetWitnessPoint(pointA, pointB);

    // displace simplex vertices along normal
    *pointA += gjkResult.direction * ra;
    *pointB -= gjkResult.direction * rb;

    return gjkResult.distance - radii;
}

bool ShapeCast(const Shape* a,
               const Transform& tfA,
               const Shape* b,
               const Transform& tfB,
               const Vec2& translationA,
               const Vec2& translationB,
               ShapeCastOutput* output)
{
    output->point.SetZero();
    output->normal.SetZero();
    output->t = 1.0f;

    float t = 0.0f;
    Vec2 n = Vec2::zero;

    const float radii = a->GetRadius() + b->GetRadius();
    const Vec2 r = translationB - translationA; // Ray vector

    Simplex simplex;

    // Get CSO support point in inverse ray direction
    int32 idA = a->GetSupport(MulT(tfA.rotation, -r));
    Vec2 pointA = Mul(tfA, a->GetVertex(idA));
    int32 idB = b->GetSupport(MulT(tfB.rotation, r));
    Vec2 pointB = Mul(tfB, b->GetVertex(idB));
    Vec2 v = pointA - pointB;

    const float target = Max(default_radius, radii - toi_position_solver_threshold);
    const float tolerance = linear_slop * 0.1f;

    const int32 maxIterations = 20;
    int32 iteration = 0;

    while (iteration < maxIterations && v.Length() - target > tolerance)
    {
        muliAssert(simplex.count < 3);

        // Get CSO support point in search direction(-v)
        idA = a->GetSupport(MulT(tfA.rotation, -v));
        pointA = Mul(tfA, a->GetVertex(idA));
        idB = b->GetSupport(MulT(tfB.rotation, v));
        pointB = Mul(tfB, b->GetVertex(idB));
        Vec2 p = pointA - pointB; // Outer vertex of CSO

        // -v is the plane normal at p
        v.Normalize();

        // Find intersection with support plane
        float vp = Dot(v, p);
        float vr = Dot(v, r);

        // March ray by (vp - target) / vr if the new t is greater
        if (vp - target > t * vr)
        {
            if (vr <= 0.0f)
            {
                return false;
            }

            t = (vp - target) / vr;
            if (t > 1.0f)
            {
                return false;
            }

            n = -v;
            simplex.count = 0;
        }

        SupportPoint* vertex = simplex.vertices + simplex.count;
        vertex->pointA.id = idA;
        vertex->pointA.p = pointA;
        vertex->pointB.id = idB;
        vertex->pointB.p = pointB + t * r; // This effectively shifts the ray origin to the new clip plane
        vertex->point = vertex->pointA.p - vertex->pointB.p;
        vertex->weight = 1.0f;
        simplex.count += 1;

        simplex.Advance(Vec2::zero);

        if (simplex.count == 3)
        {
            // Initial overlap
            return false;
        }

        // Update search direciton
        v = simplex.GetClosestPoint();

        ++iteration;
    }

    if (iteration == 0 || t == 0.0f)
    {
        // Initial overlap
        return false;
    }

    simplex.GetWitnessPoint(&pointA, &pointB);

    if (v.Length2() > 0.0f)
    {
        n = -v;
        n.Normalize();
    }

    output->point = pointA + a->GetRadius() * n + translationA * t;
    output->normal = n;
    output->t = t;
    return true;
}

} // namespace muli
