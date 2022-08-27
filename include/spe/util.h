#pragma once

#include "common.h"
#include "rigidbody.h"
#include "settings.h"

namespace spe
{

struct BodyPair
{
    uint32_t first;
    uint32_t second;
};

union PairID
{
    BodyPair pair;
    uint64_t key;
};

struct UV
{
    float u;
    float v;
};

float calculate_convex_polygon_inertia(const std::vector<glm::vec2>& vertices, float mass, float area);

inline float calculate_box_inertia(float width, float height, float mass)
{
    return (width * width + height * height) * mass / 12.0f;
}

inline float calculate_circle_inertia(float radius, float mass)
{
    return mass * radius * radius / 2.0f;
}

// Project point P to line segment AB, calculate barycentric weights
inline UV get_uv(glm::vec2 a, glm::vec2 b, glm::vec2 p)
{
    glm::vec2 dir = b - a;
    float len = glm::length(dir);
    dir = glm::normalize(dir);

    float region = glm::dot(dir, p - a) / len;

    return { 1 - region,  region };
}

// Linearly combine(interpolate) the vector using weights u, v
inline glm::vec2 lerp_vector(glm::vec2 a, glm::vec2 b, UV uv)
{
    return { a.x * uv.u + b.x * uv.v, a.y * uv.u + b.y * uv.v };
}

// Cantor pairing function, ((N, N) -> N) mapping function
// https://en.wikipedia.org/wiki/Pairing_function#Cantor_pairing_function
inline uint32_t make_pair_natural(uint32_t a, uint32_t b)
{
    return (a + b) * (a + b + 1) / 2 + b;
}

inline PairID combine_id(uint32_t a, uint32_t b)
{
    assert(a != b);
    return a < b ? PairID{ a, b } : PairID{ b, a };
}

inline bool operator==(PairID lhs, PairID rhs)
{
    return lhs.key == rhs.key;
}

// Reverse version of pairing function
// this guarantees initial pairing order
inline std::pair<uint32_t, uint32_t> separate_pair(uint32_t p)
{
    double w = glm::floor((glm::sqrt(8 * p + 1) - 1) / 2.0);
    double t = (w * w + w) / 2.0;

    double y = p - t;
    double x = w - y;

    return { static_cast<uint32_t>(x), static_cast<uint32_t>(y) };
}

inline float lerp(float left, float right, float per)
{
    return left + (right - left) * per;
}

inline float map(float v, float left, float right, float min, float max)
{
    float per = (v - left) / (right - left);

    return lerp(min, max, per);
}

inline glm::vec2 mid(glm::vec2 a, glm::vec2 b)
{
    return (a + b) / 2.0f;
}

// https://gist.github.com/ciembor/1494530
/*
 * Converts an RGB color value to HSL. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes r, g, and b are contained in the set [0, 255] and
 * returns HSL in the set [0, 1].
 */
inline glm::vec3 rgb2hsl(float r, float g, float b)
{
    r /= 255.0f;
    g /= 255.0f;
    b /= 255.0f;

    float max = glm::max(glm::max(r, g), b);
    float min = glm::min(glm::min(r, g), b);

    glm::vec3 res{ (max + min) / 2.0f };

    if (max == min)
    {
        // achromatic
        res.x = 0.0f;
        res.y = 0.0f;
    }
    else
    {
        float d = max - min;
        res.s = (res.z > 0.5f) ? d / (2.0f - max - min) : d / (max + min);

        if (max == r) res.x = (g - b) / d + (g < b ? 6 : 0);
        else if (max == g) res.x = (b - r) / d + 2;
        else if (max == b) res.x = (r - g) / d + 4;

        res.x /= 6;
    }

    return res;
}

/*
 * Converts an HUE to r, g or b.
 * returns float in the set [0, 1].
 */
inline float hue2rgb(float p, float q, float t)
{
    if (t < 0.0f) t += 1.0f;
    if (t > 1.0f) t -= 1.0f;
    if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
    if (t < 1.0f / 2.0f) return q;
    if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;

    return p;
}

/*
 * Converts an HSL color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes h, s, and l are contained in the set [0, 1] and
 * returns RGB in the set [0, 1].
 */
inline glm::vec3 hsl2rgb(float h, float s, float l)
{
    glm::vec3 res;

    if (s == 0.0f)
    {
        res.r = res.g = res.b = l; // achromatic
    }
    else
    {
        float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
        float p = 2.0f * l - q;
        res.r = hue2rgb(p, q, h + 1.0f / 3.0f);
        res.g = hue2rgb(p, q, h);
        res.b = hue2rgb(p, q, h - 1.0f / 3.0f);
    }

    return res;
}

template<typename T>
inline void log(T msg)
{
    std::cout << msg << '\n';
}

}
