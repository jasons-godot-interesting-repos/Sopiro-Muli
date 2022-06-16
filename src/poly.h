#ifndef __POLY_H__
#define __POLY_H__

#include "common.h"

class Poly
{
public:
    Poly() = delete;
    Poly(Poly& other) = delete;

    Poly(const std::vector<glm::vec3>& _vertices, const std::vector<uint32_t>& _indices);
    ~Poly();

    void draw();

private:
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices;

    uint32_t VBO;
    uint32_t VAO;
    uint32_t EBO;
};

#endif