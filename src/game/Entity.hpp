#pragma once

#include <cmath>
#include <glm/glm.hpp>
#include <array>
#include <cstdint>
#include <limits>

using id_t = uint32_t;

constexpr id_t kInvalidId = std::numeric_limits<id_t>::max();
struct Entity {
    glm::vec2 pos;
    glm::vec2 vel;
    float size = 0.1f;
    uint32_t color = 0xffffffff;
    id_t id = kInvalidId;
    uint8_t teleport_count = 0;

    void simulate(float dt);

    static id_t firstFreeId;
    static Entity create();
    static glm::vec2 randomPos();
};
