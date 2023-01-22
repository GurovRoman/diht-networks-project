#include "Entity.hpp"
#include <random>

id_t Entity::firstFreeId = 0;
static std::default_random_engine engine;
static std::uniform_int_distribution<uint32_t> colorDistr(0xffffff);
static std::uniform_real_distribution<float> coordDistr(0, 1);
constexpr float MIN_SIZE = 0.001;
constexpr float MAX_SIZE = 1.f;
static std::uniform_real_distribution<float> sizeDistr(MIN_SIZE, 0.05);

void Entity::simulate(float dt)
{
    float szCoeff = (size - MIN_SIZE) / (MAX_SIZE - MIN_SIZE);
    pos += vel * dt * 0.5f / (1 + szCoeff);
}

Entity Entity::create()
{
    Entity result{
        .pos = randomPos(),
        .size = sizeDistr(engine),
        .color = colorDistr(engine) | (128 << 24),
        .id = firstFreeId++,
    };

    if (firstFreeId == kInvalidId) {
        ++firstFreeId;
    }

    return result;
}

glm::vec2 Entity::randomPos()
{
    return {
        coordDistr(engine),
        coordDistr(engine),
    };
}
