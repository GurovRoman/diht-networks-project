#pragma once

#include "../common/proto.hpp"
#include "Entity.hpp"

PROTO_IMPL_PACKET(PossessEntity)
{
    uint32_t id;
};

PROTO_IMPL_PACKET(StateDelta)
{
    uint64_t epoch;
    uint64_t total_bytes;
    using Continuation = uint8_t;
};

PROTO_IMPL_PACKET(StateDeltaConfirmation)
{
    uint64_t epoch;
};