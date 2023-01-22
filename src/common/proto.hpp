#pragma once

#include <array>
#include <cstdint>

#include <enet/enet.h>

enum class PacketType : uint8_t {
    StartLobby,
    LobbyStarted,
    RegisterClientInLobby,
    RegisterServerInLobby,
    PlayerJoined,
    PlayerLeft,
    Chat,
    SetKey,
    StateDelta,
    StateDeltaConfirmation,
    PossessEntity,
    PlayerInput,
    COUNT,
};

template<PacketType t>
struct PacketBase {
    PacketType type{t};
};

template<PacketType type>
struct Packet;

#define PROTO_IMPL_PACKET(Name)               \
    using P##Name = Packet<PacketType::Name>; \
    template<>                                \
    struct Packet<PacketType::Name> : PacketBase<PacketType::Name>

PROTO_IMPL_PACKET(StartLobby){};

PROTO_IMPL_PACKET(LobbyStarted)
{
    ENetAddress serverAddress;
};

PROTO_IMPL_PACKET(RegisterClientInLobby){};
PROTO_IMPL_PACKET(RegisterServerInLobby){};

PROTO_IMPL_PACKET(PlayerJoined)
{
    uint32_t id;
};

PROTO_IMPL_PACKET(PlayerLeft)
{
    uint32_t id;
};

PROTO_IMPL_PACKET(Chat)
{
    uint32_t player;
    std::array<char, 1000> message;
};

PROTO_IMPL_PACKET(SetKey)
{
    std::array<uint8_t, 16> key;
};