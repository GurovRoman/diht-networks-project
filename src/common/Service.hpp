#pragma once

#include <enet/enet.h>
#include <function2/function2.hpp>
#include <span>
#include <type_traits>

#include "assert.hpp"
#include "common.hpp"
#include "proto.hpp"

// Fuck windows :)
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

template<class Derived, bool IS_SERVER = false>
class Service {
public:
    Service(const ENetAddress* address, size_t peerCount, size_t channelLimit)
        : host_{
              enet_host_create(address, peerCount, channelLimit, 0, 0),
              &enet_host_destroy}
    {
        NG_VERIFY(host_ != nullptr);
    }

    template<class F>
    void connect(ENetAddress address, F f)
    {
        ENetPeer* peer = enet_host_connect(host_.get(), &address, 2, 0);
        if (peer == nullptr) {
            f(peer);
            return;
        }
        pending_connect_.emplace(peer, std::forward<F>(f));
    }

    template<class F>
    void disconnect(ENetPeer* peer, F f)
    {
        enet_peer_disconnect_later(peer, 0);
        pending_disconnect_.emplace(peer, std::forward<F>(f));
    }

    template<PacketType t>
    void send(
        ENetPeer* peer,
        enet_uint8 channel,
        ENetPacketFlag flag,
        const Packet<t>& packet)
    {
        static_assert(
            !requires { typename Packet<t>::Continuation; },
            "Missing continuation argument in send!");
        peer_send_ciphered(
            peer, channel, enet_packet_create(&packet, sizeof(packet), flag));
    }

    template<PacketType t>
    requires requires { typename Packet<t>::Continuation; }
    void send(
        ENetPeer* peer,
        enet_uint8 channel,
        ENetPacketFlag flag,
        const Packet<t>& packet,
        std::span<const typename Packet<t>::Continuation> cont)
    {
        using Cont = typename Packet<t>::Continuation;
        size_t contSizeBytes = sizeof(Cont) * cont.size();
        auto enetpacket =
            enet_packet_create(&packet, sizeof(packet) + contSizeBytes, flag);
        std::memcpy(enetpacket->data + sizeof(packet), cont.data(), contSizeBytes);
        peer_send_ciphered(peer, channel, enetpacket);
    }

    template<PacketType t>
    void handlePacket(ENetPeer* peer, const Packet<t>&)
    {
        spdlog::error(
            "Unsupported packet {} received from {}:{}",
            t,
            formatIpAddress(peer->address.host),
            peer->address.port);
    }

    void connected(ENetPeer* peer)
    {
        spdlog::info(
            "Connection established with {}:{}",
            formatIpAddress(peer->address.host),
            peer->address.port);
    }

    void disconnected(ENetPeer* peer)
    {
        spdlog::info(
            "Disconnected from {}:{}", formatIpAddress(peer->address.host), peer->address.port);
    }

    void poll(uint32_t timeoutMs = 30)
    {
        ENetEvent event;
        while (enet_host_service(host_.get(), &event, timeoutMs) > 0) {
            switch (event.type) {
                case ENET_EVENT_TYPE_CONNECT:
                    if (auto it = pending_connect_.find(event.peer);
                        it != pending_connect_.end()) {
                        std::move(it->second)(event.peer);
                        pending_connect_.erase(it);
                    } // only servers can get abrupt connects
                    else if constexpr (IS_SERVER) {
                        self().connected(event.peer);
                    }
                    break;

                case ENET_EVENT_TYPE_DISCONNECT:
                    keys_.erase(event.peer);

                    if (auto it = pending_disconnect_.find(event.peer);
                        it != pending_disconnect_.end()) {
                        std::move(it->second)();
                        pending_disconnect_.erase(it);
                    } else // Clients can get abrupt disconnects
                    {
                        self().disconnected(event.peer);
                    }
                    break;

                case ENET_EVENT_TYPE_RECEIVE: {
                    cipherXor(event.peer, event.packet);

                    ENetPeer* peer = event.peer;
                    uint8_t* data = event.packet->data;

                    auto type =
                        *reinterpret_cast<PacketType*>(event.packet->data);

                    NG_VERIFY(
                        static_cast<int>(type) <
                        static_cast<int>(PacketType::COUNT));

                    auto procPacketType = [type,
                                           peer,
                                           data,
                                           size = event.packet->dataLength,
                                           this]<PacketType t>() {
                        if (type == t) {
                            const Packet<t>& packet =
                                *reinterpret_cast<const Packet<t>*>(data);

                            if constexpr (requires {
                                              typename Packet<t>::Continuation;
                                          }) {
                                using PacketCont =
                                    typename Packet<t>::Continuation;
                                std::span<PacketCont> cont{
                                    reinterpret_cast<PacketCont*>(
                                        data + sizeof(Packet<t>)),
                                    (size - sizeof(Packet<t>)) /
                                        sizeof(PacketCont)};
                                if constexpr (requires {
                                                  self().handlePacket(
                                                      peer, packet, cont);
                                              }) {
                                    self().handlePacket(peer, packet, cont);
                                } else {
                                    handlePacket(peer, packet);
                                }
                            } else {
                                // I hoped that it would find the default handlePacket on it's own,
                                // but two-phase lookup is hard :(
                                if constexpr (requires {
                                                  self().handlePacket(
                                                      peer, packet);
                                              }) {
                                    self().handlePacket(peer, packet);
                                } else {
                                    handlePacket(peer, packet);
                                }
                            }
                        }
                    };

                    [&procPacketType]<std::size_t... Is>(
                        std::index_sequence<Is...>)
                    {
                        (...,
                         procPacketType.template
                         operator()<static_cast<PacketType>(Is)>());
                    }
                    (std::make_index_sequence<static_cast<std::size_t>(
                         PacketType::COUNT)>{});

                    enet_packet_destroy(event.packet);
                } break;
                default:
                    break;
            };
        }
    }

    void setKeyForPeer(ENetPeer* peer, std::vector<uint8_t> key)
    {
        keys_[peer] = std::move(key);
    }

private:
    Derived& self() { return *static_cast<Derived*>(this); }
    const Derived& self() const { return *static_cast<const Derived*>(this); }

    void cipherXor(ENetPeer* peer, ENetPacket* packet) const
    {
        if (!keys_.contains(peer))
            return;

        auto& key = keys_.at(peer);

        for (size_t i = 0; i < packet->dataLength; ++i) {
            packet->data[i] ^= key[i % key.size()];
        }
    }

    void peer_send_ciphered(
        ENetPeer* peer, enet_uint8 channelID, ENetPacket* packet) const
    {
        cipherXor(peer, packet);
        enet_peer_send(peer, channelID, packet);
    }

private:
    UniquePtr<ENetHost> host_;
    std::unordered_map<ENetPeer*, fu2::function<void(ENetPeer*)>> pending_connect_;
    std::unordered_map<ENetPeer*, fu2::function<void()>> pending_disconnect_;
    std::unordered_map<ENetPeer*, std::vector<uint8_t>> keys_;
};
