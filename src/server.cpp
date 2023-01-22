#include <argparse/argparse.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <random>
#include <unordered_map>

#include "common/Service.hpp"
#include "common/assert.hpp"
#include "common/delta.hpp"
#include "common/proto.hpp"

#include "game/Entity.hpp"
#include "game/gameProto.hpp"

using namespace std::chrono_literals;

class ServerService : public Service<ServerService, true> {
    using Clock = std::chrono::steady_clock;

public:
    ServerService(ENetAddress addr) : Service(&addr, 32, 2) { resetGame(); }

    void resetGame()
    {
        constexpr size_t kBots = 10;

        entities_.clear();
        botTargets_.clear();

        entities_.reserve(kBots);
        botTargets_.reserve(kBots);
        for (size_t i = 0; i < kBots; ++i) {
            auto id = entities_.emplace_back(Entity::create()).id;
            botTargets_.emplace(id, Entity::randomPos());
        }
    }

    void registerInLobby(const char* addr, uint16_t port)
    {
        // kostyl
        static ENetAddress address;
        if (addr != nullptr) {
            enet_address_set_host(&address, addr);
            address.port = port;
        }

        connect(address, [this](ENetPeer* lobby) {
            NG_VERIFY(lobby != nullptr);
            send(lobby, 0, ENET_PACKET_FLAG_RELIABLE, PRegisterServerInLobby{});
            disconnect(lobby, []() {});
        });
    }

    void handlePacket(ENetPeer* peer, PChat packet)
    {
        packet.player = clients_.at(peer).id;
        for (auto& [client, data]: clients_) {
            if (client == peer)
                continue;

            send(client, 1, {}, packet);
        }
    }

    void connected(ENetPeer* peer)
    {
        spdlog::info("{}:{} joined", formatIpAddress(peer->address.host), peer->address.port);

        static auto genKey = []() {
            using value_type = int;
            static std::default_random_engine generator;
            static std::uniform_int_distribution<value_type> distribution;

            std::array<uint8_t, 16> key;
            std::generate(key.begin(), key.end(), []() { return distribution(generator); });

            return key;
        };

        auto key = genKey();

        send(
            peer,
            0,
            ENET_PACKET_FLAG_RELIABLE,
            PSetKey{
                .key = key,
            });

        setKeyForPeer(peer, {key.begin(), key.end()});

        auto id = idCounter_++;

        auto& created = entities_.emplace_back(Entity::create());

        clients_.emplace(
            peer,
            ClientData{
                .id = id,
                .entityId = created.id,
            });

        send(
            peer,
            0,
            ENET_PACKET_FLAG_RELIABLE,
            PPossessEntity{
                .id = created.id,
            });

        for (auto& [client, data]: clients_) {
            if (client == peer)
                continue;

            send(client, 0, ENET_PACKET_FLAG_RELIABLE, PPlayerJoined{.id = id});

            send(peer, 0, ENET_PACKET_FLAG_RELIABLE, PPlayerJoined{.id = data.id});
        }

        send_deltas();
    }

    Entity* entityById(id_t id)
    {
        auto it = std::find_if(
            entities_.begin(), entities_.end(), [id](const Entity& e) {
                return e.id == id;
            });
        if (it == entities_.end())
            return nullptr;

        return &*it;
    }

    // input delta-compression
    void handlePacket(
        ENetPeer* peer, const PStateDelta& packet, std::span<std::uint8_t> cont)
    {
        auto it = clients_.find(peer);
        if (it == clients_.end())
            return;

        auto* entity = entityById(it->second.entityId);

        if (entity == nullptr)
            return;

        NG_ASSERT(sizeof(entity->vel) == packet.total_bytes);
        delta_apply(
            {reinterpret_cast<uint8_t*>(&entity->vel), sizeof(entity->vel)},
            cont);
        send(peer, 1, {}, PStateDeltaConfirmation{.epoch = packet.epoch});

        float len = glm::length(entity->vel);

        if (len < 1e-3) {
            entity->vel = {0, 0};
            return;
        }

        entity->vel = entity->vel / len * std::clamp(len, 0.f, 1.f);
    }

    void disconnected(ENetPeer* peer)
    {
        spdlog::info("{}:{} left", formatIpAddress(peer->address.host), peer->address.port);

        auto it = clients_.find(peer);
        if (it == clients_.end()) {
            return;
        }

        auto erasedData = std::move(it->second);
        clients_.erase(it);

        for (auto& [client, data]: clients_) {
            send(
                client,
                0,
                ENET_PACKET_FLAG_RELIABLE,
                PPlayerLeft{.id = erasedData.id});
        }

        if (clients_.empty()) {
            spdlog::info("All players left, requeueing in lobby");
            resetGame();
            registerInLobby(nullptr, 0);
        }
    }

    void updateLogic(float delta)
    {
        for (auto& entity: entities_) {
            if (botTargets_.contains(entity.id)) {
                auto v = botTargets_[entity.id] - entity.pos;
                auto len = glm::length(v);

                if (len < 1e-3) {
                    botTargets_[entity.id] = Entity::randomPos();
                    continue;
                }

                entity.vel = v / len * 0.2f;
            }

            entity.simulate(delta);

            entity.pos = glm::clamp(entity.pos, glm::vec2{-0.5}, glm::vec2{1.5});
        }

        for (auto& e1: entities_) {
            for (auto& e2: entities_) {
                if (&e1 == &e2)
                    continue;

                if (e1.size < e2.size && glm::length(e1.pos - e2.pos) <
                                             (e1.size + e2.size)) {
                    e2.size += e1.size / 2;
                    e1.size /= 2;

                    e1.pos = Entity::randomPos();
                    ++e1.teleport_count;
                }
            }
        }

        for (size_t i = 0; i < entities_.size();) {
            if (entities_[i].size < 1e-3) {
                auto id = entities_[i].id;
                std::swap(entities_[i], entities_.back());
                entities_.pop_back();
            } else {
                ++i;
            }
        }
    }

    void handlePacket(ENetPeer* peer, const PStateDeltaConfirmation& packet)
    {
        auto& client = clients_.at(peer);
        client.delta_queue.ReceiveConfirmation(packet.epoch);
    }

    void send_deltas()
    {
        const std::span state{
            reinterpret_cast<const uint8_t*>(entities_.data()),
            entities_.size() * sizeof(decltype(entities_)::value_type)};

        for (auto& [to, client]: clients_) {
            const auto [epoch, delta] = client.delta_queue.GetStateDelta(state);
            send(
                to,
                1,
                {},
                PStateDelta{.epoch = epoch, .total_bytes = state.size()},
                delta);
        }
    }

    void run()
    {
        constexpr auto kSendRate = 100ms;

        auto startTime = Clock::now();
        auto currentTime = startTime;
        auto lastSendTime = startTime;

        while (true) {
            auto now = Clock::now();
            float delta =
                std::chrono::duration_cast<std::chrono::duration<float>>(
                    now - std::exchange(currentTime, now))
                    .count();

            if (clients_.size() > 0) {
                updateLogic(delta);
            }

            if ((now - lastSendTime) > kSendRate) {
                lastSendTime = now;

                send_deltas();
            }

            Service::poll(1);
        }
    }

private:
    struct ClientData {
        uint32_t id;
        id_t entityId;
        DeltaSendQueue delta_queue;
    };

    std::unordered_map<ENetPeer*, ClientData> clients_;
    uint32_t idCounter_{1};
    std::vector<Entity> entities_;

    std::unordered_map<id_t, glm::vec2> botTargets_;
};

int main(int argc, char** argv)
{
    argparse::ArgumentParser args(std::filesystem::path(argv[0]).filename().string());

    args.add_description("Main game server");

    args.add_argument("listen_port")
        .help("port to listen at")
        .scan<'u', uint16_t>()
        .default_value(uint16_t(27016));

    args.add_argument("lobby_address")
        .help("lobby server address")
        .default_value(std::string("127.0.0.1"));

    args.add_argument("lobby_port")
        .help("lobby server port")
        .scan<'u', uint16_t>()
        .default_value(uint16_t(27015));

    try {
        args.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << args;
        std::exit(1);
    }

    NG_VERIFY(enet_initialize() == 0);
    std::atexit(enet_deinitialize);

    ENetAddress address{
        .host = ENET_HOST_ANY,
        .port = args.get<uint16_t>("listen_port"),
    };

    ServerService server(address);

    server.registerInLobby(args.get("lobby_address").c_str(), args.get<uint16_t>("lobby_port"));

    server.run();

    return 0;
}
