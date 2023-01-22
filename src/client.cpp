#include <argparse/argparse.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <optional>
#include <unordered_set>

#include "common/Allegro.hpp"
#include "common/AsyncInput.hpp"
#include "common/Service.hpp"
#include "common/assert.hpp"
#include "common/common.hpp"
#include "common/delta.hpp"
#include "common/proto.hpp"

#include "game/Entity.hpp"
#include "game/gameProto.hpp"

using namespace std::chrono_literals;

ALLEGRO_COLOR colorToAllegro(uint32_t color)
{
    return al_map_rgba(
        (color >> 0) & 0xff,
        (color >> 8) & 0xff,
        (color >> 16) & 0xff,
        (color >> 24) & 0xff);
}

class ClientService :
    public Service<ClientService>,
    public AsyncInput<ClientService>,
    public Allegro<ClientService> {
    using Clock = std::chrono::steady_clock;

    struct StateSnapshot {
        std::vector<Entity> entities;
        Clock::time_point time;
    };

public:
    ClientService() : Service(nullptr, 2, 2) { }

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

    void handlePacket(ENetPeer*, const PChat& packet)
    {
        std::string line{packet.message.data()};
        std::cout << packet.player << ": " << line << std::endl;
    }

    void handlePacket(ENetPeer*, const PPlayerJoined& packet)
    {
        otherIds_.emplace(packet.id);
    }

    void handlePacket(ENetPeer*, const PPlayerLeft& packet)
    {
        otherIds_.erase(packet.id);
    }

    void handlePacket(ENetPeer*, const PPossessEntity& packet)
    {
        playerEntityId_ = packet.id;
    }

    void handlePacket(ENetPeer*, const PLobbyStarted& packet)
    {
        disconnect(std::exchange(lobby_peer_, nullptr), []() {});
        connect(packet.serverAddress, [this](ENetPeer* server) {
            NG_VERIFY(server != nullptr);
            server_peer_ = server;
            snapshotHistory_.emplace_back(StateSnapshot{.time = Clock::now()});

            // kostyl for cipher
            std::this_thread::sleep_for(100ms);
            Service::poll(1);
        });
    }

    void handlePacket(
        ENetPeer* peer, const PStateDelta& packet, std::span<std::uint8_t> cont)
    {
        auto& newSnapshot =
            snapshotHistory_.emplace_back(snapshotHistory_.back());
        newSnapshot.time = Clock::now();

        delta_apply(newSnapshot.entities, cont, packet.total_bytes);
        send(peer, 1, {}, PStateDeltaConfirmation{.epoch = packet.epoch});

        if (snapshotHistory_.size() > 1000) {
            snapshotHistory_.pop_front();
        }
    }

    // input delta-compression
    void handlePacket(ENetPeer* peer, const PStateDeltaConfirmation& packet)
    {
        inputDeltaSendQueue.ReceiveConfirmation(packet.epoch);
    }

    void handlePacket(ENetPeer* peer, const PSetKey& packet)
    {
        setKeyForPeer(peer, {packet.key.begin(), packet.key.end()});
    }

    void begin()
    {
        if (lobby_peer_ != nullptr) {
            send(lobby_peer_, 0, ENET_PACKET_FLAG_RELIABLE, PStartLobby{});
        }
    }

    void handleLine(std::string_view line)
    {
        if (line == "/begin") {
            begin();
        } else if (line == "/list") {
            for (auto id: otherIds_) {
                std::cout << id << " ";
            }
            std::cout << std::endl;
        } else if (line == "/exit" && server_peer_ != nullptr) {
            close();
        } else if (server_peer_ != nullptr) {
            PChat packet{.player = 0, .message = {0}};
            std::strncpy(
                packet.message.data(),
                line.data(),
                std::min(packet.message.size() - 1, line.size()));
            send(server_peer_, 1, {}, packet);
        }
    }

    void joinLobby(const char* addr, uint16_t port)
    {
        ENetAddress address;
        enet_address_set_host(&address, addr);
        address.port = port;

        connect(address, [this](ENetPeer* lobby) {
            NG_VERIFY(lobby != nullptr);
            send(lobby, 0, ENET_PACKET_FLAG_RELIABLE, PRegisterClientInLobby{});
            lobby_peer_ = lobby;
        });
    }

    void close()
    {
        auto cb = [this]() { shouldStop_ = true; };
        if (server_peer_ != nullptr) {
            disconnect(std::exchange(server_peer_, nullptr), cb);
        } else if (lobby_peer_ != nullptr) {
            disconnect(std::exchange(lobby_peer_, nullptr), cb);
        } else {
            cb();
        }
    }

    void keyDown(int keycode)
    {
        if (keycode == ALLEGRO_KEY_ESCAPE) {
            close();
        } else if (keycode == ALLEGRO_KEY_B) {
            begin();
        }
    }

    void keyUp(int) { }

    void mouse(int x, int y)
    {
        playerDesiredSpeed_ = {
            static_cast<float>(x - kWidth / 2),
            static_cast<float>(y - kHeight / 2)};
        const float len = glm::length(playerDesiredSpeed_);

        if (len < 1e-3)
            return;

        playerDesiredSpeed_ /= len;
        playerDesiredSpeed_ *= std::clamp(len - 30, 0.f, 100.f) / 100.f;
    }

    void draw()
    {
        glm::vec2 playerPos{0.5f, 0.5f};
        for (auto& entity: entities_) {
            if (entity.id == playerEntityId_) {
                playerPos = entity.pos;
                break;
            }
        }

        float scale = static_cast<float>(kWidth + kHeight) / 2.f;

        auto worldToScreen = [playerPos, scale](glm::vec2 v) {
            glm::vec2 screen{kWidth, kHeight};
            return screen / 2.f + (v - playerPos) * scale;
        };

        if (server_peer_ != nullptr) {
            int bars = 10;
            for (int i = 0; i <= bars; ++i) {
                float x = static_cast<float>(i) / static_cast<float>(bars);
                auto a = worldToScreen({0, x});
                auto b = worldToScreen({1, x});
                al_draw_line(
                    a.x, a.y, b.x, b.y, al_map_rgba(100, 100, 100, 128), 2);
                auto c = worldToScreen({x, 0});
                auto d = worldToScreen({x, 1});
                al_draw_line(
                    c.x, c.y, d.x, d.y, al_map_rgba(100, 100, 100, 128), 2);
            }
        }

        al_set_blender(ALLEGRO_ADD, ALLEGRO_ALPHA, ALLEGRO_INVERSE_ALPHA);

        /*if (!snapshotHistory_.empty())
            for (auto& entity: snapshotHistory_.back().entities) {
                auto p = worldToScreen(entity.pos);
                al_draw_filled_circle(
                    p.x, p.y, entity.size * scale, al_map_rgba(255,255,255,100));
            }*/

        for (auto& entity: entities_) {
            auto p = worldToScreen(entity.pos);
            al_draw_filled_circle(
                p.x, p.y, entity.size * scale, colorToAllegro(entity.color));
        }

        al_draw_text(
            getFont(),
            al_map_rgb(255, 255, 255),
            0,
            0,
            0,
            "ESC = /exit; B = /begin");
    }

    static float durationToSecs(Clock::duration d)
    {
        return std::chrono::duration_cast<std::chrono::duration<float>>(d).count();
    }

    static Clock::duration secsToDuration(float d)
    {
        return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<float>(d));
    }

    void interpolate(
        Entity& targetEnt, const Entity& prevEnt, float delta, float timeDistance)
    {
        Entity result;
        auto h = delta / timeDistance;

        if (h < 0.5)
            result = prevEnt;
        else
            result = targetEnt;

        if (targetEnt.teleport_count != prevEnt.teleport_count) {
            if (h < 0.5) {
                result.pos = prevEnt.pos + prevEnt.vel * delta;
            } else {
                result.pos = targetEnt.pos - targetEnt.vel * (timeDistance - delta);
            }
        } else {
            result.pos = targetEnt.pos * h + (1 - h) * prevEnt.pos;
        }

        targetEnt = result;
    }

    void applySnapshots(Clock::time_point now, float delta)
    {
        const auto deltaMs = secsToDuration(delta);
        static auto time = now - deltaMs;
        time += deltaMs;

        constexpr auto errorExtraTime = 20ms;
        static auto error = Clock::duration{0};

        static auto lastAccountedSnapshotTime = now;
        if (snapshotHistory_.size() >= 2 && lastAccountedSnapshotTime != snapshotHistory_.back().time) {
            // new snapshot appeared this frame
            const auto deathLine = snapshotHistory_[snapshotHistory_.size() - 2].time;
            const auto goodLine = deathLine - errorExtraTime;

            if (time > deathLine) {
                time = goodLine;
            }

            error = goodLine - time;
            lastAccountedSnapshotTime = snapshotHistory_.back().time;
            //std::cout << duration_cast<std::chrono::milliseconds>(error) << '/' << duration_cast<std::chrono::milliseconds>(now - time) << std::endl;
        }

        // constexpr auto errorCorrectionPerSecond = 30ms;
        const auto errorCorrection = duration_cast<Clock::duration>(error * delta);
        error -= errorCorrection;
        time += errorCorrection;

        if (snapshotHistory_.size() < 2) {
            return;
        }

        while (snapshotHistory_.size() > 2 && snapshotHistory_[1].time < time) {
            snapshotHistory_.pop_front();
        }

        auto targetSnapshot = snapshotHistory_[1];
        const auto& prevSnapshot = snapshotHistory_[0];

        std::unordered_map<id_t, const Entity*> prevEntities;
        for (const auto& ent: prevSnapshot.entities) {
            prevEntities.try_emplace(ent.id, &ent);
        }

        for (auto& entity: targetSnapshot.entities) {
            if (prevEntities.contains(entity.id)) {
                const auto& prevEnt = *prevEntities.at(entity.id);
                interpolate(
                    entity,
                    prevEnt,
                    durationToSecs(time - prevSnapshot.time),
                    durationToSecs(targetSnapshot.time - prevSnapshot.time));
            }
        }

        entities_ = std::move(targetSnapshot.entities);
    }

    void run()
    {
        constexpr auto kSendRate = 60ms;

        auto startTime = Clock::now();
        auto currentTime = startTime;
        auto lastSendTime = startTime;

        while (!shouldStop_) {
            auto now = Clock::now();
            float delta = durationToSecs(now - std::exchange(currentTime, now));

            AsyncInput::poll();
            Allegro::poll();
            Service::poll(1);

            if (server_peer_ != nullptr) {
                applySnapshots(now, delta);
            }

            if (server_peer_ != nullptr && playerEntityId_ != kInvalidId &&
                (now - lastSendTime) > kSendRate) {
                lastSendTime = now;

                const std::span state{
                    reinterpret_cast<const uint8_t*>(&playerDesiredSpeed_),
                    sizeof(playerDesiredSpeed_)};

                const auto [epoch, delta] =
                    inputDeltaSendQueue.GetStateDelta(state);
                send(
                    server_peer_,
                    1,
                    {},
                    PStateDelta{.epoch = epoch, .total_bytes = state.size()},
                    delta);
            }
        }

        Allegro::stop();
        std::cout << "Press ENTER to exit..." << std::endl;
        AsyncInput::stop();
    }

private:
    ENetPeer* lobby_peer_{nullptr};
    ENetPeer* server_peer_{nullptr};

    std::unordered_set<uint32_t> otherIds_;
    bool shouldStop_{false};

    id_t playerEntityId_;
    std::vector<Entity> entities_;
    std::deque<StateSnapshot> snapshotHistory_;

    DeltaSendQueue inputDeltaSendQueue;

    glm::vec2 playerDesiredSpeed_{0, 0};
};

int main(int argc, char** argv)
{
    argparse::ArgumentParser args(std::filesystem::path(argv[0]).filename().string());

    args.add_description("Game client");

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

    ClientService client;

    client.joinLobby(args.get("lobby_address").c_str(), args.get<uint16_t>("lobby_port"));

    client.run();

    return 0;
}
