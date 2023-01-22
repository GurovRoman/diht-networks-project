#include <argparse/argparse.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <unordered_set>

#include "common/AsyncInput.hpp"
#include "common/Service.hpp"
#include "common/assert.hpp"
#include "common/common.hpp"
#include "common/proto.hpp"

using namespace std::chrono_literals;

class LobbyService : public Service<LobbyService, true> {
    using Clock = std::chrono::steady_clock;

public:
    LobbyService(ENetAddress addr) : Service(&addr, 32, 2) { }

    void handlePacket(ENetPeer*, const PStartLobby&)
    {
        if (servers_.empty()) {
            spdlog::error("No servers to send clients to!");
            return;
        }

        auto server = servers_.front();
        servers_.pop_front();

        spdlog::info(
            "Sending {} clients to server {}:{}!",
            clients_.size(),
            formatIpAddress(server.host),
            server.port);

        for (auto peer: clients_) {
            send(
                peer,
                0,
                ENET_PACKET_FLAG_RELIABLE,
                PLobbyStarted{
                    .serverAddress = server,
                });
        }
    }

    void handlePacket(ENetPeer* client, const PRegisterClientInLobby&)
    {
        spdlog::info(
            "Client {}:{} registered",
            formatIpAddress(client->address.host),
            client->address.port);
        clients_.emplace(client);
    }

    void handlePacket(ENetPeer* server, const PRegisterServerInLobby&)
    {
        spdlog::info(
            "Server {}:{} registered",
            formatIpAddress(server->address.host),
            server->address.port);
        servers_.push_back(server->address);
    }

    void disconnected(ENetPeer* peer) { clients_.erase(peer); }

    void run()
    {
        auto lastPollTime = Clock::now();
        while (true) {
            auto now = Clock::now();
            auto delta = now - std::exchange(lastPollTime, now);

            UNUSED(delta);

            Service::poll();
        }
    }

private:
    std::unordered_set<ENetPeer*> clients_;
    std::deque<ENetAddress> servers_;
};

int main(int argc, char** argv)
{
    argparse::ArgumentParser args(std::filesystem::path(argv[0]).filename().string());

    args.add_description("Lobby server which routes clients to game servers");

    args.add_argument("lobby_listen_port")
        .help("port to listen at")
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
        .port = args.get<uint16_t>("lobby_listen_port"),
    };

    LobbyService lobby(address);

    lobby.run();

    return 0;
}
