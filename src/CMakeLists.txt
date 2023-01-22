cmake_minimum_required(VERSION 3.20)


add_library(common_lib common/common.cpp)
target_link_libraries(common_lib enet spdlog function2 argparse)

add_library(game_lib game/Entity.cpp)
target_link_libraries(game_lib PUBLIC spdlog glm::glm)


add_executable(client client.cpp)
target_link_libraries(client
        common_lib game_lib allegro allegro_font allegro_primitives)

copy_allegro_dlls(client)

add_executable(server server.cpp)
target_link_libraries(server common_lib game_lib)

add_executable(lobby lobby.cpp)
target_link_libraries(lobby common_lib)
