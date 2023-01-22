cmake_minimum_required(VERSION 3.21)


CPMAddPackage(
        NAME enet
        GITHUB_REPOSITORY lsalzman/enet
        GIT_TAG v1.3.17
)


if (enet_ADDED)
    target_include_directories(enet PUBLIC "${enet_SOURCE_DIR}/include")
    target_link_libraries(enet wsock32 ws2_32 winmm)
endif ()

CPMAddPackage(
        NAME spdlog
        GITHUB_REPOSITORY gabime/spdlog
        VERSION 1.9.2
)

CPMAddPackage(
        NAME function2
        GITHUB_REPOSITORY Naios/function2
        GIT_TAG 4.2.0
)

CPMAddPackage(
        NAME glm
        GITHUB_REPOSITORY g-truc/glm
        GIT_TAG master # no releases for 2 years????
)


CPMAddPackage(
        NAME allegro
        GITHUB_REPOSITORY liballeg/allegro5
        GIT_TAG 5.2.7.0
        OPTIONS
        # Ah, yes, I want my build process to take 10 minutes by default, thank you!
        "WANT_DOCS off"
        "WANT_EXAMPLES off"
        "WANT_DEMO off"
        "WANT_TESTS off"
        "WANT_VIDEO off"
        "WANT_PHYSFS off"
        "WANT_IMAGE_WEBP off"
        "WANT_AUDIO off"
)

if (allegro_ADDED)
    # allegro's authors have no idea how to write proper cmake files
    target_include_directories(allegro INTERFACE
            "${allegro_SOURCE_DIR}/include"
            "${allegro_BINARY_DIR}/include"
            )

    foreach(ADDON font image color primitives)
        target_include_directories("allegro_${ADDON}" INTERFACE
                "${allegro_SOURCE_DIR}/addons/${ADDON}"
                )
    endforeach()

    function(copy_allegro_dlls tgt)
        if (WIN32)
            string(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_TOLOWER)
            foreach(ADDON "" "_font" "_primitives")
                set(LIBNAME "allegro${ADDON}")
                if(${CMAKE_BUILD_TYPE_TOLOWER} STREQUAL "debug")
                    set(LIBNAME "${LIBNAME}-debug")
                endif()
                add_custom_command(TARGET ${tgt}
                        POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${CMAKE_BINARY_DIR}/_deps/allegro-build/lib/${LIBNAME}-5.2.dll"
                        $<TARGET_FILE_DIR:${tgt}>
                        )
            endforeach()
        endif()
    endfunction()
endif()


CPMAddPAckage(
        NAME ImGui
        GITHUB_REPOSITORY ocornut/imgui
        GIT_TAG v1.87
        DOWNLOAD_ONLY YES
)

if (ImGui_ADDED)
    add_library(DearImGui
            ${ImGui_SOURCE_DIR}/imgui.cpp ${ImGui_SOURCE_DIR}/imgui_draw.cpp
            ${ImGui_SOURCE_DIR}/imgui_tables.cpp ${ImGui_SOURCE_DIR}/imgui_widgets.cpp
            ${ImGui_SOURCE_DIR}/backends/imgui_impl_allegro5.cpp)

    target_include_directories(DearImGui PUBLIC ${ImGui_SOURCE_DIR})

    target_link_libraries(DearImGui allegro allegro_primitives)
endif ()

CPMAddPackage(
        NAME argparse
        GITHUB_REPOSITORY p-ranav/argparse
        VERSION 2.9
)