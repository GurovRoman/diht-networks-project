#pragma once
#include <enet/enet.h>

template<class T>
using UniquePtr = std::unique_ptr<T, void (*)(T*)>;

#define UNUSED(x) (void)(x);

template<class F>
struct Defer {
    F f;
    ~Defer() { f(); }
};

template<class F>
Defer(F) -> Defer<F>;

std::string formatIpAddress(uint32_t ip) {
    char temp[32];
    ENetAddress addr {.host = ip};
    enet_address_get_host_ip(&addr, temp, 32);
    return temp;
}
