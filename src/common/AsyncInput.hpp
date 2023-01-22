#pragma once

#include <atomic>
#include <deque>
#include <iostream>
#include <thread>

#include "assert.hpp"

// NOTE: shutdown support is SHIT
template<class Derived>
class AsyncInput {
public:
    AsyncInput() : worker_{[this]() { Work(); }} { }

    void stop()
    {
        stopped_.store(true, std::memory_order::relaxed);
        worker_.join();
    };

    void poll()
    {
        std::deque<std::string> stolen;
        {
            std::lock_guard lock{mtx_};
            stolen = std::move(lines_);
        }
        for (const auto& line: stolen) {
            self().handleLine(line);
        }
    }

    ~AsyncInput() { NG_VERIFY(stopped_.load(std::memory_order::relaxed)); }

private:
    Derived& self() { return *static_cast<Derived*>(this); }
    const Derived& self() const { return *static_cast<const Derived*>(this); }

    void Work()
    {
        while (!stopped_.load(std::memory_order::relaxed)) {
            std::string result;
            std::getline(std::cin, result);

            std::lock_guard lock{mtx_};
            lines_.emplace_back(std::move(result));
        }
    }

private:
    std::mutex mtx_;
    std::deque<std::string> lines_;

    std::thread worker_;
    std::atomic<bool> stopped_{false};
};
