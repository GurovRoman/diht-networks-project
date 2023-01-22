#pragma once
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

std::vector<uint8_t> delta_encode(
    std::span<const uint8_t> bytes_old, std::span<const uint8_t> bytes_new)
{
    std::vector<uint8_t> result(
        bytes_new.size() / 8 + (bytes_new.size() % 8 != 0));
    for (size_t i = 0; i < bytes_new.size(); ++i) {
        if (i >= bytes_old.size() || bytes_new[i] != bytes_old[i]) {
            result[i / 8] |= (1 << i % 8);
            result.push_back(bytes_new[i]);
        }
    }
    return result;
}

void delta_apply(std::span<uint8_t> bytes, std::span<const uint8_t> delta)
{
    size_t size = bytes.size();

    size_t delta_offset = size / 8 + (size % 8 != 0);

    for (int i = 0; i < size; ++i) {
        if ((delta[i / 8] & (1 << i % 8)) > 0) {
            bytes[i] = delta[delta_offset++];
        }
    }
}

template<class T>
void delta_apply(std::vector<T>& data, std::span<const uint8_t> delta, size_t total_size)
{
    NG_ASSERT(total_size % sizeof(T) == 0);
    data.resize(total_size / sizeof(T));
    delta_apply({reinterpret_cast<uint8_t*>(data.data()), total_size}, delta);
}

struct DataState {
    uint64_t epoch;
    std::vector<uint8_t> data;
};

class DeltaSendQueue {
public:
    DeltaSendQueue() { states_.push_back({0, {}}); }

    void ReceiveConfirmation(uint64_t confirmed_epoch)
    {
        NG_ASSERT(confirmed_epoch <= states_.front().epoch);
        while (!states_.empty() && states_.back().epoch < confirmed_epoch) {
            states_.pop_back();
        }
    }

    std::pair<uint64_t, std::vector<uint8_t>> GetStateDelta(
        std::span<const uint8_t> new_state_bytes)
    {
        const auto& new_state = states_.emplace_front(DataState{
            .epoch = states_.front().epoch + 1,
            .data = std::vector(new_state_bytes.begin(), new_state_bytes.end()),
        });

        const auto& last_confirmed_state = states_.back().data;

        return {
            new_state.epoch,
            delta_encode(last_confirmed_state, new_state.data)};
    }

private:
    std::deque<DataState> states_;
};