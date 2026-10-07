// Fixed pool of sample blocks passed from the USB event thread to a DSP
// thread. The producer never blocks: when the pool is exhausted the block is
// dropped and counted, so a slow consumer cannot stall USB streaming.
#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

namespace rx888 {

class BlockQueue {
public:
    struct Block {
        std::vector<uint8_t> data;
        size_t len = 0;
    };

    void init(size_t blockSize, size_t count) {
        std::lock_guard<std::mutex> lck(mtx);
        blocks.assign(count, Block());
        for (auto& b : blocks) { b.data.resize(blockSize); }
        head = tail = used = 0;
        dropped = 0;
        stopped = false;
    }

    // Producer: copy data into the next free block. Returns false if dropped.
    bool push(const uint8_t* data, size_t len) {
        {
            std::lock_guard<std::mutex> lck(mtx);
            if (stopped) { return false; }
            if (used == blocks.size() || len > blocks[head].data.size()) {
                dropped++;
                return false;
            }
            Block& b = blocks[head];
            memcpy(b.data.data(), data, len);
            b.len = len;
            head = (head + 1) % blocks.size();
            used++;
        }
        cv.notify_one();
        return true;
    }

    // Consumer: wait for a block (nullptr when stopped or on timeout).
    Block* front(int timeoutMs = 200) {
        std::unique_lock<std::mutex> lck(mtx);
        cv.wait_for(lck, std::chrono::milliseconds(timeoutMs), [this] { return used > 0 || stopped; });
        if (stopped || used == 0) { return nullptr; }
        return &blocks[tail];
    }

    // Consumer: release the block returned by front().
    void pop() {
        std::lock_guard<std::mutex> lck(mtx);
        if (used == 0) { return; }
        tail = (tail + 1) % blocks.size();
        used--;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lck(mtx);
            stopped = true;
        }
        cv.notify_all();
    }

    size_t fill() {
        std::lock_guard<std::mutex> lck(mtx);
        return used;
    }
    size_t capacity() const { return blocks.size(); }
    uint64_t droppedCount() const { return dropped; }

private:
    std::mutex mtx;
    std::condition_variable cv;
    std::vector<Block> blocks;
    size_t head = 0, tail = 0, used = 0;
    std::atomic<uint64_t> dropped{ 0 };
    bool stopped = false;
};

} // namespace rx888
