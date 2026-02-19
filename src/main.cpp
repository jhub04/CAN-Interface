#include "../include/can_interface.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

int main() {
    can_interface can;
    can.init("vcan0");
    can.clear_filters();

    std::atomic<int> frames_received(0);

    std::cout << "[main] Starting async receive..." << std::endl;

    can.receive_async([&frames_received](const struct canfd_frame& frame, bool success) {
        if (success) {
            frames_received++;
            std::cout << "[async thread] Received frame ID: 0x" << std::hex << frame.can_id
                      << " | frames so far: " << std::dec << frames_received.load() << std::endl;
        }
    });

    // Send frames from the main thread with delays, proving main thread is NOT blocked
    for (int i = 0; i < 5; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        uint8_t data[] = {(uint8_t)i, 0xAB, 0xCD, 0xEF};
        std::cout << "[main] Sending frame " << i << "..." << std::endl;
        can.send(0x123, data, 4);
    }

    can.stop_async_receive();

    std::cout << "[main] Done. Total frames received: " << frames_received.load() << std::endl;

    return 0;
}