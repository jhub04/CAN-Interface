#include "../include/can_interface.h"
#include <iostream>

int main() {
    CanInterface can;
    can.init("can0");

    // Accept IDs 0x123 and 0x456
    struct can_filter filters[2];
    filters[0].can_id = 0x123;
    filters[0].can_mask = 0x7FF;
    filters[1].can_id = 0x456;
    filters[1].can_mask = 0x7FF;

    can.set_filters(filters, 2);

    can.clear_filters();

    uint8_t data[] = {1, 2, 3, 4};
    can.send(0x123, data, 4);  // Will receive
    can.send(0x456, data, 4);  // Will receive
    can.send(0x789, data, 4);  // Will NOT receive

    struct canfd_frame frame;
    while (can.receive(frame, 5000)) {
        std::cout << "Got frame with ID: 0x" << std::hex << frame.can_id << std::endl;
    }

    return 0;
}