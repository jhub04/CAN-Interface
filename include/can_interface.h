#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <string>
#include <cstdint>
#include <linux/can.h>

class CanInterface {
private:
    int socket_fd_;
    bool is_initialized_;
    std::string interface_name_;

public:
    CanInterface();
    ~CanInterface();

    bool init(const std::string& ifname = "can0");
    bool send(uint32_t can_id, const uint8_t* data, uint8_t len, bool use_brs = true);
    bool receive(struct canfd_frame& frame);
    bool receive(struct canfd_frame& frame, int timeout_ms);
    bool set_filter(uint32_t can_id, uint32_t can_mask = 0x7FF);
    bool set_filters(const struct can_filter* filters, size_t num_filters);
    bool clear_filters();
    bool is_initialized() const;
    std::string get_interface_name() const;
};

#endif // CAN_INTERFACE_H