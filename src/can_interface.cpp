#include "can_interface.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can/raw.h>

can_interface::can_interface() : socket_fd_(-1), is_initialized_(false) {}

can_interface::~can_interface() {
    if (is_initialized_) {
        close(socket_fd_);
        std::cout << "Socket closed" << std::endl;
    }
}

bool can_interface::init(const std::string& ifname) {
    interface_name_ = ifname;
    
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return false;
    }

    int enable_canfd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 
                   &enable_canfd, sizeof(enable_canfd)) < 0) {
        std::cerr << "Error enabling CAN FD support" << std::endl;
        close(socket_fd_);
        return false;
    }

    // Enable loopback (receive own frames)
    // TODO: Remove when vcan testing is done, as this is not needed for real CAN interfaces
    int recv_own_msgs = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, 
                   &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
        std::cerr << "Error enabling receive own messages" << std::endl;
        close(socket_fd_);
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_name_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting interface index" << std::endl;
        close(socket_fd_);
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding socket" << std::endl;
        close(socket_fd_);
        return false;
    }

    is_initialized_ = true;
    std::cout << "CAN FD interface '" << interface_name_ 
              << "' initialized successfully" << std::endl;
    return true;
}

bool can_interface::send(uint32_t can_id, const uint8_t* data, uint8_t len, bool use_brs) {
    if (!is_initialized_) {
        std::cerr << "Interface not initialized" << std::endl;
        return false;
    }

    if (len > 64) {
        std::cerr << "Data length too large (max 64 bytes)" << std::endl;
        return false;
    }

    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = can_id;
    frame.len = len;
    frame.flags = use_brs ? CANFD_BRS : 0;
    
    if (data != nullptr && len > 0) {
        memcpy(frame.data, data, len);
    }

    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        std::cerr << "Error sending CAN frame" << std::endl;
        return false;
    }

    /*std::cout << "Sent CAN FD frame - ID: 0x" << std::hex << can_id 
              << ", Length: " << std::dec << (int)len << " bytes" << std::endl;*/
    return true;
}

bool can_interface::receive(struct canfd_frame& frame) {
    if (!is_initialized_) {
        std::cerr << "Interface not initialized" << std::endl;
        return false;
    }

    ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
    
    if (nbytes == CANFD_MTU) {
        /*std::cout << "Received CAN FD frame - ID: 0x" << std::hex << frame.can_id 
                  << ", Length: " << std::dec << (int)frame.len << " bytes" << std::endl;*/
        return true;
    } else {
        std::cerr << "Error: Invalid CAN frame received" << std::endl;
        return false;
    }
}

bool can_interface::receive(struct canfd_frame& frame, int timeout_ms) {
    if (!is_initialized_) {
        std::cerr << "Interface not initialized" << std::endl;
        return false;
    }

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        std::cerr << "Error setting socket timeout" << std::endl;
        return false;
    }

    ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
    
    if (nbytes == CANFD_MTU) {
        /*std::cout << "Received frame - ID: 0x" << std::hex << frame.can_id 
                  << ", Length: " << std::dec << (int)frame.len << " bytes" << std::endl;*/
        return true;
    } else if (nbytes < 0) {
        // Timeout or error
        return false;
    } else {
        std::cerr << "Error: Invalid CAN frame received" << std::endl;
        return false;
    }
}

bool can_interface::start_async_receive(std::function<void(const struct canfd_frame&, bool)> callback) {
    if (!is_initialized_) {
        std::cerr << "interface not initialized" << std::endl;
        return false;
    }

    receiving_ = true;

    receive_thread_ = std::thread([this, callback]() {
        while (receiving_) {
            struct canfd_frame frame;
            bool success = receive(frame, 1000);
            callback(frame, success);
        }
        std::cout << "Async receive thread stopped" << std::endl;
    });
    return true;
}

void can_interface::stop_async_receive() {
    receiving_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
        std::cout << "Receive thread joined successfully!" << std::endl;
    }
}

bool can_interface::set_filter(uint32_t can_id, uint32_t can_mask) {
    if (!is_initialized_) {
        std::cerr << "Interface not initialized" << std::endl;
        return false;
    }

    struct can_filter filter;
    filter.can_id = can_id;
    filter.can_mask = can_mask;

    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        std::cerr << "Error setting CAN filter" << std::endl;
        return false;
    }

    std::cout << "CAN filter set - ID: 0x" << std::hex << can_id 
              << ", Mask: 0x" << can_mask << std::dec << std::endl;
    return true;
}

bool can_interface::set_filters(const struct can_filter* filters, size_t num_filters) {
    if (!is_initialized_) {
        std::cerr << "Interface not initialized" << std::endl;
        return false;
    }

    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, num_filters * sizeof(struct can_filter)) < 0) {
        std::cerr << "Error setting CAN filters" << std::endl;
        return false;
    }

    std::cout << "Set " << num_filters << " filter(s)" << std::endl;
    return true;
}

bool can_interface::clear_filters() {
    if (!is_initialized_) {
        std::cerr << "Interface not initialized" << std::endl;
        return false;
    }

    struct can_filter filter;
    filter.can_id = 0x0;
    filter.can_mask = 0x0;

    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        std::cerr << "Error clearing CAN filters" << std::endl;
        return false;
    }

    std::cout << "Cleared all CAN filters" << std::endl;
    return true;
}

bool can_interface::is_initialized() const {
    return is_initialized_;
}

std::string can_interface::get_interface_name() const {
    return interface_name_;
}