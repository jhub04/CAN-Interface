#include <iostream>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class CANInterface {
private:
    int socket_fd;
    bool is_initialized;
    std::string interface_name;

public:
    CANInterface() : socket_fd(-1), is_initialized(false) {}
    
    ~CANInterface() {
        if (is_initialized) {
            close(socket_fd);
            std::cout << "Socket closed" << std::endl;
        }
    }

    bool init(const std::string& ifname = "can0") {
        interface_name = ifname;
        
        socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd < 0) {
            std::cerr << "Error creating socket" << std::endl;
            return false;
        }

        int enable_canfd = 1;
        if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 
                       &enable_canfd, sizeof(enable_canfd)) < 0) {
            std::cerr << "Error enabling CAN FD support" << std::endl;
            close(socket_fd);
            return false;
        }

        // Enable loopback (receive own frames)
        // TODO: Remove when vcan testing is done, as this is not needed for real CAN interfaces
        int recv_own_msgs = 1;
        if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, 
                       &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
            std::cerr << "Error enabling receive own messages" << std::endl;
            close(socket_fd);
            return false;
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, interface_name.c_str());
        if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
            std::cerr << "Error getting interface index" << std::endl;
            close(socket_fd);
            return false;
        }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            std::cerr << "Error binding socket" << std::endl;
            close(socket_fd);
            return false;
        }

        is_initialized = true;
        std::cout << "CAN FD interface '" << interface_name 
                  << "' initialized successfully" << std::endl;
        return true;
    }

    bool send(uint32_t can_id, const uint8_t* data, uint8_t len, bool use_brs = true) {
        if (!is_initialized) {
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

        if (write(socket_fd, &frame, sizeof(frame)) != sizeof(frame)) {
            std::cerr << "Error sending CAN frame" << std::endl;
            return false;
        }

        std::cout << "Sent CAN FD frame - ID: 0x" << std::hex << can_id 
                  << ", Length: " << std::dec << (int)len << " bytes" << std::endl;
        return true;
    }

    // Blocking receive
    bool receive(struct canfd_frame& frame) {
        if (!is_initialized) {
            std::cerr << "Interface not initialized" << std::endl;
            return false;
        }

        ssize_t nbytes = read(socket_fd, &frame, sizeof(frame));
        
        if (nbytes == CANFD_MTU) {
            std::cout << "Received CAN FD frame - ID: 0x" << std::hex << frame.can_id 
                      << ", Length: " << std::dec << (int)frame.len << " bytes" << std::endl;
            return true;
        } else {
            std::cerr << "Error: Invalid CAN frame received" << std::endl;
            return false;
        }
    }

    // Receive with timeout
    bool receive(struct canfd_frame& frame, int timeout_ms) {
        if (!is_initialized) {
            std::cerr << "Interface not initialized" << std::endl;
            return false;
        }

        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        
        if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
            std::cerr << "Error setting socket timeout" << std::endl;
            return false;
        }

        ssize_t nbytes = read(socket_fd, &frame, sizeof(frame));
        
        if (nbytes == CANFD_MTU) {
            std::cout << "Received frame - ID: 0x" << std::hex << frame.can_id 
                      << ", Length: " << std::dec << (int)frame.len << " bytes" << std::endl;
            return true;
        } else if (nbytes < 0) {
            // Timeout or error
            return false;
        } else {
            std::cerr << "Error: Invalid CAN frame received" << std::endl;
            return false;
        }
    }

    bool isInitialized() const {
        return is_initialized;
    }

    std::string getInterfaceName() const {
        return interface_name;
    }
};

int main() {
    CANInterface can;

    if (!can.init("can0")) {
        return -1;
    }

    uint8_t data[64]; 
    for (int i = 0; i < 64; ++i) {
        data[i] = i;
    }

    //can.send(0x123, data, 64);

    struct canfd_frame rx_frame;
    if (can.receive(rx_frame, 5000)) {
        // Process received frame (already printed in receive function)
        std::cout << "Data received: ";
        for (int i = 0; i < rx_frame.len; ++i) {
            printf("%02X ", rx_frame.data[i]);
        }
        std::cout << std::endl;
    }

    return 0;
}