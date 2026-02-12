#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct canfd_frame tx_frame;

    // Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    // Enable CAN FD support
    int enable_canfd = 1;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        std::cerr << "Error enabling CAN FD support" << std::endl;
        return 1;
    }

    // Enable loopback (receive own frames)
    int loopback = 1;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
        std::cerr << "Error enabling loopback" << std::endl;
        return 1;
    }

    // Also enable receiving own frames
    int recv_own_msgs = 1;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
        std::cerr << "Error enabling receive own messages" << std::endl;
        return 1;
    }

    // Specify CAN interface (e.g., "can0")
    std::strcpy(ifr.ifr_name, "can0");
    if(ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting interface index" << std::endl;
        return 1;
    }

    // Bind socket to CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding socket" << std::endl;
        return 1;
    }

    std::cout << "CAN FD initialized successfully" << std::endl;

    memset(&tx_frame, 0, sizeof(tx_frame));
    tx_frame.can_id = 0x123; // Example CAN ID
    tx_frame.len = 16;
    tx_frame.flags = CANFD_BRS; // Enable Bit Rate Switching for CAN FD

    for (int i = 0; i < tx_frame.len; i++) {
        tx_frame.data[i] = i; // Fill data with example values
    }

    if (write(s, &tx_frame, sizeof(tx_frame)) != sizeof(tx_frame)) {
        std::cerr << "Error sending CAN frame" << std::endl;
        return 1;
    } else {
        std::cout << "Sent CAN FD frame with " << (int)tx_frame.len << " bytes of data" << std::endl;
        for (int i = 0; i < tx_frame.len; i++) {
            std::cout << "Data[" << i << "] = " << (int)tx_frame.data[i] << std::endl;
        }
    }

    std::cout << "Waiting to receive CAN frame..." << std::endl;

    struct canfd_frame rx_frame;

    ssize_t nbytes = read(s, &rx_frame, CANFD_MTU);


    if (nbytes == CANFD_MTU) {
        printf("got CAN FD frame with length %d\n", rx_frame.len);
        /* rx_frame.flags contains valid data */
    } else if (nbytes == CAN_MTU) {
        printf("got Classical CAN frame with length %d\n", rx_frame.len);
        /* rx_frame.flags is undefined */
    } else {
        fprintf(stderr, "read: invalid CAN(FD) frame\n");
        return 1;
    }




    close(s);
    return 0;
}