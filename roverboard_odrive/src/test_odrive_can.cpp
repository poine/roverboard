#include <iostream>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

int main(int argc, const char * argv[]) {

    int s; /* can raw socket */
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      perror("socket");
      return 1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (!ifr.ifr_ifindex) {
      perror("if_nametoindex");
      return 1;
    }
 
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      perror("bind");
      return 1;
    }

    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame)); /* init CAN FD frame, e.g. LEN = 0 */
    frame.can_id = 0x37;// 0X17
    frame.can_id |= CAN_RTR_FLAG;
    


    int required_mtu = CAN_MTU;
    //int required_mtu = parse_canframe("017#R", &frame);//CAN_MTU;
    if (write(s, &frame, required_mtu) != required_mtu) {
      perror("write");
      return 1;
    }

    close(s);
    return 0;
}
