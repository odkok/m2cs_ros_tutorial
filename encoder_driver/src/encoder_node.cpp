#include "ros/ros.h"
#include "std_msgs/UInt32.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <poll.h>

const int TIMEOUT_MILLISECOND = 200;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_node");

    ROS_INFO("Init node");

    ros::NodeHandle nh;

    int s, i;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Socket");
        return 1;
    }

    strcpy(ifr.ifr_name, "vcan0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        return 1;
    }

    ROS_INFO("Socket Setup Successfully");

    ros::Publisher pub = nh.advertise<std_msgs::UInt32>("encoder_node/count", 1);

    while (ros::ok())
    {
        struct pollfd pfds[1];

        pfds[0].fd = s;
        pfds[0].events = POLLIN;

        int event = poll(pfds, 1, TIMEOUT_MILLISECOND);

        if (event == -1)
        {
            perror("Poll error");
        }
        else
        {
            nbytes = read(s, &frame, sizeof(struct can_frame));

            if (nbytes > 0)
            {
                uint8_t length = frame.data[0];
                uint8_t deviceId = frame.data[1];
                uint8_t cmdId = frame.data[2];

                if (length != frame.can_dlc || length <= 3)
                {
                    ROS_INFO("Invalid Packet, length=%d, can_dlc=%u", length, frame.can_dlc);
                    continue;
                }

                uint32_t reading = 0;
                for (int i = 3; i < length; ++i)
                {
                    reading |= frame.data[i] << 8 * (i - 3);
                }

                std_msgs::UInt32 msg;

                msg.data = reading;

                ROS_INFO("%x published", reading);

                pub.publish(msg);

                ros::spinOnce();
            }
        }
    }

    if (close(s) < 0)
    {
        perror("Close");
        return 1;
    }

    ros::shutdown();

    return 0;
}
