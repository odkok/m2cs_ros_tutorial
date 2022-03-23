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

#include <sstream>

const int TIMEOUT_MILLISECOND = 200;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_node");


    ros::NodeHandle nh("~");

    int s, i;
    int nbytes;
    sockaddr_can addr;
    can_frame frame;
    ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        ROS_ERROR("Socket Error");
        return 1;
    }

    std::string ifrName;
    if(! nh.getParam("ifr_name", ifrName))
    {
        ROS_ERROR("Cannot retrieve param ifr_name");
        return 1;
    }

    strcpy(ifr.ifr_name, ifrName.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        ROS_ERROR("Error when binding socket");
        return 1;
    }

    std::string filterID;
    if(nh.getParam("filter_id", filterID))
    {
        if(filterID != "")
        {
            std::stringstream ss;
            ss << std::hex << filterID;
            uint32_t canId;
            ss >> canId;
            if(canId > 0xfff)
            {
                ROS_WARN("Invalid can filter id: %x", canId);
            }
            else
            {
                can_filter filter[1];
                filter[0].can_id = canId;
                filter[0].can_mask = CAN_SFF_MASK;

                setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
                ROS_INFO("Set filter id=%x", canId);
            }
        }
    }

    ros::Publisher pub = nh.advertise<std_msgs::UInt32>("count", 1);

    ROS_INFO("Init node");
    
    while (ros::ok())
    {
        pollfd pfds[1];

        pfds[0].fd = s;
        pfds[0].events = POLLIN;

        int event = poll(pfds, 1, TIMEOUT_MILLISECOND);

        if (event == -1)
        {
            ROS_ERROR("Poll error");
        }
        else
        {
            nbytes = read(s, &frame, sizeof(can_frame));

            if (nbytes > 0)
            {
                uint8_t length = frame.data[0];
                uint8_t deviceId = frame.data[1];
                uint8_t cmdId = frame.data[2];

                if (length != frame.can_dlc || length <= 3)
                {
                    ROS_WARN("Invalid Packet, length=%d, can_dlc=%u", length, frame.can_dlc);
                    continue;
                }

                uint32_t reading = 0;
                for (int i = 3; i < length; ++i)
                {
                    reading |= frame.data[i] << 8 * (i - 3);
                }

                std_msgs::UInt32 msg;
                msg.data = reading;
                pub.publish(msg);
                ros::spinOnce();
            }
        }
    }

    if (close(s) < 0)
    {
        ROS_ERROR("Error when closing socket");
        return 1;
    }

    ros::shutdown();

    return 0;
}
