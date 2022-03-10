#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "encoder_node");

    ros::NodeHandle nh;

    int s, i; 
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

    strcpy(ifr.ifr_name, "vcan0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return 1;
	}

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("encoder_node/count", 1);

    while(ros::ok()){

        nbytes = read(s, &frame, sizeof(struct can_frame));

        if(nbytes > 0){

            uint8_t length = frame.data[0];

            if(length != frame.can_dlc){
                continue;
            }

            uint8_t deviceId = frame.data[1];
            uint8_t cmdId = frame.data[2];

            uint32_t reading;
            for(int i=3; i<length -3;++i){
                reading |= frame.data[i] << 8 * (i - 3);
            }
            

            std_msgs::Int32 msg;

            msg.data = reading;

            ROS_INFO("%d published", count);

            pub.publish(msg);
        }

        
    }

    return 0;
}
