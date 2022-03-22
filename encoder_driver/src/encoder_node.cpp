#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "encoder_node");
    ros::NodeHandle nh("~");
    std::string node_name = nh.resolveName("");

    ros::Publisher encoder_pub = nh.advertise<std_msgs::Int32>("p_feedback", 1);

    // param from launch file
    int can_id;
    if (nh.getParam("can_id", can_id)) {
        ROS_INFO("%s: Can id: %i", node_name.c_str(), can_id);
    } else {
        ROS_ERROR("%s: Cannot retrieve can id", node_name.c_str());
        return 1;
    }

    int device_id;
    if (nh.getParam("device_id", device_id)) {
        ROS_INFO("%s: Device id: %i", node_name.c_str(), device_id);
    } else {
        ROS_ERROR("%s: Cannot retrieve device id", node_name.c_str());
        return 1;
    }

    // set up socket
    int s;
    int nbytes;
    sockaddr_can addr;
    ifreq ifr;
    can_frame frame;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        ROS_ERROR("Socket");
        return 1;
    }

    strcpy(ifr.ifr_name, "vcan0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("Bind");
        return 1;
    }

    can_filter rfilter[1];
    rfilter[0].can_id   = can_id;
    rfilter[0].can_mask = CAN_EFF_MASK; // enable all 29 filter bits

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    while (ros::ok()) {
        // read the CAN frame
        nbytes = read(s, &frame, sizeof(can_frame));
     	if (nbytes < 0) {
            ROS_ERROR_THROTTLE(1, "Read");
            continue;
    	}
        if (frame.data[1] != device_id) {
            // device id not match
            // TODO: how to handle this
            continue;
        }

        // print data
        // char data_str[32];
        // int idx = 0;
        // ROS_INFO("0x%03X [%d] ", frame.can_id, frame.can_dlc);
        // for (int i = 0; i < frame.can_dlc; i++) {
        //     idx += snprintf(data_str+idx, sizeof data_str, "%02X ", frame.data[i]);
        // }
        // ROS_INFO("%s\n", data_str);

        // fill in msg based on the contents from CAN frame
        uint8_t len = frame.data[0];
        if (len != frame.can_dlc) {
            ROS_WARN_THROTTLE(1, "frame size received not match with data len");
            continue;
        }
        if (len - 3 < 4) {
            ROS_WARN_THROTTLE(1, "byte too short, not enough data to parse int32");
            continue;
        }
        std_msgs::Int32 msg;
        memcpy(&msg.data, frame.data + 3, 4);
        encoder_pub.publish(msg);
    }
    // close socket
    if (close(s) < 0) {
        ROS_ERROR("Close");
        return 1;
    }
    return 0;
}
