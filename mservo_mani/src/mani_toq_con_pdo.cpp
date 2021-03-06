#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "mservo_mani/des_toq.h"
#include "mservo_mani/motor_data.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <iostream>

#include <net/if.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'

using namespace std;

// Manipulator Joint Parameters
unsigned short g_usNodeId_arr[] = {1, 2, 3, 4, 5, 6, 7};
//int g_NbofJoint = sizeof(g_usNodeId_arr)/ sizeof(*g_usNodeId_arr);
int g_NbofJoint = 7;
int g_GearRatio[] = {160, 160, 160, 120, 100, 100, 100};
double g_AxisVelRatio[] = {1.6, 1.6, 1.6, 1.2, 1, 1, 1};
int g_PulseRev[] = {4096, 4096, 4096, 4096, 2048, 2048, 2048};
int g_HomeOffset_arr[] = {72818, 72818, 72818, 54613, 22378, 22378, 22378};
long g_SafeRange[] = {2148124, 1121393, 1856853, 1004885, 575715, 423253, 637156};
int g_PosLimitMin[] = {-5000000, -5000000, -5000000, -5000000, -5000000, -5000000, -5000000};
int g_PosLimitMax[] = {5000000, 5000000, 5000000, 5000000, 5000000, 5000000, 5000000};
double rad2inc[7];
double rads2rpm[7];
int s;
string Control_Endless = "3F00";
string Control_Enable = "0F00";

unsigned short COB_ID[][4] = {{0x223, 0x323, 0x423, 0x523},
                              {0x221, 0x321, 0x421, 0x521},
                              {0x222, 0x322, 0x422, 0x522},
                              {0x223, 0x323, 0x423, 0x523},
                              {0x224, 0x324, 0x424, 0x524},
                              {0x225, 0x325, 0x425, 0x525},
                              {0x226, 0x326, 0x426, 0x526},
                              {0x227, 0x327, 0x427, 0x527}};


unsigned char asc2nibble(char c) {

    if ((c >= '0') && (c <= '9'))
        return c - '0';

    if ((c >= 'A') && (c <= 'F'))
        return c - 'A' + 10;

    if ((c >= 'a') && (c <= 'f'))
        return c - 'a' + 10;

    return 16; /* error */
}

int hexstring2data(char *arg, unsigned char *data, int maxdlen) {

    int len = strlen(arg);
    int i;
    unsigned char tmp;

    if (!len || len%2 || len > maxdlen*2)
        return 1;

    memset(data, 0, maxdlen);

    for (i=0; i < len/2; i++) {

        tmp = asc2nibble(*(arg+(2*i)));
        if (tmp > 0x0F)
            return 1;

        data[i] = (tmp << 4);

        tmp = asc2nibble(*(arg+(2*i)+1));
        if (tmp > 0x0F)
            return 1;

        data[i] |= tmp;
    }

    return 0;
}

int parse_canframe(char *cs, struct canfd_frame *cf) {
    /* documentation see lib.h */

    int i, idx, dlen, len;
    int maxdlen = CAN_MAX_DLEN;
    int ret = CAN_MTU;
    unsigned char tmp;

    len = strlen(cs);
    //printf("'%s' len %d\n", cs, len);

    memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

    if (len < 4)
        return 0;

    if (cs[3] == CANID_DELIM) { /* 3 digits */

        idx = 4;
        for (i=0; i<3; i++){
            if ((tmp = asc2nibble(cs[i])) > 0x0F)
                return 0;
            cf->can_id |= (tmp << (2-i)*4);
        }

    } else if (cs[8] == CANID_DELIM) { /* 8 digits */

        idx = 9;
        for (i=0; i<8; i++){
            if ((tmp = asc2nibble(cs[i])) > 0x0F)
                return 0;
            cf->can_id |= (tmp << (7-i)*4);
        }
        if (!(cf->can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
            cf->can_id |= CAN_EFF_FLAG;   /* then it is an extended frame */

    } else
        return 0;

    if((cs[idx] == 'R') || (cs[idx] == 'r')){ /* RTR frame */
        cf->can_id |= CAN_RTR_FLAG;

        /* check for optional DLC value for CAN 2.0B frames */
        if(cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC)
            cf->len = tmp;

        return ret;
    }

    if (cs[idx] == CANID_DELIM) { /* CAN FD frame escape char '##' */

        maxdlen = CANFD_MAX_DLEN;
        ret = CANFD_MTU;

        /* CAN FD frame <canid>##<flags><data>* */
        if ((tmp = asc2nibble(cs[idx+1])) > 0x0F)
            return 0;

        cf->flags = tmp;
        idx += 2;
    }

    for (i=0, dlen=0; i < maxdlen; i++){

        if(cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
            idx++;

        if(idx >= len) /* end of string => end of data */
            break;

        if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
            return 0;
        cf->data[i] = (tmp << 4);
        if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
            return 0;
        cf->data[i] |= tmp;
        dlen++;
    }
    cf->len = dlen;

    return ret;
}

void LogInfo(string message)
{
    cout << message << endl;
}

template <typename T>
std::string dec_to_hex(T dec, int NbofByte){
    std::stringstream stream_HL;
    string s, s_LH;
    stream_HL << std::setfill ('0') << std::setw(sizeof(T)*2) <<std::hex << dec;
    s = stream_HL.str();
    for (int i=0; i<NbofByte; i++){
        s_LH.append(s.substr(2*(NbofByte-1-i),2));
    }
    return s_LH;
}

int hexarray_to_int(unsigned char *buffer, int length){
    int hextoint = 0;
    for (int i=0; i<length; i++)
    {
        hextoint += (buffer[i] << 8*i);
    }
    return hextoint;
}

void initialize(double *p_rad2inc, double *p_rads2rpm)
{
    for(int i=0; i<g_NbofJoint; i++){
        p_rad2inc[i] = g_PulseRev[i]*2*g_GearRatio[i]/M_PI;
        p_rads2rpm[i] = 30*g_GearRatio[i]/M_PI;
    }
}

std::string stringappend(string a, string b)
{
    string s;
    s.append(a);
    s.append(b);
    return s;
}

void commandCallback(const mservo_mani::des_toq::ConstPtr& msg)
{
    int des_toq;
    struct can_frame frame;
    string data;
    des_toq = msg->toq;

    frame.can_id  = COB_ID[0][3];
    frame.can_dlc = 2;
    data = dec_to_hex(des_toq, 4);
    hexstring2data((char *) data.c_str(), frame.data, 4);
    write(s, &frame, sizeof(struct can_frame));
    ros::Duration(0.0001).sleep();
    ROS_INFO("Torque: %d", des_toq);
}


int main (int argc, char **argv){
    ros::init(argc, argv, "Controller_toq_PDO");
    ros::NodeHandle n;

    initialize(rad2inc, rads2rpm);

    int pos, vel;
    short toq;

    struct sockaddr_can addr;
    struct can_frame frame;
    struct canfd_frame frame_get, frame_fd;
    struct iovec iov;
    struct msghdr canmsg;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3* sizeof(struct timespec) + sizeof(__u32))];
    struct ifreq ifr;

    const char *ifname = "slcan0";

    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

//    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    // Initiallize NMT Services
    LogInfo("Initialize NMT Services");
    // Reset Communication
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x82;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Reset Communication");
    ros::Duration(2).sleep();

    // Start Remote Node
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x01;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Start Remote Node");
    ros::Duration(1).sleep();

    frame.can_id  = COB_ID[0][1];
    frame.can_dlc = 3;
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x0A;
    write(s, &frame, sizeof(struct can_frame));
    ros::Duration(0.5).sleep();

    // Shutdown Controlword
    frame.can_id  = COB_ID[0][0];
    frame.can_dlc = 2;
    frame.data[0] = 0x06;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    ros::Duration(0.5).sleep();

    // Enable Controlword
    frame.can_id  = COB_ID[0][0];
    frame.can_dlc = 2;
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    ros::Duration(0.5).sleep();

    LogInfo("Set CST mode");

    iov.iov_base = &frame_get;
    canmsg.msg_name = &addr;
    canmsg.msg_iov = &iov;
    canmsg.msg_iovlen = 1;
    canmsg.msg_control = &ctrlmsg;

    iov.iov_len = sizeof(frame_get);
    canmsg.msg_namelen = sizeof(addr);
    canmsg.msg_controllen = sizeof(ctrlmsg);
    canmsg.msg_flags = 0;

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    for (int i=0; i<3; i++)
    {
        recvmsg(s, &canmsg, 0);
    }

    ros::Publisher pub = n.advertise<mservo_mani::motor_data>("motor_state",1);
    ros::Subscriber sub = n.subscribe("des_toq", 1, commandCallback);
//    ros::spin();
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        stringstream ss;
        mservo_mani::motor_data motor_state;

        ros::Time begin = ros::Time::now();
        frame.can_id  = 0x080;
        frame.can_dlc = 2;
        frame.data[0] = 0x00;
        frame.data[1] = 0x00;
        write(s, &frame, sizeof(struct can_frame));
        ros::Duration(0.00001).sleep();

        recvmsg(s, &canmsg, 0);
        motor_state.pos = hexarray_to_int(frame_get.data,4);
        ROS_INFO("pos: %d", motor_state.pos);
        recvmsg(s, &canmsg, 0);
        motor_state.vel = hexarray_to_int(frame_get.data,4);
        ROS_INFO("vel: %d", motor_state.vel);
        recvmsg(s, &canmsg, 0);
        motor_state.toq = hexarray_to_int(frame_get.data,2);
        ROS_INFO("toq: %d", motor_state.toq);

//        string rtr = "283#R";
//        int required_mtu = parse_canframe((char*)(rtr.c_str()), &frame_fd);
//        write(s, &frame_fd, required_mtu);
//        ros::Duration(0.0001).sleep();
//        recvmsg(s, &canmsg, 0);
        ros::Time finish = ros::Time::now();
        ss << (finish - begin) << " " ;
        LogInfo(ss.str());

        pub.publish(motor_state);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Reset Communication
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x81;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Reset Communication");
    ros::Duration(2).sleep();

    // Stop Remote Node
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x02;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Stop Remote Node");

    return 0;
}