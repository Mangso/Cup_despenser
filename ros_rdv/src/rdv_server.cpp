#include <ros/ros.h>
#include "ros_rdv/rdv.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <string>
#include <bitset>


serial::Serial ser;
std::vector<uint8_t> data = {0xFF, 0xFF, 0xFF, 0x01, 0x05, 0xF3, 0x86, 0, 0, 0};


void dec2hex(int num, std::vector<uint8_t>& data)
{
    char hexdec_num[2]={0};
    char hexdec_num2[2]={0};

    char hex[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    int i = 0;
    int sum = 0;
    int dec_num = num;
    int r, n;
    //   while(dec_num > 0){

    //     if(i<2){
    //         r = dec_num % 16;
    //         hexdec_num[i] = hex[r];
    //         dec_num = dec_num/16;
    //     }
    //     else if (i==2 || i ==3){
    //         r = dec_num % 16;
    //         hexdec_num2[i-2] = hex[r];
    //         dec_num = dec_num/16;
    //     }
    //     i++;
    //  }

    int factor_1,factor_2;
    factor_1 = dec_num >> 8;
    factor_2 = dec_num & 0x00ff;

    // data[7] = std::stoi(hexdec_num, nullptr,16);
    // data[8] = std::stoi(hexdec_num2,nullptr,16);

    data[8] = factor_1;
    data[7] = factor_2;

    for(int i=3;i<=8;i++){
        sum += data[i];
    }
    std::bitset<8> bb;

    bb= std::bitset<8>(sum);
    bb.flip();

    int checksum = bb.to_ulong();

    data[9] = checksum;

}

bool onnoff(ros_rdv::rdv::Request &req, ros_rdv::rdv::Response &res)
{

    int num = (long int)req.a;
    dec2hex(num, data);

    for(int i=0;i < 10; i++){
        printf("%x ", data[i]);
    }

    ROS_INFO("request message : %ld", (long int)req.a);

    std::vector<uint8_t> data2 = {0xFF, 0xFF, 0xFF, 0x01, 0x05, 0xF3, 0x86, 0xFF, 0x07, 0x7A};
    // ser.write(data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rdv_serial_server");

    ros::NodeHandle n;

    // try
    // {
    //     ser.setPort("/dev/ttyUSB0");
    //     ser.setBaudrate(57600);
    //     serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    //     ser.setTimeout(to);
    //     ser.open();
    // }
    // catch (serial::IOException& e)
    // {
    //     ROS_ERROR_STREAM("Unable to open port ");
    //     return -1;
    // }

    // if(ser.isOpen()){
    //     ROS_INFO_STREAM("Serial Port initialized");
    // }else{
    //     return -1;
    // }
    ros_rdv::rdv::Request req;
    ros::ServiceServer service = n.advertiseService("rdv_serial", onnoff);
    
    ros::spin();
   
    return 0;
    
}