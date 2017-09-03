#include "ros/ros.h"
#include <iostream>
#include <chrono>
#include "std_msgs/String.h"
#include "rosrtlsdr/IQSample.h"
#include "rosrtlsdr/IQSampleArray.h"
#include "RtlSdr.hpp"

std::chrono::time_point<std::chrono::system_clock> start, end;
ros::Publisher                                     pub;


RtlSdr rtl;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosrtlsdr_node");
    ros::NodeHandle n;

    ros::Publisher  pub = n.advertise<std_msgs::String>("path_to_samples", 10);

    if (!rtl.open())
    {
        std::cerr << "Failed to open RTLSDR device " << '\n';
        return 0;
    }
    else
    {
        rtl.setCenterFreq(102.4e6 - 250e3);
        std::cout << "center_freq " << rtl.getCenterFreq() << '\n';
        rtl.setSampleRate(1102500);
    }

    while (ros::ok())
    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        if (rtl.toFile(0x7FFFF + 1, "/tmp/samples.bin"))
        {
            std_msgs::String str;
            str.data = "/tmp/samples.bin";
            pub.publish(str);
        }
        else
            break;
        end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << "reading time: " << elapsed_seconds.count() << "s\n";

        //ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    return 0;
}
