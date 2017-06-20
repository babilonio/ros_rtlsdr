// g++ -o main main.cpp rtlsdr.cpp -Wall -lrtlsdr
#include "ros/ros.h"

#include <iostream>
#include <chrono>
#include "RtlSdr.hpp"

#include "ros_rtlsdr/IQSample.h"
#include "ros_rtlsdr/IQSampleArray.h"

RtlSdr rtl;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_rtlsdr_node");
    ros::NodeHandle n;

    ros::Publisher  pub = n.advertise<ros_rtlsdr::IQSampleArray>("iqdata", 100);

    if (!rtl.open())
    {
        std::cerr << "Failed to open RTLSDR device " << '\n';
        return 0;
    }
    else
    {
        rtl.setCenterFreq(99.8e6);
        std::cout << "center_freq " << rtl.getCenterFreq() << '\n';
        rtl.setSampleRate(1102500);
    }

    IQVector                 samples(0x7FFF + 1);
    ros_rtlsdr::IQSampleArray compArray;
    while (ros::ok())
    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        if (rtl.readSync(samples))
        {
            for (uint32_t i = 0; i < samples.size(); i++)
            {
                ros_rtlsdr::IQSample c;
                c.real = samples[i].real();
                c.imag = samples[i].imag();
                compArray.data.push_back(c);
            }
            std::cout << "publishing compArray " << compArray.data.size() << '\n';
            pub.publish(compArray);
            compArray.data.clear();
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
