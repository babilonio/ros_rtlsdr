#include "ros/ros.h"

#include <iostream>
#include <chrono>

#include "ros_rtlsdr/Complex.h"
#include "ros_rtlsdr/ComplexArray.h"

std::chrono::time_point<std::chrono::system_clock> start, end;


void iqCallback(const ros_rtlsdr::ComplexArray::ConstPtr& array)
{
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "reading time: " << elapsed_seconds.count() << "s\n";
    std::cout << array->data.size() << " samples received" << "\n";
    start = std::chrono::system_clock::now();

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fm_receiver");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("iqdata", 100, iqCallback);

    ros::spin();

    return 0;
}
