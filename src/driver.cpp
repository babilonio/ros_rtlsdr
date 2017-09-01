#include "ros/ros.h"
#include <iostream>
#include <chrono>
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "ros_rtlsdr/IQSample.h"
#include "ros_rtlsdr/IQSampleArray.h"
#include "ros_rtlsdr/ParamSet.h"

#include "RtlSdr.hpp"

//*** GLOBAL VAR *****//
std::chrono::time_point<std::chrono::system_clock> start, end;
RtlSdr                                             rtl;
int                                                N = 0x7FFF + 1;

int                                                max_files   = 1;
int                                                active_file = 0;
//*****************//

//*** ROS SERVICES **//
bool set_center_freq(ros_rtlsdr::ParamSet::Request  &req,
                     ros_rtlsdr::ParamSet::Response &res)
{
    ROS_INFO("center_freq_set request: fc=%ud", (unsigned int) req.param);

    if (req.param > 30e6 && req.param < 1.7e9)
        res.error = rtl.setCenterFreq(req.param);
    else
        res.error = 1;

    ROS_INFO("sending back response error: [%d]", res.error);
    return true;
}

bool set_sample_rate(ros_rtlsdr::ParamSet::Request  &req,
                     ros_rtlsdr::ParamSet::Response &res)
{
    ROS_INFO("sample_rate_set request: fs=%ud", (unsigned int) req.param);

    if (req.param > 0.95e6 && req.param < 2.5e6)
        res.error = rtl.setSampleRate(req.param);
    else
        res.error = 1;

    ROS_INFO("sending back response error: [%d]", res.error);
    return true;
}

bool set_num_samples_to_read(ros_rtlsdr::ParamSet::Request  &req,
                             ros_rtlsdr::ParamSet::Response &res)
{
    ROS_INFO("num_samples_to_read_set request: N=%d", (int) req.param);

    if ((int) req.param > 0x7FFF)
    {
        N         = (int) req.param;
        res.error = 0;
    }
    else
        res.error = 1;

    ROS_INFO("sending back response error: [%d]", res.error);
    return true;
}

//*****************//

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver");
    ros::NodeHandle    n;

    ros::Publisher     pub_path        = n.advertise<std_msgs::String>("path_to_samples", max_files);
    ros::Publisher     pub_center_freq = n.advertise<std_msgs::UInt32>("center_freq", max_files);
    ros::Publisher     pub_sample_rate = n.advertise<std_msgs::UInt32>("sample_rate", max_files);

    ros::ServiceServer service_cf = n.advertiseService("set_center_freq", set_center_freq);
    ros::ServiceServer service_sr = n.advertiseService("set_sample_rate", set_sample_rate);
    ros::ServiceServer service_ns = n.advertiseService("set_num_samples_to_read", set_num_samples_to_read);


    while (ros::ok())
    {
        if (!rtl.ok())
        {
            ROS_INFO("Opening RTL-SDR device");
            if (!rtl.open())
            {
                ROS_ERROR("Failed to open RTLSDR device ");
                ros::Duration(0.5).sleep();
                continue;
            }
            else
            {
                rtl.setCenterFreq(102.4e6);
                rtl.setSampleRate(2.4e6);

                ROS_INFO("RTL-SDR center_freq : %ud", rtl.getCenterFreq());
                ROS_INFO("RTL-SDR sample_rate : %ud", rtl.getSampleRate());
                ROS_INFO("RTL-SDR num_samples_to_read : %d", N);
            }
        }
        else
        {
            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            if (rtl.toFile(N, "/tmp/samples" + std::to_string(active_file) + ".bin"))
            {
                std_msgs::String str;
                str.data = "/tmp/samples" + std::to_string(active_file) + ".bin";
                pub_path.publish(str);

                if (++active_file >= max_files)
                    active_file = 0;

                end = std::chrono::system_clock::now();

                std::chrono::duration<double> elapsed_seconds = end - start;
                ROS_INFO("RTL-SDR samples reading time: %f", elapsed_seconds.count());
            }
            else
            {
                ROS_ERROR("Error reading samples, closing device");
                rtl.close();
            }
        }

        ros::spinOnce();
    }

    return 0;
}
