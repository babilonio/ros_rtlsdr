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
std::chrono::time_point<std::chrono::system_clock> last_publish;

RtlSdr                                             rtl;
int                                                N = 0x7FFF + 1;

int                                                max_files   = 1;
int                                                active_file = 0;

ros::Publisher                                     pub_path;
ros::Publisher                                     pub_center_freq;
ros::Publisher                                     pub_sample_rate;
//*****************//

//*** ROS SERVICES **//
bool set_center_freq(ros_rtlsdr::ParamSet::Request  &req,
                     ros_rtlsdr::ParamSet::Response &res)
{
    ROS_INFO("center_freq_set request: fc=%u", (unsigned int) req.param);

    if (req.param > 30e6 && req.param < 1.7e9)
        res.error = rtl.setCenterFreq(req.param);
    else
        res.error = 1;

    std_msgs::UInt32 val;
    val.data = rtl.getCenterFreq();
    pub_sample_rate.publish(val);

    ROS_INFO("sending back response error: [%d]", res.error);
    return true;
}

bool set_sample_rate(ros_rtlsdr::ParamSet::Request  &req,
                     ros_rtlsdr::ParamSet::Response &res)
{
    ROS_INFO("sample_rate_set request: fs=%u", (unsigned int) req.param);


    if (req.param > 0.95e6 && req.param < 2.5e6)
        res.error = rtl.setSampleRate(req.param);
    else
        res.error = 1;

    std_msgs::UInt32 val;
    val.data = rtl.getSampleRate();
    pub_sample_rate.publish(val);

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
    ros::NodeHandle n;

    pub_path        = n.advertise<std_msgs::String>("path_to_samples", max_files);
    pub_center_freq = n.advertise<std_msgs::UInt32>("center_freq", max_files);
    pub_sample_rate = n.advertise<std_msgs::UInt32>("sample_rate", max_files);

    ros::ServiceServer service_cf = n.advertiseService("set_center_freq", set_center_freq);
    ros::ServiceServer service_sr = n.advertiseService("set_sample_rate", set_sample_rate);
    ros::ServiceServer service_ns = n.advertiseService("set_num_samples_to_read", set_num_samples_to_read);


    while (ros::ok())
    {
        if (rtl.getDeviceCount() > 0)
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

                    std_msgs::UInt32 val;
                    val.data = rtl.getSampleRate();
                    pub_sample_rate.publish(val);
                    val.data = rtl.getCenterFreq();
                    pub_center_freq.publish(val);

                    last_publish = std::chrono::system_clock::now();

                    ROS_INFO("RTL-SDR center_freq : %u", rtl.getCenterFreq());
                    ROS_INFO("RTL-SDR sample_rate : %u", rtl.getSampleRate());
                    ROS_INFO("RTL-SDR num_samples_to_read : %d", N);
                }
            }
            else
            {

                // start = std::chrono::system_clock::now();

                if (rtl.toFile(N, "/tmp/samples" + std::to_string(active_file) + ".bin"))
                {
                    std_msgs::String str;
                    str.data = "/tmp/samples" + std::to_string(active_file) + ".bin";
                    pub_path.publish(str);

                    if (++active_file >= max_files)
                        active_file = 0;

                    // end = std::chrono::system_clock::now();

                    // std::chrono::duration<double> elapsed_seconds = end - start;
                    // ROS_INFO("RTL-SDR samples reading time: %f", elapsed_seconds.count());
                }
                else
                {
                    ROS_ERROR("Error reading samples, exiting program");

                    return EXIT_FAILURE;
                    //rtl.close();
                }
            }
        }
        else
        {
            ROS_WARN("No RTL-SDR device found");
            ros::Duration(1).sleep();
        }

        std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - last_publish;
        if (elapsed_seconds.count() > 5.0)
        {
            std_msgs::UInt32 val;
            val.data = rtl.getSampleRate();
            pub_sample_rate.publish(val);
            ROS_INFO("RTL-SDR publishing sample_rate : %u", val.data);

            val.data = rtl.getCenterFreq();
            pub_center_freq.publish(val);
            ROS_INFO("RTL-SDR publishing center_freq : %u", val.data);

            last_publish = std::chrono::system_clock::now();
        }

        ros::spinOnce();
    }

    return 0;
}
