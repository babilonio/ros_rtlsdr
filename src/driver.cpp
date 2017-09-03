#include "ros/ros.h"
#include <iostream>
#include <chrono>
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "rosrtlsdr/IQSample.h"
#include "rosrtlsdr/IQSampleArray.h"
#include "rosrtlsdr/ParamSet.h"

#include "textcolor.h"
#include "RtlSdr.hpp"

//*** GLOBAL VAR *****//
double                                             dt;
std::chrono::duration<double>                      elapsed_seconds;
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
bool set_center_freq(rosrtlsdr::ParamSet::Request  &req,
                     rosrtlsdr::ParamSet::Response &res)
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

bool set_sample_rate(rosrtlsdr::ParamSet::Request  &req,
                     rosrtlsdr::ParamSet::Response &res)
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

bool set_tuner_gain(rosrtlsdr::ParamSet::Request  &req,
                    rosrtlsdr::ParamSet::Response &res)
{
    ROS_INFO("set_tuner_gain request: gain=%d", (int) req.param);


    if (req.param >= 0 && req.param < 29)
        res.error = rtl.setTunerGain(req.param);
    else
        res.error = 1;

    ROS_INFO("sending back response error: [%d]", res.error);
    return true;
}

bool set_tuner_gain_mode(rosrtlsdr::ParamSet::Request  &req,
                         rosrtlsdr::ParamSet::Response &res)
{
    ROS_INFO("set_tuner_gain_mode request: mode=%d", (int) req.param);


    if (req.param >= 0 && req.param < 29)
        res.error = rtl.setTunerGainMode(req.param);
    else
        res.error = 1;

    ROS_INFO("sending back response error: [%d]", res.error);
    return true;
}

bool set_num_samples_to_read(rosrtlsdr::ParamSet::Request  &req,
                             rosrtlsdr::ParamSet::Response &res)
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

    ros::ServiceServer service_cf  = n.advertiseService("set_center_freq", set_center_freq);
    ros::ServiceServer service_sr  = n.advertiseService("set_sample_rate", set_sample_rate);
    ros::ServiceServer service_ns  = n.advertiseService("set_num_samples_to_read", set_num_samples_to_read);
    ros::ServiceServer service_tg  = n.advertiseService("set_tuner_gain", set_tuner_gain);
    ros::ServiceServer service_tgm = n.advertiseService("set_tuner_gain_mode", set_tuner_gain_mode);

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
                    rtl.setSampleRate(2097152);

                    std_msgs::UInt32 sample_rate, center_freq;
                    sample_rate.data = rtl.getSampleRate();
                    pub_sample_rate.publish(sample_rate);
                    center_freq.data = rtl.getCenterFreq();
                    pub_center_freq.publish(center_freq);

                    last_publish = std::chrono::system_clock::now();

                    ROS_INFO("RTL-SDR center_freq : %.3f MHz", rtl.getCenterFreq() / 1e6);
                    ROS_INFO("RTL-SDR sample_rate : %.3f MHz", rtl.getSampleRate() / 1e6);
                    ROS_INFO("RTL-SDR num_samples_to_read : %d, %f seconds", N, N / (sample_rate.data * 1.0f));

                    //ROS_INFO("%s", rtl.getGainsString().c_str());
                }
            }
            else
            {
                if (rtl.toFile(N, "/media/ramdisk/samples" + std::to_string(active_file) + ".bin"))
                {
                    std_msgs::String str;
                    str.data = "/media/ramdisk/samples" + std::to_string(active_file) + ".bin";
                    pub_path.publish(str);

                    if (++active_file >= max_files)
                        active_file = 0;

                    end             = std::chrono::system_clock::now();
                    elapsed_seconds = end - start;
                    dt              = elapsed_seconds.count();

                    start = std::chrono::system_clock::now();
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
        if (elapsed_seconds.count() > 3.0)
        {
            std_msgs::UInt32 sample_rate, center_freq;
            sample_rate.data = rtl.getSampleRate();
            pub_sample_rate.publish(sample_rate);
            center_freq.data = rtl.getCenterFreq();
            pub_center_freq.publish(center_freq);

            ROS_INFO("%sRTL-SDR publishing sample_rate : %.3f MHz%s", BLUE_COLOR, sample_rate.data / 1e6, END_COLOR);
            ROS_INFO("%sRTL-SDR publishing center_freq : %.3f MHz%s", BLUE_COLOR, center_freq.data / 1e6, END_COLOR);

            last_publish = std::chrono::system_clock::now();


            ROS_INFO("RTL-SDR %d samples (%f seconds) reading time: %f", N, N / (sample_rate.data * 1.0f), dt);
        }

        ros::spinOnce();
    }

    return 0;
}
