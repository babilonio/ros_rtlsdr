#include "ros/ros.h"

#include <iostream>
#include <chrono>
#include "std_msgs/Float32MultiArray.h"
#include "ros_rtlsdr/IQSample.h"
#include "ros_rtlsdr/IQSampleArray.h"
#include <fftw3.h>

#include <stdio.h>
#include <math.h>

#define NUM_POINTS 64
#define REAL 0
#define IMAG 1

std::chrono::time_point<std::chrono::system_clock> start, end;
ros::Publisher                                     pub;


void iqCallback(const ros_rtlsdr::IQSampleArray::ConstPtr& array)
{
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "reading time: " << elapsed_seconds.count() << "s\n";
    std::cout << array->data.size() << " samples received" << "\n";
    start = std::chrono::system_clock::now();



    return;
}

void acquire_from_somewhere(fftw_complex* signal) {
        /* Generate two sine waves of different frequencies and
         * amplitudes.
         */

        int i;
        for (i = 0; i < NUM_POINTS; ++i) {
                double theta = (double)i / (double)NUM_POINTS * M_PI;

                signal[i][REAL] = 1.0 * cos(10.0 * theta) +
                                  0.5 * cos(25.0 * theta);

                signal[i][IMAG] = 1.0 * sin(10.0 * theta) +
                                  0.5 * sin(25.0 * theta);
        }
}

void do_something_with(fftw_complex* result) {
        int i;
        for (i = 0; i < NUM_POINTS; ++i) {
                double mag = sqrt(result[i][REAL] * result[i][REAL] +
                                  result[i][IMAG] * result[i][IMAG]);

                printf("%g\n", mag);
        }
}


/* Resume reading here */

int test() {
        fftw_complex signal[NUM_POINTS];
        fftw_complex result[NUM_POINTS];

        fftw_plan plan = fftw_plan_dft_1d(NUM_POINTS,
                                          signal,
                                          result,
                                          FFTW_FORWARD,
                                          FFTW_ESTIMATE);

        acquire_from_somewhere(signal);
        fftw_execute(plan);
        do_something_with(result);

        fftw_destroy_plan(plan);

        return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iq_to_psd");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("iqdata", 1, iqCallback);
    pub = n.advertise<std_msgs::Float32MultiArray>("psd", 1);

    fftw_complex signal[64];

    float        freq = 200e3;
    unsigned int df   = 0;
    ros::Rate    r(5);
    while (ros::ok())
    {
        df = (df + 500) % 10000;
        //test(freq + df);
        //test(freq);
        test();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
