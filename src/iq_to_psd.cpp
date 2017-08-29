#include "ros/ros.h"

#include <iostream>
#include <chrono>
#include "std_msgs/Float32MultiArray.h"
#include "ros_rtlsdr/IQSample.h"
#include "ros_rtlsdr/IQSampleArray.h"
#include "fft.h"


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

int test(float freq)
{
    // const Complex test[] = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0 };
    // CArray data(test, 8);

    std_msgs::Float32MultiArray multiarray;

    float                       sampleRate = 1024*1024;
    int                         N          = 1024*32;
    unsigned int                SIZE       = sampleRate;

    CArray                      data(SIZE);
    Complex                     c = 0;
    CArray                      result(c, N);
    multiarray.data.resize(data.size() / 2);

    for (unsigned int n = 0; n < SIZE; n++)
    {
        data[n] = cos(2.0 * M_PI * n * freq / sampleRate);
    }
    std::cout << '\n';
    std::cout << "Cicle len : " << (sampleRate / freq) << std::endl;
    std::cout << "Wave sum : " << real(data.sum()) << '\n';
    // forward fft
    std::chrono::time_point<std::chrono::system_clock> start, end;

    start = std::chrono::high_resolution_clock::now();

    std::cout << "Data size : " << data.size() << '\n';
    // for (unsigned int i = 0; (i * N / 2 + N) < SIZE; i++)
    // {
    //     std::cout << '\n';
    //     std::cout << "\tloop " << i << " to " << i * N/2 + N << '\n';
    //
    //     //CArray window;
    //     // if (data.size() <= (i * N / 2 + N))
    //     // {
    //     //     std::cout << "\tRemaining data smaller than " << N << '\n';
    //     //     std::cout << "\tresizing to " << (i * N / 2 + N) - data.size() << '\n';
    //     //     window.resize( data.size() - i * N / 2);
    //     //     window = data[std::slice(i * N / 2, window.size(), 1)];
    //     // }
    //     // else
    //     // {
    //     //     window.resize(N);
    //     //     window = data[std::slice(i * N / 2, N, 1)];
    //     // }
    //     CArray window = data[std::slice(i * N/2, N, 1)];
    //     fft(window);
    //     window /= window.size();
    //     // result[std::slice(i * N / 2, N / 2, 1) ] += window;
    //     std::cout << "\tWindow sum : " << std::abs(window.sum()) << '\n';
    //     result += window;
    // }
    // result /= (SIZE/N);
    std::cout << "result sum : " << std::abs(result.sum()) << '\n';

    fft(data);
    data /= data.size();
    end = std::chrono::system_clock::now();;
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "fft " << elapsed.count() << std::endl;


    for (unsigned long int i = 0; i < multiarray.data.size(); i++)
    {
        multiarray.data[i] = std::abs(data[i]);
        //multiarray.data[i] = real(data[i]);
    }

    pub.publish(multiarray);



    // inverse fft
    // ifft(data);
    //
    // std::cout << std::endl << "ifft" << std::endl;
    // for (int i = 0; i < 8; ++i)
    // {
    //     std::cout << data[i] << std::endl;
    // }


    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iq_to_psd");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("iqdata", 1, iqCallback);
    pub = n.advertise<std_msgs::Float32MultiArray>("psd", 1);

    float        freq = 200e3;
    unsigned int df   = 0;
    ros::Rate    r(5);
    while (ros::ok())
    {
        df = (df + 500) % 10000;
        test(freq + df);
        //test(freq);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
