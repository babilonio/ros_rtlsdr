#include "ros/ros.h"

#include <iostream>
#include <chrono>

#include "ros_rtlsdr/IQSample.h"
#include "ros_rtlsdr/IQSampleArray.h"
#include "fft.h"


std::chrono::time_point<std::chrono::system_clock> start, end;

void iqCallback(const ros_rtlsdr::IQSampleArray::ConstPtr& array)
{
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "reading time: " << elapsed_seconds.count() << "s\n";
    std::cout << array->data.size() << " samples received" << "\n";
    start = std::chrono::system_clock::now();



    return;
}

int test()
{
    // const Complex test[] = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0 };
    // CArray data(test, 8);


    float sampleRate = 44.1e3;
    float freq = 2e3;
		int N = 1024;
    int SIZE = 32*N;

    CArray data(SIZE);
    Complex c = 0;
		CArray result(c,SIZE);

    for (int n = 0; n < SIZE; n++){
      data[n] = cos(2*M_PI*n*freq /sampleRate );
    }


    // forward fft
    std::chrono::time_point<std::chrono::system_clock> start, end;

    start = std::chrono::high_resolution_clock::now();

		for (long int i = 0; i < (2*SIZE/N) ; i++){
			CArray window = data[std::slice(i* N/2, N/2, 1) ];
			fft(window);
      result[std::slice(i* N/2, N/2, 1) ] += window;
    }
		result /= (2*SIZE/N);

    end = std::chrono::system_clock::now();;
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "fft " << elapsed.count() << std::endl;

    std::cout << "fft" << std::endl;
    for (int i = 0; i < 8; ++i)
    {
        std::cout << result[i] << std::endl;
    }

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

    ros::Subscriber sub = n.subscribe("iqdata", 100, iqCallback);

    while (ros::ok())
    {

        ros::spinOnce();
    }

    return 0;
}
