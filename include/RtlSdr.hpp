
#ifndef __RTLSDR_HPP__
#define __RTLSDR_HPP__

#include <fstream>
#include <cmath>
#include <complex>
#include <vector>
#include <algorithm>
#include <iterator>
#include "rtl-sdr.h"

typedef std::vector<std::complex<float>> IQVector;

class RtlSdr {
private:

bool deviceOk;
uint32_t     deviceCount;
rtlsdr_dev_t * dev;
static void  callback(unsigned char *, uint32_t, void *);

public:

RtlSdr();
~RtlSdr();

bool open(uint32_t = 0);
void close();

uint32_t getDeviceCount();
uint32_t getCenterFreq();
uint32_t getSampleRate();
void displayDevicesInfo();

int setCenterFreq(uint32_t);
int setSampleRate(uint32_t);
bool readSync(IQVector&);
bool toFile(int, std::string);
bool ok(){return deviceOk;};

};


#endif // __RTLSDR_HPP__
