
#ifndef __RTLSDR_HPP__
#define __RTLSDR_HPP__

#include <cmath>
#include <complex>
#include <vector>
#include "rtl-sdr.h"

typedef std::vector<std::complex<float>> IQVector;

class RtlSdr {
private:

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
void displayDevicesInfo();

int setCenterFreq(uint32_t);
int setSampleRate(uint32_t);
bool readSync(IQVector&);

};


#endif // __RTLSDR_HPP__
