#include <iostream>
#include "RtlSdr.hpp"

RtlSdr::RtlSdr()
{
    deviceCount = rtlsdr_get_device_count();
}

RtlSdr::~RtlSdr()
{
    close();
}

uint32_t RtlSdr::getDeviceCount()
{
    return deviceCount;
}

bool RtlSdr::open(uint32_t dev_index)
{
    int r = rtlsdr_open(&dev, (uint32_t) dev_index);

    //std::cout << "(RET) rtlsdr_set_freq_correction = " << rtlsdr_set_freq_correction(dev, 68) << std::endl;
    std::cout << "(RET) rtlsdr_reset_buffer = " << rtlsdr_reset_buffer(dev) << std::endl;
    return r >= 0;
}

void RtlSdr::close()
{
    rtlsdr_close(dev);
    std::cout << "Closing RTLSDR device" << '\n';
}

void RtlSdr::displayDevicesInfo()
{
    for (uint32_t i = 0; i < deviceCount; ++i)
    {
        std::cout << "Device " << i << std::endl;
        std::cout << "\tName: " << rtlsdr_get_device_name(i) << std::endl;

        char manufacturer[256];
        char product[256];
        char serial[256];

        rtlsdr_get_device_usb_strings(i, manufacturer, product, serial);

        std::cout << "\tManufacturer: " << manufacturer << std::endl;
        std::cout << "\tProduct: " << product << std::endl;
        std::cout << "\tSerial: " << serial << std::endl;
    }
}

int RtlSdr::setCenterFreq(uint32_t freq)
{
    return rtlsdr_set_center_freq(dev, freq);
}
uint32_t RtlSdr::getCenterFreq()
{
    return rtlsdr_get_center_freq(dev);
}
int RtlSdr::setSampleRate(uint32_t rate)
{
    return rtlsdr_set_sample_rate(dev, rate);
}

void RtlSdr::callback(unsigned char *buf, uint32_t len, void *ctx)
{
    std::cout << len << '\n';
}

bool RtlSdr::readSync(IQVector & v)
{
    if (!dev)
        return false;

    int r, n_read;
    int block_length = v.size();

    std::vector<uint8_t> buffer(block_length * 2);

    r = rtlsdr_read_sync(dev, buffer.data(), block_length * 2, &n_read);
    if (r < 0)
    {
        std::cerr << "rtlsdr_read_sync failed" << '\n';
        return false;
    }

    if (n_read != 2 * block_length)
    {
        std::cerr << "short read, samples lost" << '\n';
        return false;
    }

    for (int i = 0; i < block_length; i++)
    {
        int32_t re = buffer[2 * i];
        int32_t im = buffer[2 * i + 1];
        v[i] = std::complex<double>((re - 128) / 128.0, (im - 128) / 128.0);
    }

    return true;
}
