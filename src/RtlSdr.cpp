#include <iostream>
#include <sstream>
#include "RtlSdr.hpp"

RtlSdr::RtlSdr()
{
    deviceOk = false;
}

RtlSdr::~RtlSdr()
{
    close();
}

uint32_t RtlSdr::getDeviceCount()
{
    return rtlsdr_get_device_count();
}

bool RtlSdr::open(uint32_t dev_index)
{
    int r = rtlsdr_open(&dev, (uint32_t) dev_index);

    //std::cout << "(RET) rtlsdr_set_freq_correction = " << rtlsdr_set_freq_correction(dev, 68) << std::endl;
    std::cout << "(RET) rtlsdr_reset_buffer = " << rtlsdr_reset_buffer(dev) << std::endl;
    std::cout << "(RET) rtlsdr_set_tuner_gain_mode = " << rtlsdr_set_tuner_gain_mode(dev, 1) << std::endl;
    deviceOk = (r >= 0);
    return r >= 0;
}

void RtlSdr::close()
{
    deviceOk = false;
    rtlsdr_close(dev);
    std::cout << "Closing RTLSDR device" << '\n';
}

void RtlSdr::displayDevicesInfo()
{
    for (uint32_t i = 0; i < getDeviceCount(); ++i)
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

std::string RtlSdr::getGainsString()
{
    std::stringstream ss;
    int               gains[30];
    int               len_gains;

    len_gains = rtlsdr_get_tuner_gains(dev, gains);
    std::cout << "Tuner Gains: " << len_gains << '\n';
    if (len_gains < 30)
        for (int i = 0; i < len_gains; i++)
        {
            std::cout << "Gain " << i << " : " << gains[i] << '\n';
        }
    return ss.str();
}

int RtlSdr::setTunerGainMode(int mode)
{
    return rtlsdr_set_tuner_gain_mode(dev, mode);
}

int RtlSdr::setTunerGain(int gain)
{
    return rtlsdr_set_tuner_gain(dev, gain);
}
int RtlSdr::getTunerGain()
{
    return rtlsdr_get_tuner_gain(dev);
}
int RtlSdr::setTunerIFGain(int stage, int gain)
{
    return rtlsdr_set_tuner_if_gain(dev, stage, gain);
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
uint32_t RtlSdr::getSampleRate()
{
    return rtlsdr_get_sample_rate(dev);
}

int RtlSdr::setTunerBandwidth(uint32_t bw)
{
    return rtlsdr_set_tuner_bandwidth(dev, bw);
}

void RtlSdr::callback(unsigned char *buf, uint32_t len, void *ctx)
{
    std::cout << len << '\n';
}

bool RtlSdr::readSync(IQVector & v)
{
    if (!dev)
        return false;

    int                  r, n_read;
    int                  block_length = v.size();

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

bool RtlSdr::toFile(int num_samples, std::string path)
{
    if (!dev)
        return false;

    int                  r, n_read;
    int                  block_length = num_samples;

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

    std::ofstream FILE(path, std::ios::out | std::ios::binary);
    std::copy(buffer.begin(), buffer.end(), std::ostreambuf_iterator<char>(FILE));

    return true;
}
