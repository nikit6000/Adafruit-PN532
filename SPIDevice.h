#ifndef _SPI_DEVICE_H_
#define _SPI_DEVICE_H_

#include <driver/spi_master.h>
#include <esp_err.h>

#define SPI_SLAVE_DEVICE_DUMMY_BUFFER_SIZE      (256)

class SPISlaveDevice
{
public:
    enum State {
        idle,
        configured
    };

    struct Config
    {
        int mosiPin;
        int misoPin;
        int clkPin;
        int csPin;
        spi_host_device_t host;
    };
    

public:
    SPISlaveDevice(Config config);

    esp_err_t configure(int clock);
    esp_err_t transmitReceive(uint8_t* txBuffer, size_t txLen, uint8_t* rxBuffer, size_t rxLen);
    esp_err_t transmit(uint8_t* txBuffer, size_t txLen);
    esp_err_t receive(uint8_t* rxBuffer, size_t rxLen);

    ~SPISlaveDevice();
private:
   spi_device_handle_t _spiDev;
   uint8_t _dummyBytes[SPI_SLAVE_DEVICE_DUMMY_BUFFER_SIZE];
   State _currentState;
   Config _config;
};

#endif
