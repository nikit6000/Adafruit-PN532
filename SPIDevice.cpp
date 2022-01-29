#include "SPIDevice.h"
#include <driver/spi_common.h>
#include <Arduino.h>

SPISlaveDevice::SPISlaveDevice(Config config): 
    _config(config)
{
     _currentState = State::idle;
     memset(_dummyBytes, 0xFF, SPI_SLAVE_DEVICE_DUMMY_BUFFER_SIZE);
}

esp_err_t SPISlaveDevice::configure(int clock) {

    if (_currentState != State::idle)
        return ESP_FAIL;

    if (!GPIO_IS_VALID_GPIO(_config.csPin))
        return ESP_FAIL;

    esp_err_t ret;
    spi_bus_config_t busConfig;
    spi_device_interface_config_t devConfig;

    memset(&busConfig, 0, sizeof(spi_bus_config_t));
    memset(&devConfig, 0, sizeof(spi_device_interface_config_t));

    busConfig.miso_io_num=_config.misoPin;
    busConfig.mosi_io_num=_config.mosiPin;
    busConfig.sclk_io_num=_config.clkPin;
    busConfig.quadwp_io_num = -1;
    busConfig.quadhd_io_num = -1;
    busConfig.max_transfer_sz = 0;

    devConfig.command_bits = 8;
    devConfig.address_bits = 0;
    devConfig.dummy_bits = 0;
    devConfig.mode = 0;
    devConfig.duty_cycle_pos = 0;  
    devConfig.cs_ena_pretrans = 0;  
    devConfig.cs_ena_posttrans = 0;
    devConfig.clock_speed_hz = clock;
    devConfig.spics_io_num = _config.csPin;
    devConfig.flags = SPI_DEVICE_TXBIT_LSBFIRST | SPI_DEVICE_RXBIT_LSBFIRST;  
    devConfig.queue_size = 1;
    devConfig.pre_cb = NULL;
    devConfig.post_cb = NULL;

    pinMode(_config.csPin, OUTPUT);

    ret = spi_bus_initialize(_config.host, &busConfig, 0);
       
    if (ret != ESP_OK)
        return ret;

    ret = spi_bus_add_device(_config.host, &devConfig, &_spiDev);

    if (ret != ESP_OK)
        return ret;

    _currentState = State::configured;

    return ret;
}

esp_err_t SPISlaveDevice::transmitReceive(uint8_t* txBuffer, size_t txLen, uint8_t* rxBuffer, size_t rxLen) {

    if (_currentState != State::configured)
        return ESP_FAIL;

    esp_err_t ret;
    spi_transaction_t transaction;

    memset(&transaction, 0, sizeof(transaction));

    ret = spi_device_acquire_bus(_spiDev, portMAX_DELAY);

    if (ret != ESP_OK)
        return ret;
    
    gpio_set_level((gpio_num_t)_config.csPin, 0);

    transaction.cmd = *txBuffer;
    transaction.rx_buffer = rxBuffer;
    transaction.tx_buffer = txBuffer + 1;
    transaction.length = (txLen + rxLen - 1) * 8;
    transaction.rxlength = rxLen * 8;

    ret = spi_device_transmit(_spiDev, &transaction);

    gpio_set_level((gpio_num_t)_config.csPin, 1);
    spi_device_release_bus(_spiDev);
    
    return ret;
}

esp_err_t SPISlaveDevice::transmit(uint8_t* txBuffer, size_t txLen) {

    if (_currentState != State::configured)
        return ESP_FAIL;

    spi_transaction_t transaction;
    esp_err_t ret;

    memset(&transaction, 0, sizeof(transaction));

    transaction.tx_buffer = txBuffer;
    transaction.length = txLen * 8;

    ret = spi_device_acquire_bus(_spiDev, portMAX_DELAY);

    if (ret != ESP_OK)
        return ret;

    gpio_set_level((gpio_num_t)_config.csPin, 0);

    ret = spi_device_transmit(_spiDev, &transaction);

    gpio_set_level((gpio_num_t)_config.csPin, 1);

    spi_device_release_bus(_spiDev);

    return ret;
}

esp_err_t SPISlaveDevice::receive(uint8_t* rxBuffer, size_t rxLen) {

    if (_currentState != State::configured || rxLen > SPI_SLAVE_DEVICE_DUMMY_BUFFER_SIZE)
        return ESP_FAIL;

    spi_transaction_t transaction;
    esp_err_t ret;

    memset(&transaction, 0, sizeof(transaction));

    transaction.rx_buffer = rxBuffer;
    transaction.tx_buffer = _dummyBytes;
    transaction.length = rxLen * 8;
    transaction.rxlength = rxLen * 8;

    ret = spi_device_acquire_bus(_spiDev, portMAX_DELAY);

    if (ret != ESP_OK)
        return ret;

    gpio_set_level((gpio_num_t)_config.csPin, 0);

    ret = spi_device_transmit(_spiDev, &transaction);

    gpio_set_level((gpio_num_t)_config.csPin, 1);

    spi_device_release_bus(_spiDev);

    return ret;
}

SPISlaveDevice::~SPISlaveDevice() {
    spi_device_release_bus(_spiDev);
}