#ifndef __SPI_H__
#define __SPI_H__


#include <stdint.h>
#include <stddef.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

class SpiBus
{
public:
    SpiBus(spi_host_device_t host,
           gpio_num_t         mosi_pin,
           gpio_num_t         miso_pin,
           gpio_num_t         clk_pin,
           gpio_num_t         cs_pin,
           uint32_t           freq_hz = 10000000,
           uint8_t            mode    = 0);

    ~SpiBus();

    void init();

    esp_err_t transfer(const uint8_t *tx_data,
                       uint8_t       *rx_data,
                       size_t         len);


    esp_err_t mcp4132_write_register(uint8_t reg_addr, uint8_t data);

    esp_err_t mcp4132_read_register(uint8_t  reg_addr,
                            uint8_t *data,
                            size_t   len);

private:
    spi_host_device_t  _host;
    gpio_num_t         _mosi_pin;
    gpio_num_t         _miso_pin;
    gpio_num_t         _clk_pin;
    gpio_num_t         _cs_pin;
    uint32_t           _freq_hz;
    uint8_t            _mode;

    spi_device_handle_t _device;
};

#endif