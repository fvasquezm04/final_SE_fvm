
#include "SPI.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "SPI";

SpiBus::SpiBus(spi_host_device_t host,
               gpio_num_t        mosi_pin,
               gpio_num_t        miso_pin,
               gpio_num_t        clk_pin,
               gpio_num_t        cs_pin,
               uint32_t          freq_hz,
               uint8_t           mode)
    : _host(host),
      _mosi_pin(mosi_pin),
      _miso_pin(miso_pin),
      _clk_pin(clk_pin),
      _cs_pin(cs_pin),
      _freq_hz(freq_hz),
      _mode(mode),
      _device(nullptr)
{
}

SpiBus::~SpiBus()
{

    if (_device) {
        spi_bus_remove_device(_device);
        _device = nullptr;
    }
    spi_bus_free(_host);
}

void SpiBus::init()
{
    spi_bus_config_t buscfg = {
        .mosi_io_num     = _mosi_pin,
        .miso_io_num     = _miso_pin,
        .sclk_io_num     = _clk_pin,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
        .flags           = 0,
        .intr_flags      = 0,
    };

    esp_err_t ret = spi_bus_initialize(_host, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar bus SPI: %s",
                 esp_err_to_name(ret));
        return;
    }

    spi_device_interface_config_t devcfg = {
        .command_bits     = 0,
        .address_bits     = 0,
        .dummy_bits       = 0,
        .mode             = _mode,
        .duty_cycle_pos   = 0,
        .cs_ena_pretrans  = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz   = (int)_freq_hz,
        .input_delay_ns   = 0,
        .spics_io_num     = _cs_pin,
        .flags            = 0,
        .queue_size       = 1,
        .pre_cb           = nullptr,
        .post_cb          = nullptr,
    };

    ret = spi_bus_add_device(_host, &devcfg, &_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al añadir dispositivo SPI: %s",
                 esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Bus SPI listo | MOSI=GPIO%d MISO=GPIO%d CLK=GPIO%d "
                  "CS=GPIO%d | %lu Hz | Modo %d",
             (int)_mosi_pin, (int)_miso_pin, (int)_clk_pin,
             (int)_cs_pin, _freq_hz, (int)_mode);
}

esp_err_t SpiBus::transfer(const uint8_t *tx_data,
                            uint8_t       *rx_data,
                            size_t         len)
{
    if (len == 0 || !_device) return ESP_ERR_INVALID_ARG;

    spi_transaction_t trans = {};
    trans.length    = len * 8;
    trans.rxlength  = len * 8;
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;

    esp_err_t ret = spi_device_transmit(_device, &trans);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en transfer (len=%d): %s",
                 (int)len, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t SpiBus::mcp4132_write_register(uint8_t reg_addr, uint8_t data)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg_addr & 0x7F), data };
    uint8_t rx[2] = { 0, 0 };

    return transfer(tx, rx, 2);
}

esp_err_t SpiBus::mcp4132_read_register(uint8_t  reg_addr,
                                 uint8_t *data,
                                 size_t   len)
{
    if (len == 0) return ESP_ERR_INVALID_ARG;

    
    size_t total = len + 1;

    uint8_t tx[total];
    uint8_t rx[total];
    memset(tx, 0x00, total);

    tx[0] = reg_addr | 0x80 | (len > 1 ? 0x40 : 0x00);

    esp_err_t ret = transfer(tx, rx, total);

    if (ret == ESP_OK) {

        memcpy(data, &rx[1], len);
    }
    return ret;
}