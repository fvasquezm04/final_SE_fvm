/*
 * ============================================================
 *  SPI.cpp  –  Implementación de la clase SpiBus
 * ============================================================ */

#include "SPI.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "SPI";

/* ──────────────────────────────────────────
 *  Constructor / Destructor
 * ────────────────────────────────────────── */
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
    /*
     * Desregistra el dispositivo y libera el bus al destruir el objeto.
     */
    if (_device) {
        spi_bus_remove_device(_device);
        _device = nullptr;
    }
    spi_bus_free(_host);
}

/* ──────────────────────────────────────────
 *  init()
 * ────────────────────────────────────────── */
void SpiBus::init()
{
    /*
     * spi_bus_config_t: define el BUS físico (pines MOSI, MISO, CLK).
     * quadwp / quadhd son para modo QSPI → ponlos en -1 si no se usan.
     * max_transfer_sz: máximo de bytes por transacción.
     *   0 = valor por defecto del driver (4092 bytes sin DMA).
     */
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

    /*
     * SPI_DMA_DISABLED: sin DMA, adecuado para transferencias cortas.
     * Para bloques grandes (ej. pantallas, SD) usa SPI_DMA_CH_AUTO.
     */
    esp_err_t ret = spi_bus_initialize(_host, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar bus SPI: %s",
                 esp_err_to_name(ret));
        return;
    }

    /*
     * spi_device_interface_config_t: define UN dispositivo sobre el bus.
     * Se pueden añadir varios dispositivos con distintos CS, todos
     * comparten los mismos pines MOSI/MISO/CLK.
     *
     * .mode combina CPOL y CPHA:
     *   Modo 0 → CPOL=0 CPHA=0: reloj inicia bajo, captura en subida
     *   Modo 1 → CPOL=0 CPHA=1: reloj inicia bajo, captura en bajada
     *   Modo 2 → CPOL=1 CPHA=0: reloj inicia alto, captura en bajada
     *   Modo 3 → CPOL=1 CPHA=1: reloj inicia alto, captura en subida
     *
     * .queue_size: transacciones encoladas simultáneamente.
     *   1 es suficiente para uso síncrono (bloqueante).
     *
     * .pre_cb / .post_cb: callbacks opcionales antes/después de cada
     *   transacción. Útiles para CS manual o delays especiales.
     */
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

/* ──────────────────────────────────────────
 *  transfer()
 * ────────────────────────────────────────── */
esp_err_t SpiBus::transfer(const uint8_t *tx_data,
                            uint8_t       *rx_data,
                            size_t         len)
{
    if (len == 0 || !_device) return ESP_ERR_INVALID_ARG;

    /*
     * spi_transaction_t: describe UNA transacción SPI completa.
     *
     * .length   → longitud en BITS de los datos a enviar (no bytes)
     * .rxlength → longitud en BITS a recibir (igual a length en full-duplex)
     * .tx_buffer → puntero al buffer de salida (MOSI)
     * .rx_buffer → puntero al buffer de entrada (MISO)
     *
     * El driver maneja el CS automáticamente:
     *   lo baja justo antes de la transacción y lo sube al terminar.
     */
    spi_transaction_t trans = {};
    trans.length    = len * 8;
    trans.rxlength  = len * 8;
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;

    /*
     * spi_device_transmit: envío BLOQUEANTE.
     * El hilo espera hasta que la transacción finaliza.
     * Para uso no bloqueante existe spi_device_queue_trans +
     * spi_device_get_trans_result, pero no es necesario aquí.
     */
    esp_err_t ret = spi_device_transmit(_device, &trans);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en transfer (len=%d): %s",
                 (int)len, esp_err_to_name(ret));
    }
    return ret;
}

/* ──────────────────────────────────────────
 *  writeRegister()
 * ────────────────────────────────────────── */
esp_err_t SpiBus::writeRegister(uint8_t reg_addr, uint8_t data)
{
    /*
     * Patrón estándar de escritura (ADXL345, BMI160, etc.):
     *   tx[0] = reg_addr & 0x7F  → MSB=0 indica escritura
     *   tx[1] = data
     *
     * Si tu sensor usa un patrón distinto, llama a transfer()
     * directamente construyendo tus propios bytes de comando.
     */
    uint8_t tx[2] = { static_cast<uint8_t>(reg_addr & 0x7F), data };
    uint8_t rx[2] = { 0, 0 };

    return transfer(tx, rx, 2);
}

/* ──────────────────────────────────────────
 *  readRegisters()
 * ────────────────────────────────────────── */
esp_err_t SpiBus::readRegisters(uint8_t  reg_addr,
                                 uint8_t *data,
                                 size_t   len)
{
    if (len == 0) return ESP_ERR_INVALID_ARG;

    /*
     * Patrón estándar de lectura:
     *   tx[0] = reg_addr | 0x80         → MSB=1 indica lectura
     *           | 0x40 si len > 1       → bit 6=1 para multi-byte
     *                                     (solo en algunos chips)
     *   tx[1..len] = 0x00 (bytes dummy) → el maestro sigue pulsando
     *                                     el reloj para recibir datos
     *
     * rx[0] → basura (llegó mientras se enviaba el comando)
     * rx[1..len] → datos útiles del sensor
     *
     * NOTA: algunos sensores no usan el bit 6 para multi-byte.
     * Elimina el "| (len > 1 ? 0x40 : 0x00)" si no aplica.
     */
    size_t total = len + 1;  // +1 por el byte de comando

    uint8_t tx[total];
    uint8_t rx[total];
    memset(tx, 0x00, total);

    tx[0] = reg_addr | 0x80 | (len > 1 ? 0x40 : 0x00);

    esp_err_t ret = transfer(tx, rx, total);

    if (ret == ESP_OK) {
        /* Descartamos rx[0] (basura) y copiamos rx[1..total-1] */
        memcpy(data, &rx[1], len);
    }
    return ret;
}