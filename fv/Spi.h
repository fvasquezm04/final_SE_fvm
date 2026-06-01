#ifndef __SPI_H__
#define __SPI_H__

/*
 * ============================================================
 *  SPI.h  –  Clase genérica para bus SPI (modo Maestro)
 * ============================================================
 *
 *  ¿Qué es SPI?
 *  ─────────────
 *  Bus síncrono de 4 señales, FULL-DUPLEX: maestro y esclavo
 *  intercambian datos al mismo tiempo.
 *
 *    SCLK → Serial Clock    (reloj, siempre lo genera el maestro)
 *    MOSI → Master Out, Slave In   (maestro → esclavo)
 *    MISO → Master In,  Slave Out  (esclavo → maestro)
 *    CS   → Chip Select            (activo en bajo, uno por esclavo)
 *
 *  Diferencias clave frente a I2C:
 *  ┌─────────────────┬──────────────┬────────────────┐
 *  │                 │   I2C        │   SPI          │
 *  ├─────────────────┼──────────────┼────────────────┤
 *  │ Cables          │ 2            │ 4 (+ CS extra) │
 *  │ Velocidad       │ 100–400 kHz  │ 1–80+ MHz      │
 *  │ Dúplex          │ Half         │ Full           │
 *  │ Direccionamiento│ Sí (7 bits)  │ No (usa CS)    │
 *  │ ACK             │ Sí           │ No             │
 *  └─────────────────┴──────────────┴────────────────┘
 *
 *  Modos de reloj (CPOL / CPHA):
 *    CPOL = nivel del reloj en reposo (0 = bajo, 1 = alto)
 *    CPHA = flanco de captura        (0 = primero, 1 = segundo)
 *    → Modo 0 (CPOL=0, CPHA=0) es el más común.
 *    → SIEMPRE consulta el datasheet de tu periférico.
 *
 *  En el ESP32:
 *    SPI0/SPI1 → reservados para Flash interna (NO tocar)
 *    SPI2 (HSPI) → SPI2_HOST, disponible para usuario
 *    SPI3 (VSPI) → SPI3_HOST, disponible para usuario
 *
 *  Shift register (cómo funciona internamente):
 *    Con cada pulso de SCLK, un bit sale del maestro por MOSI
 *    y otro entra desde el esclavo por MISO. Tras 8 pulsos
 *    se completa el intercambio de un byte en ambas direcciones.
 *    → Si solo quieres LEER: envía bytes dummy (0x00 / 0xFF).
 *    → Si solo quieres ESCRIBIR: ignora los bytes de rx_data.
 *
 * ============================================================
 *  USO RÁPIDO
 * ============================================================
 *
 *  // 1. Crear instancia
 *  SpiBus bus(SPI2_HOST,
 *             GPIO_NUM_23,   // MOSI
 *             GPIO_NUM_19,   // MISO
 *             GPIO_NUM_18,   // CLK
 *             GPIO_NUM_5,    // CS
 *             1000000,       // 1 MHz
 *             0);            // Modo 0
 *
 *  // 2. Inicializar
 *  bus.init();
 *
 *  // 3. Transferencia full-duplex (raw)
 *  uint8_t tx[2] = {0x80, 0x00};
 *  uint8_t rx[2] = {0};
 *  bus.transfer(tx, rx, 2);
 *
 *  // 4. Escribir un registro (patrón MSB=0 → escritura)
 *  bus.writeRegister(0x2D, 0x08);
 *
 *  // 5. Leer registros (patrón MSB=1 → lectura)
 *  uint8_t buf[6];
 *  bus.readRegisters(0x32, buf, 6);
 *
 * ============================================================ */

#include <stdint.h>
#include <stddef.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

class SpiBus
{
public:
    /*
     * Constructor
     *
     * host     : SPI2_HOST (HSPI) o SPI3_HOST (VSPI)
     * mosi_pin : GPIO MOSI
     * miso_pin : GPIO MISO
     * clk_pin  : GPIO SCLK
     * cs_pin   : GPIO CS (Chip Select)
     * freq_hz  : velocidad del bus en Hz (ej. 1000000 = 1 MHz)
     * mode     : modo SPI 0-3 (CPOL/CPHA, ver datasheet del periférico)
     */
    SpiBus(spi_host_device_t host,
           gpio_num_t         mosi_pin,
           gpio_num_t         miso_pin,
           gpio_num_t         clk_pin,
           gpio_num_t         cs_pin,
           uint32_t           freq_hz = 1000000,
           uint8_t            mode    = 0);

    ~SpiBus();

    /*
     * init()
     * Inicializa el bus SPI y añade el dispositivo esclavo.
     * Llama esto UNA SOLA VEZ antes de cualquier transacción.
     */
    void init();

    /*
     * transfer(tx_data, rx_data, len)
     * Transferencia full-duplex de `len` bytes.
     *
     * tx_data → bytes a enviar por MOSI
     * rx_data → buffer donde se guardan los bytes recibidos por MISO
     *
     * NOTA: tx_data y rx_data deben tener al menos `len` bytes.
     * En SPI siempre se envía y recibe simultáneamente.
     *
     * Retorna ESP_OK si tuvo éxito.
     */
    esp_err_t transfer(const uint8_t *tx_data,
                       uint8_t       *rx_data,
                       size_t         len);

    /*
     * writeRegister(reg_addr, data)
     * Escribe un byte en un registro del esclavo.
     *
     * Patrón estándar (ADXL345, MPU-9250, etc.):
     *   byte[0] = reg_addr & 0x7F  → MSB=0 indica escritura
     *   byte[1] = data
     *
     * IMPORTANTE: el formato exacto depende del periférico.
     * Consulta siempre el datasheet. Si tu sensor usa otro
     * patrón, usa transfer() directamente con tus propios bytes.
     *
     * Retorna ESP_OK si tuvo éxito.
     */
    esp_err_t writeRegister(uint8_t reg_addr, uint8_t data);

    /*
     * readRegisters(reg_addr, data, len)
     * Lee `len` bytes consecutivos desde `reg_addr`.
     *
     * Patrón estándar:
     *   byte[0] = reg_addr | 0x80          → MSB=1 indica lectura
     *   byte[0] = reg_addr | 0x80 | 0x40   → además bit 6=1 para
     *                                         multi-byte (ej. ADXL345)
     *
     * Los bytes útiles empiezan en rx[1] (rx[0] es basura
     * recibida mientras se enviaba el comando).
     *
     * Retorna ESP_OK si tuvo éxito.
     */
    esp_err_t readRegisters(uint8_t  reg_addr,
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

    spi_device_handle_t _device;  // Handle del esclavo SPI
};

#endif // __SPI_H__