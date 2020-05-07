/********************************************************
 * This is an ESP-IDF library for the MCP23017 i2c port expander.
 * 
 * It is inspired by the Adafruit Arduino library.
 *  
 * 
 ********************************************************/

#ifndef _MCP23017_H_
#define _MCP23017_H_

#include "driver/i2c.h"
#include "esp_log.h"
#include <stdexcept>
#include "sdkconfig.h"

//GPIO Pins
typedef enum
{    
    MCP23017_PIN0 = 0,
    MCP23017_PIN1 = 1,
    MCP23017_PIN2 = 2,
    MCP23017_PIN3 = 3,
    MCP23017_PIN4 = 4,
    MCP23017_PIN5 = 5,
    MCP23017_PIN6 = 6,
    MCP23017_PIN7 = 7,
    MCP23017_PIN8 = 8,
    MCP23017_PIN9 = 9,
    MCP23017_PIN10 = 10,
    MCP23017_PIN11 = 11,
    MCP23017_PIN12 = 12,
    MCP23017_PIN13 = 13,
    MCP23017_PIN14 = 14,
    MCP23017_PIN15 = 15,    
} mcp23017_pin_t;

// GPIO Ports
typedef enum
{
    MCP23017_GPIOA,
    MCP23017_GPIOB
} mcp23017_gpio_t;

//Interrupt modes
typedef enum
{
    MCP23017_INT_CHANGE,
    MCP23017_INT_FALLING,
    MCP23017_INT_RISING
} mcp23017_interrupt_t;

// Registers
typedef enum
{
    MCP23017_REG_IODIRA = 0x00,
    MCP23017_REG_IODIRB,
    MCP23017_REG_IPOLA,
    MCP23017_REG_IPOLB,
    MCP23017_REG_GPINTENA,
    MCP23017_REG_GPINTENB,
    MCP23017_REG_DEFVALA,
    MCP23017_REG_DEFVALB,
    MCP23017_REG_INTCONA,
    MCP23017_REG_INTCONB,
    MCP23017_REG_GPPUA,
    MCP23017_REG_GPPUB,
    MCP23017_REG_INTFA,
    MCP23017_REG_INTFB,
    MCP23017_REG_INTCAPA,
    MCP23017_REG_INTCAPB,
    MCP23017_REG_GPIOA,
    MCP23017_REG_GPIOB,
    MCP23017_REG_OLATA,
    MCP23017_REG_OLATB
} mcp23017_reg_t;

#define MCP23017_INT_ERR 255
#define MCP23017_DEF_ADDR 0x20

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// Bit manipulation
#define SET_BIT(_byte, _bit) ((_byte) |= (1UL << (_bit)))
#define CLEAR_BIT(_byte, _bit) ((_byte) &= ~(1UL << (_bit)))
#define IS_SET(_byte, _bit) (((_byte) & (1UL << (_bit))) >> (_bit))

class MCP23017
{
public:
    MCP23017(i2c_port_t i2cPort, gpio_num_t sdaPin, gpio_num_t sclPin, uint8_t addr, int clkHz = 100000);
    ~MCP23017();

    // Individual pins
    esp_err_t setPinMode(mcp23017_pin_t pin, bool isInput);
    esp_err_t setPullUp(mcp23017_pin_t pin, bool pullUp);
    void writePin(mcp23017_pin_t pin, bool value);
    bool readPin(mcp23017_pin_t pin);

    // GPIO ports    
    esp_err_t readBothPorts(uint16_t *value);
    esp_err_t writePort(mcp23017_gpio_t port, uint8_t value);
    esp_err_t readPort(mcp23017_gpio_t port, uint8_t *value);

    // Interrupts
    void setupInterrupts(bool mirroring, uint8_t openDrain, uint8_t polarity);
    void setupInterruptPin(uint8_t pin, mcp23017_interrupt_t mode);
    mcp23017_pin_t getLastInterruptPin();
    bool getLastInterruptPinValue();

private:
    i2c_port_t mI2CPort;
    uint8_t mDeviceAddress;

    // Helper functions
    uint8_t bitForPin(mcp23017_pin_t pin);
    mcp23017_reg_t regForPin(mcp23017_pin_t pin, mcp23017_reg_t regPortA, mcp23017_reg_t regPortB);

    esp_err_t readRegister(mcp23017_reg_t reg, uint8_t *value);
    esp_err_t writeRegister(mcp23017_reg_t reg, uint8_t value);

    esp_err_t readBytes(mcp23017_reg_t startReg, uint8_t numberOfRegisters, uint8_t *data);
    esp_err_t writeBytes(mcp23017_reg_t startReg, uint8_t numberOfRegisters, uint8_t *data);

    esp_err_t updateRegisterBit(mcp23017_pin_t pin, bool value, mcp23017_reg_t regPortA, mcp23017_reg_t regPortB);

    uint8_t bitWrite(uint8_t registerByte, uint8_t bit, bool value);
    bool bitRead(uint8_t registerByte, uint8_t bit);
};

#endif
