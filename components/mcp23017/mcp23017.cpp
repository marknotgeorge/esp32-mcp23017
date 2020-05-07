
#include "mcp23017.h"
#include "sdkconfig.h"

using namespace std;

char tag[] = "MCP23017";

/**
 * @brief Construct a new MCP23017::MCP23017 object
 * 
 * @param i2cPort The I2C port of the ESP32.
 * @param sdaPin The GPIO pin connected to SDA.
 * @param sclPin The GPIO pin connected to SCL.
 * @param addr The I2C address of the MCP23017 device. Defaults to 0x20.
 * @param clkHz The clock frequency of the I2C bus. Defaults to 100 kHz.
 */
MCP23017::MCP23017(i2c_port_t i2cPort, gpio_num_t sdaPin, gpio_num_t sclPin, uint8_t addr, int clkHz)
{
    mI2CPort = i2cPort;
    mDeviceAddress = addr;

    esp_err_t result;

    // Configure I2C Bus
    i2c_config_t i2cConfig;
    i2cConfig.mode = I2C_MODE_MASTER;
    i2cConfig.sda_io_num = sdaPin,
    i2cConfig.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2cConfig.scl_io_num = sclPin;
    i2cConfig.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2cConfig.master.clk_speed = clkHz;

    result = i2c_param_config(mI2CPort, &i2cConfig);

    if (result != ESP_OK)
    {
        throw std::runtime_error(esp_err_to_name(result));
    }

    result = i2c_driver_install(mI2CPort, I2C_MODE_MASTER, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
    if (result != ESP_OK)
    {
        throw std::runtime_error(esp_err_to_name(result));
    }

    // Set default - all pins INPUT
    writeRegister(MCP23017_REG_IODIRA, 0xff);
    writeRegister(MCP23017_REG_IODIRB, 0xff);
}

/**
 * @brief Destroy the MCP23017::MCP23017 object
 * 
 */
MCP23017::~MCP23017()
{
    i2c_driver_delete(mI2CPort);
}

/**
 * @brief Writes a single byte to an MCP23017 register.
 * 
 * @param reg The register to write to.
 * @param value The byte to write.
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::writeRegister(mcp23017_reg_t reg, uint8_t value)
{
    esp_err_t result = ESP_FAIL;

    // Set up i2c command
    i2c_cmd_handle_t command = i2c_cmd_link_create();
    i2c_master_start(command);
    i2c_master_write_byte(command, (mDeviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(command, reg, true);
    i2c_master_write_byte(command, value, true);
    i2c_master_stop(command);

    // Send command
    result = i2c_master_cmd_begin(mI2CPort, command, 1000 / portTICK_RATE_MS);

    // Free up resources
    i2c_cmd_link_delete(command);

    return result;
}

/**
 * @brief Reads a single byte from an MCP23017 register
 * 
 * @param reg The register to read.
 * @param value The value read from the register.
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::readRegister(mcp23017_reg_t reg, uint8_t *value)
{
    esp_err_t result = ESP_FAIL;

    // Send the command to prepare the MCP23017 to be read.
    i2c_cmd_handle_t command = i2c_cmd_link_create();
    i2c_master_start(command);
    i2c_master_write_byte(command, (mDeviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(command, (uint8_t)reg, true);
    i2c_master_stop(command);
    result = i2c_master_cmd_begin(mI2CPort, command, 1000 / portTICK_RATE_MS);

    if (result == ESP_FAIL)
    {
        return result;
    }

    // Now create the command to read the data
    command = i2c_cmd_link_create();
    i2c_master_start(command);
    i2c_master_write_byte(command, (mDeviceAddress << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(command, value, I2C_MASTER_NACK);
    i2c_master_stop(command);
    result = i2c_master_cmd_begin(mI2CPort, command, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(command);

    return result;
}

/**
 * @brief Returns the bit number for a particular pin.
 * 
 * @param pin The pin for which to return the bit mask.
 * 
 * @return uint8_t The pin's bit number, between 0 and 7.
 */
uint8_t MCP23017::bitForPin(mcp23017_pin_t pin)
{
    return pin % 8;
}

/**
 * @brief The correct register for a given pin
 * 
 * @param pin The pin.
 * @param regPortA The register for GPIO port A
 * @param regPortB The register for GPIO port B
 * @return mcp23017_reg_t The correct register for the pin: Pins 0-7 are Port A, 8-15 are Port B.
 */
mcp23017_reg_t MCP23017::regForPin(mcp23017_pin_t pin, mcp23017_reg_t regPortA, mcp23017_reg_t regPortB)
{
    return (pin < MCP23017_PIN8) ? regPortA : regPortB;
}

/**
 * @brief Write a particular bit into a byte.
 * 
 * @param registerByte The byte to be written to.
 * @param bit The bit number to set, between 0 and 7.
 * @param value The value to write, where true is 1 and false is 0
 * @return uint8_t The new value of registerByte.
 */
uint8_t MCP23017::bitWrite(uint8_t registerByte, uint8_t bit, bool value)
{
    if (value)
    {
        return registerByte | (1 << bit);
    }
    else
    {
        return registerByte & ~(1 << bit);
    }
}

/**
 * @brief Read a particular bit in a byte.
 * 
 * @param registerByte The byte to be read.
 * @param bit The bit number to read, between 0 & 7.
 * @return true The bit is 1
 * @return false The bit is 0
 */
bool MCP23017::bitRead(uint8_t registerByte, uint8_t bit)
{
    return (bool)((registerByte & (1 << bit)) >> bit);
}

/**
 * @brief Update a single bit in the register relating to a particular pin.
 * 
 * @param pin The particular GPIO pin, between 0 & 15.
 * @param value The value to write.
 * @param regPortA The register for port A
 * @param regPortB The register for port B
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::updateRegisterBit(mcp23017_pin_t pin, bool value, mcp23017_reg_t regPortA, mcp23017_reg_t regPortB)
{
    esp_err_t result = ESP_FAIL;
    uint8_t registerValue;
    mcp23017_reg_t reg = regForPin(pin, regPortA, regPortB);
    uint8_t bit = bitForPin(pin);
    ESP_LOGD(tag, "Pin bit: %d", bit);

    result = readRegister(reg, &registerValue);

    if (result != ESP_OK)
    {
        ESP_LOGE(tag, "Unable to read register %d : %s", reg, esp_err_to_name(result));
        return result;
    }
    ESP_LOGD(tag, "Register %d old value: %d", reg, registerValue);

    registerValue = bitWrite(registerValue, bit, value);
    ESP_LOGD(tag, "Register new value: %d", registerValue);

    result = writeRegister(reg, registerValue);

    return result;
}

/**
 * @brief Read a number of registers
 * 
 * @param startReg The register to start
 * @param numberOfRegisters the number of registers to read.
 * @param data The pointer to the byte array in which to store the read data. 
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::readBytes(mcp23017_reg_t startReg, uint8_t numberOfRegisters, uint8_t *data)
{
    esp_err_t result = ESP_FAIL;
    uint32_t i = 0;

    if (data != NULL)
    {
        i2c_cmd_handle_t command = i2c_cmd_link_create();
        i2c_master_start(command);
        i2c_master_write_byte(command, (mDeviceAddress << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(command, startReg, true);
        i2c_master_stop(command);
        result = i2c_master_cmd_begin(mI2CPort, command, 1000 / portTICK_RATE_MS);

        i2c_cmd_link_delete(command);

        if (result != ESP_OK)
        {
            ESP_LOGD(tag, "readBytes send command failed: %s", esp_err_to_name(result));
            return result;
        }

        command = i2c_cmd_link_create();
        i2c_master_start(command);
        i2c_master_write_byte(command, (mDeviceAddress << 1) | I2C_MASTER_READ, true);
        for (i = 0; i < numberOfRegisters - 1; i++)
        {
            i2c_master_read_byte(command, &data[i], I2C_MASTER_ACK);
        }
        i2c_master_read_byte(command, &data[i], I2C_MASTER_NACK);
        i2c_master_stop(command);
        result = i2c_master_cmd_begin(mI2CPort, command, 1000 / portTICK_RATE_MS);
        if (result != ESP_OK)
        {
            ESP_LOGD(tag, "readBytes read command failed: %s", esp_err_to_name(result));
        }
    }
    return result;
}

/**
 * @brief Write a number of registers.
 * 
 * @param startReg The register to start writing.
 * @param numberOfRegisters The number of registers to write.
 * @param data Pointer to the byte array of data to write.
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::writeBytes(mcp23017_reg_t startReg, uint8_t numberOfRegisters, uint8_t *data)
{
    uint32_t i = 0;
    esp_err_t result = ESP_FAIL;

    if (data != NULL)
    {
        i2c_cmd_handle_t command = i2c_cmd_link_create();
        for (i = 0; i < numberOfRegisters; i++)
        {
            i2c_master_start(command);
            i2c_master_write_byte(command, (mDeviceAddress << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(command, startReg + i, true);
            i2c_master_write_byte(command, data[i], true);
        }
        i2c_master_stop(command);
        result = i2c_master_cmd_begin(mI2CPort, command, 1000 / portTICK_RATE_MS);
        ESP_LOGD(tag, "writeBytes failed: %s", esp_err_to_name(result));
    }
    return result;
}

/**
 * @brief Read a particular GPIO port.
 * 
 * @param port The port to read.
 * @param value The variable to store the result in.
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::readPort(mcp23017_gpio_t port, uint8_t *value)
{
    mcp23017_reg_t reg = (port == MCP23017_GPIOA) ? MCP23017_REG_GPIOA : MCP23017_REG_GPIOB;

    return readRegister(reg, value);
}

/**
 * @brief Write to a particular GPIO port
 * 
 * @param port The port to write to.
 * @param value The value to write.
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::writePort(mcp23017_gpio_t port, uint8_t value)
{
    mcp23017_reg_t reg = (port == MCP23017_GPIOA) ? MCP23017_REG_OLATA : MCP23017_REG_OLATB;

    return writeRegister(reg, value);
}

/**
 * @brief Read both ports into a single 16-bit variable.
 * 
 * @param value The value read from the ports.
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::readBothPorts(uint16_t *value)
{
    uint8_t a, b;

    esp_err_t result;

    result = readRegister(MCP23017_REG_GPIOA, &a);

    if (result != ESP_OK)
    {
        ESP_LOGE(tag, "Failed to read port A: %s", esp_err_to_name(result));
        return result;
    }

    result = readRegister(MCP23017_REG_GPIOB, &b);

    if (result != ESP_OK)
    {
        ESP_LOGE(tag, "Failed to read port A: %s", esp_err_to_name(result));
        return result;
    }

    ESP_LOGD(tag, "Port values A: %d, B: %d", a, b);

    *value = b << 8;
    *value |= a;

    return result;
}

/**
 * @brief Set the mode of a particular pin.
 * 
 * @param pin The number of the pin (0-15). 
 * @param isInput True to set the pin high; false for low.
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::setPinMode(mcp23017_pin_t pin, bool isInput)
{
    return updateRegisterBit(pin, (isInput == GPIO_MODE_INPUT), MCP23017_REG_IODIRA, MCP23017_REG_IODIRB);
}

/**
 * @brief Set the pull-up resistor for a particular pin.
 * 
 * @param pin The number of the pin (0-15).
 * @param pullUp True to connect the internal pull-up resistor; false to disconnect.
 * @return esp_err_t ESP_OK if successful; ESP_FAIL otherwise.
 */
esp_err_t MCP23017::setPullUp(mcp23017_pin_t pin, bool pullUp)
{
    return updateRegisterBit(pin, (pullUp == true), MCP23017_REG_GPPUA, MCP23017_REG_GPPUB);
}
