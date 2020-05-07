#include <esp_log.h>
#include <string>
#include "sdkconfig.h"
#include "mcp23017.h"

static char tag[] = "mcp23017demo";

extern "C"
{
    void app_main(void);
}

void app_main(void)
{
    MCP23017 *mcp = NULL;

    esp_err_t result;

    mcp = new MCP23017(I2C_NUM_0, GPIO_NUM_23, GPIO_NUM_19, 0x20);

    // Set pin 0 to output
    result = mcp->setPinMode(MCP23017_PIN0, false);

    // Enable pullup resistor
    if (result == ESP_OK)
    {
        result = mcp->setPullUp(MCP23017_PIN0, true);
    }

    if (result == ESP_OK)
    {
        while (true)
        {

            for (int a = 0; a < 256; a++)
            {
                ESP_LOGI(tag, "A: %d", a);
                result = mcp->writePort(MCP23017_GPIOA, a);
                if (result != ESP_OK)
                {
                    ESP_LOGE(tag, "Unable to write to port A: %s", esp_err_to_name(result));
                }

                vTaskDelay(1000 / portTICK_RATE_MS);
            }
        }
    }
    else 
    {
        ESP_LOGE(tag, "Unable to put pin 0 into write mode: %s", esp_err_to_name(result));
    }
}
