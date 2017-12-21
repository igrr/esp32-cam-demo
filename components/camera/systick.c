#include "systick.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void systick_sleep(int delay_ms)
{
    vTaskDelay((delay_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}
