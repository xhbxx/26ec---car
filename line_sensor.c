#include "line_sensor.h"

#include "car_config.h"
#include "infrared.h"

static const int16_t g_weights[LINE_SENSOR_COUNT] = {-3, -1, 1, 3};
static int32_t g_last_error_x100 = 0;

void LineSensor_Init(void)
{
    Infrared_Init();
    g_last_error_x100 = 0;
}

void LineSensor_Read(LineSensorData *data)
{
    uint8_t index;
    uint8_t mask;
    int32_t weight_sum = 0;

    if (data == 0) {
        return;
    }

    data->mask = 0U;
    data->active_count = 0U;
    data->all_active = 0U;
    data->line_valid = 0U;

    mask = Infrared_Read();
    for (index = 0U; index < LINE_SENSOR_COUNT; index++) {
        uint8_t active = (uint8_t)((mask >> index) & 0x01U);
        data->raw[index] = active;
        if (active != 0U) {
            data->mask |= (uint8_t)(1U << index);
            data->active_count++;
            weight_sum += g_weights[index];
        }
    }

    data->all_active = (data->mask == 0x0FU) ? 1U : 0U;
    if ((data->active_count != 0U) && (data->all_active == 0U)) {
        data->error_x100 = (weight_sum * 100L) / data->active_count;
        g_last_error_x100 = data->error_x100;
        data->line_valid = 1U;
    } else if (data->all_active != 0U) {
        data->error_x100 = 0;
        data->line_valid = 1U;
    } else {
        /* 丢线时保留最后方向，真正的直行方向由陀螺仪航向闭环兜底。 */
        data->error_x100 = g_last_error_x100;
    }
}
