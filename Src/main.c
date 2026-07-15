#include "gy271.h"
#include "delay.h"
#include "rcc.h"
#include "uart2.h"

volatile int16_t raw_x = 0;
volatile int16_t raw_y = 0;
volatile int16_t raw_z = 0;

volatile float mag_x = 0.0f;
volatile float mag_y = 0.0f;
volatile float mag_z = 0.0f;
volatile float heading = 0.0f;
volatile float compass_degree = 0.0f;
volatile uint16_t compass_degree_int = 0;

volatile GY271_StatusTypeDef gy271_status = GY271_ERROR;

int main(void)
{
    rcc_init();
    delay_init(RCC_SYS_CLOCK_HZ);
    uart2_init();

    gy271_status = GY271_Init();
    uart2_send_string("GY271 UART2 start\r\n");

    while (1)
    {
        GY271_RawData current_raw;
        GY271_MagneticData current_mag;
        float current_heading = 0.0f;

        if (gy271_status == GY271_OK)
        {
            gy271_status = GY271_ReadData(&current_raw, &current_mag, &current_heading);
            if (gy271_status == GY271_OK)
            {
                raw_x = current_raw.x;
                raw_y = current_raw.y;
                raw_z = current_raw.z;

                mag_x = current_mag.x;
                mag_y = current_mag.y;
                mag_z = current_mag.z;

                heading = current_heading;
                compass_degree = current_heading;
                compass_degree_int = (uint16_t)current_heading;

                uart2_send_string("heading=");
                uart2_send_float(compass_degree, 2);
                uart2_send_string(" deg\r\n");
            }
        }
        else
        {
            gy271_status = GY271_Init();
        }

        delay_ms(200);
    }
}
