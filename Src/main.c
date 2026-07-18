#include "gy271.h"
#include "delay.h"
#include "rcc.h"
#include "uart2.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define COMPASS_CALIBRATION_TIME_MS 5000U
#define COMPASS_CALIBRATION_SAMPLE_DELAY_MS 20U
#define COMPASS_MIN_AXIS_SPAN 1.0f
#define COMPASS_KALMAN_PROCESS_NOISE 0.05f
#define COMPASS_KALMAN_MEASUREMENT_NOISE_STILL 20.0f
#define COMPASS_KALMAN_MEASUREMENT_NOISE_TURN 0.1f
#define COMPASS_KALMAN_TURN_ERROR_THRESHOLD_DEG 5.0f

volatile int16_t raw_x = 0;
volatile int16_t raw_y = 0;
volatile int16_t raw_z = 0;

volatile float mag_x = 0.0f;
volatile float mag_y = 0.0f;
volatile float mag_z = 0.0f;
volatile float heading = 0.0f;
volatile float heading_raw = 0.0f;
volatile float compass_degree = 0.0f;
volatile uint16_t compass_degree_int = 0;

volatile GY271_StatusTypeDef gy271_status = GY271_ERROR;

static float compass_offset_x = 0.0f;
static float compass_offset_y = 0.0f;
static float compass_offset_z = 0.0f;
static float compass_scale_x = 1.0f;
static float compass_scale_y = 1.0f;
static float compass_scale_z = 1.0f;
static float compass_kalman_angle = 0.0f;
static float compass_kalman_error = 1.0f;
static uint8_t compass_kalman_initialized = 0;

static float Compass_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Compass_LimitMin(float value, float min_value)
{
    if (Compass_AbsFloat(value) < min_value)
    {
        return (value < 0.0f) ? -min_value : min_value;
    }

    return value;
}

static float Compass_CalculateHeading(float x, float y)
{
    float result = atan2f(y, x) * 180.0f / (float)M_PI;

    if (result < 0.0f)
    {
        result += 360.0f;
    }

    return result;
}

static float Compass_NormalizeAngle(float angle)
{
    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }

    while (angle < 0.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static float Compass_WrapAngleError(float error)
{
    while (error > 180.0f)
    {
        error -= 360.0f;
    }

    while (error < -180.0f)
    {
        error += 360.0f;
    }

    return error;
}

static float Compass_GetAdaptiveMeasurementNoise(float angle_error)
{
    float abs_error = Compass_AbsFloat(angle_error);

    if (abs_error < COMPASS_KALMAN_TURN_ERROR_THRESHOLD_DEG)
    {
        return COMPASS_KALMAN_MEASUREMENT_NOISE_STILL;
    }

    return COMPASS_KALMAN_MEASUREMENT_NOISE_TURN;
}

static float Compass_KalmanUpdate(float measurement)
{
    float kalman_gain;
    float error;
    float measurement_noise;

    if (compass_kalman_initialized == 0)
    {
        compass_kalman_angle = Compass_NormalizeAngle(measurement);
        compass_kalman_error = 1.0f;
        compass_kalman_initialized = 1;
        return compass_kalman_angle;
    }

    compass_kalman_error += COMPASS_KALMAN_PROCESS_NOISE;

    error = Compass_WrapAngleError(measurement - compass_kalman_angle);
    measurement_noise = Compass_GetAdaptiveMeasurementNoise(error);
    kalman_gain = compass_kalman_error / (compass_kalman_error + measurement_noise);

    compass_kalman_angle += kalman_gain * error;
    compass_kalman_angle = Compass_NormalizeAngle(compass_kalman_angle);
    compass_kalman_error = (1.0f - kalman_gain) * compass_kalman_error;

    return compass_kalman_angle;
}

static GY271_StatusTypeDef Compass_Calibrate(void)
{
    GY271_RawData sample;
    GY271_StatusTypeDef status;
    int16_t min_x = 32767;
    int16_t min_y = 32767;
    int16_t min_z = 32767;
    int16_t max_x = -32768;
    int16_t max_y = -32768;
    int16_t max_z = -32768;
    uint32_t elapsed_ms = 0;
    uint16_t sample_count = 0;

    uart2_send_string("Calib start: rotate sensor for 5s\r\n");

    while (elapsed_ms < COMPASS_CALIBRATION_TIME_MS)
    {
        status = GY271_ReadRawData(&sample);
        if (status == GY271_OK)
        {
            if (sample.x < min_x) min_x = sample.x;
            if (sample.x > max_x) max_x = sample.x;
            if (sample.y < min_y) min_y = sample.y;
            if (sample.y > max_y) max_y = sample.y;
            if (sample.z < min_z) min_z = sample.z;
            if (sample.z > max_z) max_z = sample.z;

            sample_count++;
        }

        delay_ms(COMPASS_CALIBRATION_SAMPLE_DELAY_MS);
        elapsed_ms += COMPASS_CALIBRATION_SAMPLE_DELAY_MS;
    }

    if (sample_count == 0)
    {
        return GY271_ERROR;
    }

    float span_x = Compass_LimitMin((float)(max_x - min_x), COMPASS_MIN_AXIS_SPAN);
    float span_y = Compass_LimitMin((float)(max_y - min_y), COMPASS_MIN_AXIS_SPAN);
    float span_z = Compass_LimitMin((float)(max_z - min_z), COMPASS_MIN_AXIS_SPAN);
    float avg_span = (span_x + span_y + span_z) / 3.0f;

    compass_offset_x = ((float)max_x + (float)min_x) / 2.0f;
    compass_offset_y = ((float)max_y + (float)min_y) / 2.0f;
    compass_offset_z = ((float)max_z + (float)min_z) / 2.0f;

    compass_scale_x = avg_span / span_x;
    compass_scale_y = avg_span / span_y;
    compass_scale_z = avg_span / span_z;

    uart2_send_string("Calib done\r\n");
    uart2_send_string("offset x=");
    uart2_send_float(compass_offset_x, 2);
    uart2_send_string(" y=");
    uart2_send_float(compass_offset_y, 2);
    uart2_send_string(" z=");
    uart2_send_float(compass_offset_z, 2);
    uart2_send_string("\r\n");

    return GY271_OK;
}

static void Compass_ApplyCalibration(const GY271_RawData *rawData, GY271_MagneticData *calibratedData)
{
    calibratedData->x = ((float)rawData->x - compass_offset_x) * compass_scale_x;
    calibratedData->y = ((float)rawData->y - compass_offset_y) * compass_scale_y;
    calibratedData->z = ((float)rawData->z - compass_offset_z) * compass_scale_z;
}

int main(void)
{
    rcc_init();
    delay_init(RCC_SYS_CLOCK_HZ);
    uart2_init();

    gy271_status = GY271_Init();
    uart2_send_string("GY271 UART2 start\r\n");
    if (gy271_status == GY271_OK)
    {
        gy271_status = Compass_Calibrate();
        compass_kalman_initialized = 0;
    }

    while (1)
    {
        GY271_RawData current_raw;
        GY271_MagneticData current_mag;
        float current_raw_heading = 0.0f;
        float current_filtered_heading = 0.0f;

        if (gy271_status == GY271_OK)
        {
            gy271_status = GY271_ReadRawData(&current_raw);
            if (gy271_status == GY271_OK)
            {
                Compass_ApplyCalibration(&current_raw, &current_mag);
                current_raw_heading = Compass_CalculateHeading(current_mag.x, current_mag.y);
                current_filtered_heading = Compass_KalmanUpdate(current_raw_heading);

                raw_x = current_raw.x;
                raw_y = current_raw.y;
                raw_z = current_raw.z;

                mag_x = current_mag.x;
                mag_y = current_mag.y;
                mag_z = current_mag.z;

                heading_raw = current_raw_heading;
                heading = current_filtered_heading;
                compass_degree = current_filtered_heading;
                compass_degree_int = (uint16_t)current_filtered_heading;

                uart2_send_string("heading=");
                uart2_send_float(compass_degree, 2);
                uart2_send_string(" deg\r\n");
            }
        }
        else
        {
            gy271_status = GY271_Init();
            if (gy271_status == GY271_OK)
            {
                gy271_status = Compass_Calibrate();
                compass_kalman_initialized = 0;
            }
        }

        delay_ms(200);
    }
}
