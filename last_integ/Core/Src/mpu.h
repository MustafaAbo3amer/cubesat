/*
 * mpu.h
 *
 *  Created on: Nov 3, 2024
 *      Author: Magico
 */

#ifndef SRC_MPU_H_
#define SRC_MPU_H_

/*
 * mpu.c
 *
 *  Created on: Nov 3, 2024
 *      Author: Magico
 */
#include "main.h"

#include "math.h"
#define MPU6050_ADDR 0xD0 // MPU6050 I2C address (0x68 << 1 for write)
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
int16_t ax, ay, az, gx, gy, gz;
int pitch, roll;
int ax_mps2, ay_mps2, az_mps2;
I2C_HandleTypeDef hi2c1;
/* USER CODE END 0 */
void MPU6050_Init(void) {
	uint8_t check;
	uint8_t data;

	// Check if the MPU6050 is connected
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &check, 1, HAL_MAX_DELAY);
	if (check == 0x68) {
		// Power Management 1 register: wake up MPU6050
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	}
}
void MPU6050_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t data[14];

	// Read 6 bytes of data from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data,14, HAL_MAX_DELAY);

	// Convert the data
	*ax = (int16_t)((data[0] << 8) | data[1]);
	*ay = (int16_t)((data[2] << 8) | data[3]);
	*az = (int16_t)((data[4] << 8) | data[5]);
	*gx = (int16_t)((data[8] << 8) | data[9]);
	*gy = (int16_t)((data[10] << 8) | data[11]);
	*gz = (int16_t)((data[12] << 8) | data[13]);
}
void MPU6050_Read_Calculate(int *ax_mps2, int *ay_mps2, int *az_mps2, int *pitch, int *roll, int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t data[14];

    // Read 14 bytes of data from the MPU6050
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 14, HAL_MAX_DELAY);

    // Convert the data for accelerometer and gyroscope
    int16_t ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t az = (int16_t)((data[4] << 8) | data[5]);
    *gx = (int16_t)((data[8] << 8) | data[9]);
    *gy = (int16_t)((data[10] << 8) | data[11]);
    *gz = (int16_t)((data[12] << 8) | data[13]);

    // Calculate acceleration in m/s² (assuming ±2g sensitivity)
    *ax_mps2 = (ax * 9.81 / 16384.0);
    *ay_mps2 = (ay * 9.81 / 16384.0);
    *az_mps2 = (az * 9.81 / 16384.0);

    // Convert to g for pitch and roll calculations
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    // Calculate pitch and roll
    *pitch = (atan2f(ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * 180.0 / M_PI);
    *roll = (atan2f(-ax_g, az_g) * 180.0 / M_PI);
}


#endif /* SRC_MPU_H_ */
