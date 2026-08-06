//
// Created by user on 31.07.26.
//
#include "main.h"
#include "MPU6050.h"
#include "mpu6050_my.h"
#include <math.h>
#include "i2c.h"
/* Interrupt detection Flags */
volatile uint8_t motion_detection_int = 0;
volatile uint8_t data_ready_int = 0;
// Other - no need

MPU6050_TypeDef mpu6050_device = {
    .chip_id = MPU6050_CHIP_ADRR_2_DEFAULT,
    .intf_ptr = &hi2c1,
    .read = mpu6050_user_read,
    .write = mpu6050_user_write,
    .delay = mpu6050_user_delay,
    .callback = MPU6050_Callback_my
};
MPU6050_InterruptConfig mpu6050_int_conf = {
    .int_level = 0,
    .int_pull = 0,
    .latch_int_enable = 1,
    .int_rd_clear = 0
};

MPU6050_Accel_Data mpu6050_accel_data = {0};

float_t MPU6050_Calculate_Roll(MPU6050_Accel_Data *accel_data) {
    float_t roll = atan2f(accel_data->acc_y, accel_data->acc_z) * (180.0f / (float_t)M_PI);
    return roll;
}

uint8_t mpu6050_user_read(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void *intf_ptr) {
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef*)intf_ptr;
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_CHIP_ADRR_2_DEFAULT, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 100) != HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t mpu6050_user_write(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void *intf_ptr) {
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef*)intf_ptr;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_CHIP_ADRR_2_DEFAULT, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 100) != HAL_OK) {
        return 1;
    }
    return 0;
}

void mpu6050_user_delay(uint32_t period) {
    HAL_Delay(period);
}

MPU6050_TypeDef* MPU6050_GetHandleTypeDef() {
    return &mpu6050_device;
}
MPU6050_Accel_Data* MPU6050_GetAccelDataStruct() {
    return &mpu6050_accel_data;
}

MPU6050_InterruptConfig* MPU6050_GetInterruptConfigStruct() {
    return &mpu6050_int_conf;
}

void MPU6050_Callback_my(MPU6050_TypeDef *dev, uint8_t status) {
    if (status & MPU6050_INT_DATA_RDY_MASK) {
        data_ready_int = 1;
    }

    if (status & MPU6050_INT_I2C_MST_MASK) {
        // TODO
    }

    if (status & MPU6050_INT_FIFO_OVERFLOW_MASK) {
        // TODO
    }

    if (status & MPU6050_INT_MOTION_MASK) {
        motion_detection_int = 1;
    }
}

