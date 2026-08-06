//
// Created by user on 31.07.26.
//

#ifndef F407G_MPU6050_LED_PWM_MCU2_TASK_MPU6050_MY_H
#define F407G_MPU6050_LED_PWM_MCU2_TASK_MPU6050_MY_H
#include <math.h>



uint8_t mpu6050_user_read(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void *intf_ptr);
uint8_t mpu6050_user_write(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void *intf_ptr);
void mpu6050_user_delay(uint32_t period);
MPU6050_TypeDef* MPU6050_GetHandleTypeDef();
MPU6050_Accel_Data* MPU6050_GetAccelDataStruct();
float_t MPU6050_Calculate_Roll(MPU6050_Accel_Data *accel_data);
void MPU6050_Callback_my(MPU6050_TypeDef *dev, uint8_t status);
MPU6050_InterruptConfig* MPU6050_GetInterruptConfigStruct();

#endif //F407G_MPU6050_LED_PWM_MCU2_TASK_MPU6050_MY_H
