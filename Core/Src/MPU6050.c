//
// Created by user on 30.07.26.
//
#include <stdint.h>
#include <stddef.h>
#include "MPU6050.h"

#include "stm32f4xx_hal.h"

MPU6050_Accel_Calibration_Values accel_calib_values = {0};

uint8_t MPU6050_Init(MPU6050_TypeDef *dev) {
    uint8_t data = 0;
    uint8_t rslt = mpu6050_read_reg(dev, MPU6050_WHO_AM_I, &data, 1);
    if (data == (MPU6050_CHIP_ADRR_2_DEFAULT >> 1)) {
        mpu6050_write_reg(dev, MPU6050_PWR_MGMT_1, 0x00,1);
        uint8_t dlpf_val = 0x04;
        mpu6050_write_reg(dev, MPU6050_CONFIG, &dlpf_val, 1);
        return 0;
    } else {
        return 1;
    }
    return 0;
}

uint8_t MPU6050_Read_Accel_Data(MPU6050_TypeDef *dev, MPU6050_Accel_Data *accel_data) {
    uint8_t raw_data_buffer[6];
    mpu6050_read_reg(dev,MPU6050_ACCEL_DATA_START, raw_data_buffer, 6);
    accel_data->acc_x = (int16_t)((raw_data_buffer[0] << 8) | raw_data_buffer[1]);
    accel_data->acc_y = (int16_t)((raw_data_buffer[2] << 8) | raw_data_buffer[3]);
    accel_data->acc_z = (int16_t)((raw_data_buffer[4] << 8) | raw_data_buffer[5]);

    /* Apply Calibration */
    if (accel_calib_values.is_calibrated == 1) {
        mpu6050_apply_accel_calib(accel_data);
    }
    return 0;
}

uint8_t MPU6050_Accel_Calibration(MPU6050_TypeDef *dev) {
    long x_sum = 0, y_sum = 0, z_sum = 0;
    MPU6050_Accel_Data temp_data_for_calibration = {0};
    int32_t samples = 1000;
    for (int i=0; i<samples; i++) {
        MPU6050_Read_Accel_Data(dev, &temp_data_for_calibration);
        x_sum += temp_data_for_calibration.acc_x;
        y_sum += temp_data_for_calibration.acc_y;
        z_sum += temp_data_for_calibration.acc_z;
        HAL_Delay(3);
    }

        accel_calib_values.acc_x_calib_val = (int16_t)(x_sum / samples);
        accel_calib_values.acc_y_calib_val = (int16_t)(y_sum / samples);
        accel_calib_values.acc_z_calib_val = (int16_t)((z_sum / samples) - 16384);
        accel_calib_values.is_calibrated = 1;

    return 0;
}

static void mpu6050_apply_accel_calib(MPU6050_Accel_Data *accel_data) {
    accel_data->acc_x -= accel_calib_values.acc_x_calib_val;
    accel_data->acc_y -= accel_calib_values.acc_y_calib_val;
    accel_data->acc_z -= accel_calib_values.acc_z_calib_val;
}

uint8_t mpu6050_read_reg(MPU6050_TypeDef *dev, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t rslt;
    rslt = dev->read(reg, data, len, dev->intf_ptr);
    return rslt;
}

uint8_t mpu6050_write_reg(MPU6050_TypeDef *dev, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t rslt;
    rslt = dev->write(reg, data, len, dev->intf_ptr);
    return rslt;
}

uint8_t MPU6050_Interrupt_Conf(MPU6050_TypeDef *dev, MPU6050_InterruptConfig *cfg, uint8_t intr_type_en) {
    // Read
    uint8_t reg_data_conf = 0;
    uint8_t reg_enable = 0;
    uint8_t reg_data_conf_mask = 0;
    mpu6050_read_reg(dev, MPU6050_INT_PIN_CFG, &reg_data_conf, 1);
    mpu6050_read_reg(dev, MPU6050_INT_ENABLE, &reg_enable, 1);

    // Modify
    reg_enable &= ~intr_type_en; // clear
    reg_enable |= intr_type_en; // write new val
    reg_data_conf_mask = (cfg->int_level << 7) | (cfg->int_pull << 6) \
                            | (cfg->latch_int_enable << 5) | (cfg->int_rd_clear << 4) \
                            | (cfg->fsync_int_level << 3) | (cfg->fsync_int_enable << 2) \
                            | (cfg->i2c_bypass_en << 1);
    reg_data_conf &= ~0xFE;
    reg_data_conf |= reg_data_conf_mask;
    // Write
    mpu6050_write_reg(dev, MPU6050_INT_PIN_CFG, &reg_data_conf, 1);
    mpu6050_write_reg(dev, MPU6050_INT_ENABLE, &reg_enable, 1);
    return 0;
}

/* Interrupt handler function, must be called from User IRQ handler*/
void MPU6050_IRQHandler(MPU6050_TypeDef *dev) {
    uint8_t status = 0;

    /* Read SR registry and instantly clear */
    mpu6050_read_reg(dev, MPU6050_INT_STATUS, &status, 1);

    /* Check which INT is occurs
     * --- The User can copy this "if checks" and apply own logic ---
     */
    if (status & MPU6050_INT_DATA_RDY_MASK) {
        // TODO
    }

    if (status & MPU6050_INT_I2C_MST_MASK) {
        // TODO
    }

    if (status & MPU6050_INT_FIFO_OVERFLOW_MASK) {
        // TODO
    }

    if (status & MPU6050_INT_MOTION_MASK) {
        // TODO
    }

    /* Call user callback */
    if (dev->callback != NULL)
        /* User by yourself deside what to do with certain status */
        dev->callback(dev, status);
    else return;

}

// Це кастомний колбек який має написати сам користувач він вилкикається у MPU6050_IRQHandler для додаткової логіки якщо таку хоче корситувач
__weak void MPU6050_Callback(MPU6050_TypeDef *dev) {

}

uint8_t MPU6050_MotionInterrupt_Config(MPU6050_TypeDef *dev) {
    // 1. Config Low POwer Accelrometer (LPA)
    // 2. Впевнетись що акселеромтр парцює
    // PWR_MGMT_1 cycle=0 and sleep = 0, PWR_MGMT_2 make STBY_XA STBY_YA STBY_ZA = 0
    // 3. In ACCEL_CONFIG set ACCEL_HPF = 0
    // 4. Set Accel HPF to 256hz Bandwith in CONFIG  DLPF_CFG = 0
    // 5. Enable motion interrupt INT_ENABLE whole reg to 0x40
    // 6. in MOT_DUR (0x20) set val 1
    // 7. In MOT_THR (0x1F) set val 1-255, good val is 20
    // 8. give 1ms delay
    // 9. in ACCEL_CONFIG set ACCEL_HPF = 7
    // 10. PWR_MGMT_2 set bsit 6 and 7 to desire val 5hz
    // 11. Enable cycle mode - PWR_MGMT_1 cycle=1 and sleep = 0
    uint8_t reg = 0;
    // Step 1
    mpu6050_read_reg(dev, MPU6050_PWR_MGMT_1, &reg, 1);
    reg &= ~(0x03 << 5);
    uint8_t pwr_mgmt1 = reg;
    mpu6050_write_reg(dev, MPU6050_PWR_MGMT_1, &reg, 1);

    mpu6050_read_reg(dev, MPU6050_PWR_MGMT_2, &reg, 1);
    reg &= ~(0x07 << 3);
    uint8_t pwr_mgmt2 = reg;
    mpu6050_write_reg(dev, MPU6050_PWR_MGMT_2, &reg, 1);

    // Step 2
    mpu6050_read_reg(dev, MPU6050_ACCEL_CONFIG, &reg, 1);
    reg &= ~(0x07 << 0);
    uint8_t accel_hpf = reg;
    mpu6050_write_reg(dev, MPU6050_ACCEL_CONFIG, &reg, 1);

    // Step 3
    mpu6050_read_reg(dev, MPU6050_CONFIG, &reg, 1);
    reg &= ~(0x07 << 0);
    mpu6050_write_reg(dev, MPU6050_CONFIG, &reg, 1);

    // Step 4
    mpu6050_read_reg(dev, MPU6050_INT_ENABLE, &reg, 1);
    reg &= ~(0xFF << 0);
    reg |= 0x40;
    mpu6050_write_reg(dev, MPU6050_INT_ENABLE, &reg, 1);

    // Step 5
    mpu6050_read_reg(dev, MPU6050_MOT_DUR, &reg, 1);
    reg &= ~(0xFF << 0);
    reg |= 0x01;
    mpu6050_write_reg(dev, MPU6050_MOT_DUR, &reg, 1);

    // Step 6
    mpu6050_read_reg(dev, MPU6050_MOT_THR, &reg, 1);
    reg &= ~(0xFF << 0);
    reg |= 0x20; // Small motion interrupt
    mpu6050_write_reg(dev, MPU6050_MOT_THR, &reg, 1);

    // Step 7 TODO ділає має бути 1 ms але HAL_delay вроді більший
    dev->delay(1);

    // Step 8
    accel_hpf |= (0x07 << 0);
    mpu6050_write_reg(dev, MPU6050_ACCEL_CONFIG, &accel_hpf, 1);

    // Step 9
    pwr_mgmt2 |= (0x01 << 6);
    pwr_mgmt2 |= (0x07 << 0);
    mpu6050_write_reg(dev, MPU6050_PWR_MGMT_2, &pwr_mgmt2, 1);

    // Step 10
    pwr_mgmt1 |= (0x01 << 5);
    mpu6050_write_reg(dev, MPU6050_PWR_MGMT_1, &pwr_mgmt1, 1);
    return 0;
}
