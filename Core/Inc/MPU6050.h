//
// Created by user on 30.07.26.
//
#include <inttypes.h>

#ifndef F407G_MPU6050_LED_PWM_MCU2_TASK_MPU6050_H
#define F407G_MPU6050_LED_PWM_MCU2_TASK_MPU6050_H


// Регістри самотестування
#define MPU6050_SELF_TEST_X      0x0D
#define MPU6050_SELF_TEST_Y      0x0E
#define MPU6050_SELF_TEST_Z      0x0F
#define MPU6050_SELF_TEST_A      0x10

// Налаштування частоти та фільтрів
#define MPU6050_SMPLRT_DIV       0x19 // Дільник частоти дискретизації
#define MPU6050_CONFIG           0x1A // Конфігурація (DLPF - цифровий фільтр низьких частот)
#define MPU6050_GYRO_CONFIG      0x1B // Налаштування гіроскопа (діапазон)
#define MPU6050_ACCEL_CONFIG     0x1C // Налаштування акселерометра (діапазон)

// Регістри FIFO та I2C
#define MPU6050_FIFO_EN          0x23 // Увімкнення буфера FIFO
#define MPU6050_I2C_MST_CTRL     0x24 // Керування I2C Master
#define MPU6050_I2C_SLV0_ADDR    0x25 // Адреса I2C Slave 0
#define MPU6050_I2C_SLV0_REG     0x26 // Регістр I2C Slave 0
#define MPU6050_I2C_SLV0_CTRL    0x27 // Керування I2C Slave 0

// Регістри переривань
#define MPU6050_INT_PIN_CFG      0x37 // Налаштування піна переривання
#define MPU6050_INT_ENABLE       0x38 // Дозвіл переривань
#define MPU6050_INT_STATUS       0x3A // Статус переривань

// Біти Регістру MPU6050_INT_PIN_CFG
#define MPU6050_BIT_INT_LEVEL       7
#define MPU6050_BIT_INT_OPEN        6
#define MPU6050_BIT_LATCH_INT_EN    5
#define MPU6050_BIT_INT_RD_CLEAR    4
#define MPU6050_FSYNC_INT_LEVEL     3
#define MPU6050_FSYNC_INT_EN        2
#define MPU6050_BIT_I2C_BYPASS_EN   1

// Біти регістру MPU6050_INT_ENABLE
#define MPU6050_DATA_RDY_EN_MASK               (0x01 << 0)
#define MPU6050_I2C_MST_INT_EN_MASK            (0x01 << 4)
#define MPU6050_FIFO_OVERFLOW_EN_MASK          (0x01 << 5)

// Біти регістру MPU6050_INT_STATUS
#define MPU6050_INT_DATA_RDY_MASK         (1 << 0)
#define MPU6050_INT_I2C_MST_MASK          (1 << 3)
#define MPU6050_INT_FIFO_OVERFLOW_MASK    (1 << 4)
#define MPU6050_INT_MOTION_MASK           (1 << 7)

// Motion Detection
#define MPU6050_MOT_THR          0x1F
#define MPU6050_MOT_DUR          0x20

// Дані акселерометра (Осі X, Y, Z)
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_ACCEL_XOUT_L     0x3C
#define MPU6050_ACCEL_YOUT_H     0x3D
#define MPU6050_ACCEL_YOUT_L     0x3E
#define MPU6050_ACCEL_ZOUT_H     0x3F
#define MPU6050_ACCEL_ZOUT_L     0x40
#define MPU6050_ACCEL_DATA_START MPU6050_ACCEL_XOUT_H

// Дані температури
#define MPU6050_TEMP_OUT_H       0x41
#define MPU6050_TEMP_OUT_L       0x42

// Дані гіроскопа (Осі X, Y, Z)
#define MPU6050_GYRO_XOUT_H      0x43
#define MPU6050_GYRO_XOUT_L      0x44
#define MPU6050_GYRO_YOUT_H      0x45
#define MPU6050_GYRO_YOUT_L      0x46
#define MPU6050_GYRO_ZOUT_H      0x47
#define MPU6050_GYRO_ZOUT_L      0x48

// Регістри керування живленням
#define MPU6050_PWR_MGMT_1       0x6B // Керування живленням 1 (Сон, вибір джерела тактування)
#define MPU6050_PWR_MGMT_2       0x6C // Керування живленням 2 (Режим очікування для окремих осей)

// Регістри FIFO (читання/запис)
#define MPU6050_FIFO_COUNTH      0x72 // Лічильник даних у FIFO (старший байт)
#define MPU6050_FIFO_COUNTL      0x73 // Лічильник даних у FIFO (молодший байт)
#define MPU6050_FIFO_R_W         0x74 // Регістр читання/запису FIFO

// Ідентифікація пристрою
#define MPU6050_WHO_AM_I         0x75 // Регістр ідентифікації (за замовчуванн

#define MPU6050_CHIP_ADRR_1         (0x69 << 1)
#define MPU6050_CHIP_ADRR_2_DEFAULT (0x68 << 1)

/* ----------- STRUCT DEFINITION ------------- */
typedef struct MPU6050_TypeDef MPU6050_TypeDef;
typedef struct MPU6050_Accel_Data MPU6050_Accel_Data;
typedef struct MPU6050_InterruptConfig MPU6050_InterruptConfig;
typedef struct MPU6050_InterruptStatus MPU6050_InterruptStatus;
typedef struct MPU6050_Accel_Calibration_Values MPU6050_Accel_Calibration_Values;

/*!
* @brief Bus communication function pointer which should be mapped to
* the platform specific read functions of the user
*
* @param[in] reg_addr       : Register address from which data is read.
* @param[out] reg_data      : Pointer to data buffer where read data is stored.
* @param[in] len            : Number of bytes of data to be read.
* @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
*                             for interface related call backs.
*/
typedef uint8_t (*mpu6050_write_fptr)(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void *intf_ptr);
typedef uint8_t (*mpu6050_read_fptr)(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void *intf_ptr);
typedef void (*mpu6050_delay_fptr)(uint32_t period);
typedef void (*mpu6050_callback_fptr)(MPU6050_TypeDef *dev, uint8_t status);


/* -------------- API STRUCTS ------------ */
struct MPU6050_TypeDef {
    uint8_t chip_id;
    void *intf_ptr;
    mpu6050_write_fptr write;
    mpu6050_read_fptr read;
    mpu6050_delay_fptr delay;
    mpu6050_callback_fptr callback;

    /* API internal filed */
    uint8_t result;
};

struct MPU6050_Accel_Data {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
};

struct MPU6050_Accel_Calibration_Values {
    int16_t acc_x_calib_val;
    int16_t acc_y_calib_val;
    int16_t acc_z_calib_val;

    // 1 - TURE, 0 - FALSE
    uint8_t is_calibrated;
};

struct MPU6050_InterruptConfig {
    uint8_t int_level;
    uint8_t int_pull;
    uint8_t latch_int_enable;
    uint8_t int_rd_clear;
    uint8_t fsync_int_level;
    uint8_t fsync_int_enable;
    uint8_t i2c_bypass_en;
};

struct MPU6050_InterruptStatus {
    uint8_t fifo_overflow;
    uint8_t i2c_int_set;
    uint8_t data_ready;
};

/* -------------- API FUNCTIONS ------------ */
uint8_t MPU6050_Init(MPU6050_TypeDef *dev);
uint8_t MPU6050_Read_Accel_Data(MPU6050_TypeDef *dev, MPU6050_Accel_Data *accel_data);
uint8_t MPU6050_Accel_Calibration(MPU6050_TypeDef *dev);
void MPU6050_IRQHandler(MPU6050_TypeDef *dev);
uint8_t MPU6050_MotionInterrupt_Config(MPU6050_TypeDef *dev);
void MPU6050_Callback(MPU6050_TypeDef *dev);

/* -------------- API PRIVATE FUNCTIONS ------------ */
uint8_t mpu6050_read_reg(MPU6050_TypeDef *dev, uint8_t reg, uint8_t *data, uint16_t len);
uint8_t mpu6050_write_reg(MPU6050_TypeDef *dev, uint8_t reg, uint8_t *data, uint16_t len);
static void mpu6050_apply_accel_calib(MPU6050_Accel_Data *accel_data);
uint8_t MPU6050_Interrupt_Conf(MPU6050_TypeDef *dev, MPU6050_InterruptConfig *cfg, uint8_t intr_type_en);


#endif //F407G_MPU6050_LED_PWM_MCU2_TASK_MPU6050_H
