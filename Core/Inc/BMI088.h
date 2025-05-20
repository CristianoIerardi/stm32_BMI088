/**
 * @file    BMI088.h
 * @brief   Header file for BMI088 IMU driver (accelerometer + gyroscope).
 * @date    Mar 7, 2025
 * @author  crist
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "stm32f4xx_hal.h"

/** @defgroup BMI088_Register_Definitions
 *  @brief BMI088 register addresses
 *  @{
 */
#define BMI_ACC_CHIP_ID          0x00  /**< Accelerometer Chip ID register */
#define BMI_ACC_DATA             0x12  /**< Accelerometer data register start address */
#define BMI_TEMP_DATA            0x22  /**< Temperature data register */
#define BMI_ACC_CONF             0x40  /**< Accelerometer configuration */
#define BMI_ACC_RANGE            0x41  /**< Accelerometer range configuration */
#define BMI_INT1_IO_CONF         0x53  /**< Accelerometer INT1/INT2 pin configuration */
#define BMI_INT1_INT2_MAP_DATA   0x58  /**< Accelerometer interrupt data mapping */
#define BMI_ACC_PWR_CONF         0x7C  /**< Accelerometer power configuration */
#define BMI_ACC_PWR_CTRL         0x7D  /**< Accelerometer power control */
#define BMI_ACC_SOFTRESET        0x7E  /**< Accelerometer software reset */

#define BMI_GYR_CHIP_ID          0x00  /**< Gyroscope Chip ID register */
#define BMI_GYR_DATA             0x02  /**< Gyroscope data register start address */
#define BMI_GYR_RANGE            0x0F  /**< Gyroscope range configuration */
#define BMI_GYR_BANDWIDTH        0x10  /**< Gyroscope bandwidth configuration */
#define BMI_GYR_SOFTRESET        0x14  /**< Gyroscope software reset */
#define BMI_GYR_INT_CTRL         0x15  /**< Gyroscope interrupt control */
#define BMI_INT3_INT4_IO_CONF    0x16  /**< Gyroscope INT3/INT4 pin configuration */
#define BMI_INT3_INT4_IO_MAP     0x18  /**< Gyroscope interrupt mapping */
/** @} */

/**
 * @brief BMI088 device structure
 */
typedef struct {

    /* SPI interface */
    SPI_HandleTypeDef *spiHandle;      /**< Pointer to SPI handle */
    GPIO_TypeDef      *csAccPinBank;   /**< GPIO port for accelerometer chip select */
    GPIO_TypeDef      *csGyrPinBank;   /**< GPIO port for gyroscope chip select */
    uint16_t           csAccPin;       /**< GPIO pin for accelerometer chip select */
    uint16_t           csGyrPin;       /**< GPIO pin for gyroscope chip select */

    /* DMA flags and buffers */
    uint8_t readingAcc;                /**< Accelerometer DMA read in progress flag */
    uint8_t readingGyr;                /**< Gyroscope DMA read in progress flag */
    uint8_t accTxBuf[8];               /**< Accelerometer TX buffer */
    uint8_t gyrTxBuf[7];               /**< Gyroscope TX buffer */
    volatile uint8_t accRxBuf[8];      /**< Accelerometer RX buffer */
    volatile uint8_t gyrRxBuf[7];      /**< Gyroscope RX buffer */

    /* Conversion factors from raw to physical units */
    float accConversion;               /**< Accelerometer conversion factor (LSB to m/s^2) */
    float gyrConversion;               /**< Gyroscope conversion factor (LSB to rad/s) */

    /* Sensor data */
    float acc_mps2[3];                 /**< Accelerometer measurements (x, y, z) in m/s^2 */
    float gyr_rps[3];                  /**< Gyroscope measurements (x, y, z) in rad/s */
    float acc_bias[3];                 /**< Accelerometer bias (x, y, z) */
    float gyr_bias[3];                 /**< Gyroscope bias (x, y, z) */

} BMI088;

/**
 * @brief Initialize BMI088 IMU device.
 * @param imu Pointer to BMI088 structure
 * @param spiHandle Pointer to SPI handle
 * @param csAccPinBank GPIO port of accelerometer CS
 * @param csAccPin GPIO pin of accelerometer CS
 * @param csGyrPinBank GPIO port of gyroscope CS
 * @param csGyrPin GPIO pin of gyroscope CS
 * @retval 0 if success, non-zero if error
 */
uint8_t BMI088_Init(BMI088 *imu,
                    SPI_HandleTypeDef *spiHandle,
                    GPIO_TypeDef *csAccPinBank, uint16_t csAccPin,
                    GPIO_TypeDef *csGyrPinBank, uint16_t csGyrPin);

/**
 * @brief Initialize accelerometer and gyroscope bias by averaging readings.
 * @param imu Pointer to BMI088 structure
 * @param cycles Number of averaging cycles
 */
void Init_BMI088_Bias(BMI088* imu, int cycles);

/**
 * @brief Read a register from the accelerometer.
 * @param imu Pointer to BMI088 structure
 * @param regAddr Register address
 * @param data Pointer to store read data
 * @retval 0 if success, non-zero if error
 */
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);

/**
 * @brief Read a register from the gyroscope.
 * @param imu Pointer to BMI088 structure
 * @param regAddr Register address
 * @param data Pointer to store read data
 * @retval 0 if success, non-zero if error
 */
uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);

/**
 * @brief Write a register on the accelerometer.
 * @param imu Pointer to BMI088 structure
 * @param regAddr Register address
 * @param data Data to write
 * @retval 0 if success, non-zero if error
 */
uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);

/**
 * @brief Write a register on the gyroscope.
 * @param imu Pointer to BMI088 structure
 * @param regAddr Register address
 * @param data Data to write
 * @retval 0 if success, non-zero if error
 */
uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);

/**
 * @brief Read accelerometer data using polling.
 * @param imu Pointer to BMI088 structure
 * @retval 0 if success, non-zero if error
 */
uint8_t BMI088_ReadAccelerometer(BMI088 *imu);

/**
 * @brief Read gyroscope data using polling.
 * @param imu Pointer to BMI088 structure
 * @retval 0 if success, non-zero if error
 */
uint8_t BMI088_ReadGyroscope(BMI088 *imu);

/**
 * @brief Start reading accelerometer data using DMA.
 * @param imu Pointer to BMI088 structure
 * @retval 0 if success, non-zero if error
 */
uint8_t BMI088_ReadAccelerometerDMA(BMI088 *imu);

/**
 * @brief DMA complete callback for accelerometer.
 * @param imu Pointer to BMI088 structure
 */
void BMI088_ReadAccelerometerDMA_Complete(BMI088 *imu);

/**
 * @brief Start reading gyroscope data using DMA.
 * @param imu Pointer to BMI088 structure
 * @retval 0 if success, non-zero if error
 */
uint8_t BMI088_ReadGyroscopeDMA(BMI088 *imu);

/**
 * @brief DMA complete callback for gyroscope.
 * @param imu Pointer to BMI088 structure
 */
void BMI088_ReadGyroscopeDMA_Complete(BMI088 *imu);

#endif /* INC_BMI088_H_ */
