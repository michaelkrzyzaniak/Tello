
#include <Arduino.h>

//float mpu_mag_factory_gain_x =  1;
//float mpu_mag_factory_gain_y =  1;
//float mpu_mag_factory_gain_z = -1;

/* if ADC0 pin is connected to ground */
#define MPU_1_SLAVE_ADDRESS   0x68
/* if ADC0 pin is connected to VCC */
#define MPU_2_SLAVE_ADDRESS   0x69
/* put MPU into pass through mode before using this from mcu */
#define MPU_MAG_SLAVE_ADDRESS 0x0C

#define MPU_CONFIG_ADDRESS         0x1A

/* degrees per second */
#define MPU_GYRO_CONFIG_ADDRESS    0x1B
#define MPU_GYRO_SENSITIVITY_250   0x00
#define MPU_GYRO_SENSITIVITY_500   0x08
#define MPU_GYRO_SENSITIVITY_1000  0x10
#define MPU_GYRO_SENSITIVITY_2000  0x18

#define MPU_ACCEL_CONFIG_ADDRESS   0x1C
#define MPU_ACCEL_SENSITIVITY_2_G  0x00
#define MPU_ACCEL_SENSITIVITY_4_G  0x08
#define MPU_ACCEL_SENSITIVITY_8_G  0x10
#define MPU_ACCEL_SENSITIVITY_16_G 0x18

#define MPU_ACCEL_X_ADDRESS        0x3B
#define MPU_ACCEL_Y_ADDRESS        0x3D
#define MPU_ACCEL_Z_ADDRESS        0x3F

#define MPU_TEMP_ADDRESS           0x41

#define MPU_GYRO_X_ADDRESS         0x43
#define MPU_GYRO_Y_ADDRESS         0x45
#define MPU_GYRO_Z_ADDRESS         0x47

/* the first 6 EXT_SENS_DATA registers of MPU9150 */
#define MPU_MAG_X_ADDRESS          0x49
#define MPU_MAG_Y_ADDRESS          0x51
#define MPU_MAG_Z_ADDRESS          0x53

#define MPU_MAG_X_NATIVE_ADDRESS   0x03
#define MPU_MAG_Y_NATIVE_ADDRESS   0x05
#define MPU_MAG_Z_NATIVE_ADDRESS   0x07

/* for sleep and wake */
#define MPU_PWR_MGMT_1_ADDRESS     0x6B
#define MPU_PWR_MGMT_2_ADDRESS     0x6C

#define MPU_FIFO_EN_ADDRESS        0x23
#define MPU_FIFO_COUNTH_ADDRESS    0x72
#define MPU_FIFO_COUNTL_ADDRESS    0x73
#define MPU_FIFO_R_W_ADDRESS       0x74

#define MPU_INT_ENABLE_ADDRESS     0x38

/* for direcly accessing mag */
#define MPU_USER_CTRL_ADDRESS      0x6A /* write bit 5 low for direct access*/
#define MPU_BYPASS_ADDRESS         0x37 /* write bit 1 high for direct access*/

/* for configuring 'external' sensor (i.e. mag) */
#define MPU_I2C_MST_CTRL_ADDRESS   0x24
#define MPU_SLV0_ADDR_ADDRESS      0x25
#define MPU_SLV0_REG_ADDRESS       0x26
#define MPU_SLV0_CTRL_ADDRESS      0x27
#define MPU_SLV1_ADDR_ADDRESS      0x28
#define MPU_SLV1_REG_ADDRESS       0x29
#define MPU_SLV1_CTRL_ADDRESS      0x2A
#define MPU_SLV1_DATA_OUT_ADDRESS  0x64
#define MPU_SLV4_ADDR_ADDRESS      0x31
#define MPU_SLV4_REG_ADDRESS       0x32
#define MPU_SLV4_DATA_OUT_ADDRESS  0x33
#define MPU_SLV4_CTRL_ADDRESS      0x34
#define MPU_SLV4_DATA_IN_ADDRESS   0x35

#define MPU_I2C_MST_DELAY_CTRL_ADDRESS 0x67
#define MPU_SMPLRT_DIV_ADDRESS        0x19
#define MPU_I2C_SLV4_CTRL_ADDRESS     0x34

/* User-defined offsets for gyroscope */
#define MPU_XG_OFFSET_H_ADDRESS        0x13
#define MPU_XG_OFFSET_L_ADDRESS        0x14
#define MPU_YG_OFFSET_H_ADDRESS        0x15
#define MPU_YG_OFFSET_L_ADDRESS        0x16
#define MPU_ZG_OFFSET_H_ADDRESS        0x17
#define MPU_ZG_OFFSET_L_ADDRESS        0x18

#define MPU_XA_OFFSET_H_ADDRESS        0x77
#define MPU_XA_OFFSET_L_ADDRESS        0x78
#define MPU_YA_OFFSET_H_ADDRESS        0x7A
#define MPU_YA_OFFSET_L_ADDRESS        0x7B
#define MPU_ZA_OFFSET_H_ADDRESS        0x7D
#define MPU_ZA_OFFSET_L_ADDRESS        0x7E

#define MPU_WRITE_BIT 0x00
#define MPU_READ_BIT  0x80

#define MPU_CS_PIN       20
#define MPU_MOSI_PIN     11
#define MPU_MISO_PIN     12
#define MPU_SCLK_PIN     13


typedef struct mpu_sensor_data_struct
{
  float accel[3]; //g
  float gyro[3];  //rad / sec
  float mag[3];   //
}mpu_sensor_data_t;

#ifdef __cplusplus
extern "C" {
#endif

uint8_t  mpu_read_8(uint8_t reg);
void     mpu_write_8(uint8_t reg, uint8_t val);
void     mpu_read_buffer(uint8_t reg, void* b, int n);
uint8_t  mpu_i2c_read_8(uint8_t address, uint8_t reg);
void     mpu_i2c_write_8(uint8_t address, uint8_t reg, uint8_t val);
uint8_t  mpu_magnetometer_read_8(uint8_t reg);
void     mpu_magnetometer_write_8(uint8_t reg, uint8_t val);
void     mpu_sleep();
void     mpu_wake();

void     mpu_set_gyro_resolution (uint8_t resolution);
void     mpu_set_accel_resolution (uint8_t resolution);
uint8_t  mpu_get_gyro_resolution ();
uint8_t  mpu_get_accel_resolution ();
void     mpu_read_sensors(mpu_sensor_data_t* data);
uint8_t  mpu_9250_init();

#ifdef __cplusplus
}
#endif
