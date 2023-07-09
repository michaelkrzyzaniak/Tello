#include "MPU9250.h"
#include <SPI.h>

/*----------------------------------------------------*/
inline void mpu_begin_transaction_for_reading()
{
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE2));

  //Wire.beginTransmission(MPU_1_SLAVE_ADDRESS);
}

/*----------------------------------------------------*/
inline void mpu_begin_transaction_for_writing()
{
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  
  //Wire.beginTransmission(MPU_1_SLAVE_ADDRESS);
}

/*----------------------------------------------------*/
inline void mpu_end_transaction()
{
  SPI.endTransaction();
  digitalWrite(MPU_CS_PIN, HIGH);

  //Wire.endTransmission();
}

/*----------------------------------------------------*/
uint8_t mpu_read_8(uint8_t reg)
{
  uint8_t result = 0;
  mpu_begin_transaction_for_reading();
  SPI.transfer(MPU_READ_BIT | reg);
  result = SPI.transfer(0x00);
  mpu_end_transaction();
  return result;
}

/*----------------------------------------------------*/
void mpu_write_8(uint8_t reg, uint8_t val)
{
  mpu_begin_transaction_for_writing();
  SPI.transfer(MPU_WRITE_BIT | reg);
  SPI.transfer(val);
  mpu_end_transaction();
}

/*----------------------------------------------------*/
inline void mpu_read_buffer(uint8_t reg, void* b, int n)
{
  mpu_begin_transaction_for_reading();
  uint8_t* _b = (uint8_t*)b;
  SPI.transfer(MPU_READ_BIT | reg);
  while(n-- > 0)
    *_b++ = SPI.transfer(0x00);
  mpu_end_transaction();
}


/*----------------------------------------------------*/
uint8_t mpu_i2c_read_8(uint8_t address, uint8_t reg)
{
  mpu_write_8(MPU_SLV4_ADDR_ADDRESS, 0x80 | address);
  mpu_write_8(MPU_SLV4_REG_ADDRESS , reg);
  mpu_write_8(MPU_SLV4_CTRL_ADDRESS, 0x80); //trigger the i2c transaction 
  delay(1);
  return mpu_read_8(MPU_SLV4_DATA_IN_ADDRESS);  
}

/*----------------------------------------------------*/
void mpu_i2c_write_8(uint8_t address, uint8_t reg, uint8_t val)
{
  mpu_write_8(MPU_SLV4_ADDR_ADDRESS, address);
  mpu_write_8(MPU_SLV4_REG_ADDRESS , reg);
  mpu_write_8(MPU_SLV4_DATA_OUT_ADDRESS, val);
  mpu_write_8(MPU_SLV4_CTRL_ADDRESS, 0x80); //trigger the i2c transaction 
  delay(1);  
}

/*----------------------------------------------------*/
uint8_t mpu_magnetometer_read_8(uint8_t reg)
{
  return mpu_i2c_read_8(MPU_MAG_SLAVE_ADDRESS,  reg);
}

/*----------------------------------------------------*/
void mpu_magnetometer_write_8(uint8_t reg, uint8_t val)
{
  mpu_i2c_write_8(MPU_MAG_SLAVE_ADDRESS,  reg, val);
}

/*----------------------------------------------------*/
/*
uint8_t mpu_high_g_read_8(uint8_t reg)
{
  return mpu_i2c_read_8(MPU_HIGH_G_SLAVE_ADDRESS,  reg);
}
*/
/*----------------------------------------------------*/
/*
void mpu_high_g_write_8(uint8_t reg, uint8_t val)
{
  mpu_i2c_write_8(MPU_HIGH_G_SLAVE_ADDRESS,  reg, val);
}
*/
/*----------------------------------------------------*/
void mpu_sleep()
{
  uint8_t val = mpu_read_8(MPU_PWR_MGMT_1_ADDRESS);
  val |= 0x40;
  mpu_write_8(MPU_PWR_MGMT_1_ADDRESS, val);
}

/*----------------------------------------------------*/
void mpu_wake()
{
  uint8_t val = mpu_read_8(MPU_PWR_MGMT_1_ADDRESS);
  delay(100);
  val &= 0xBF; //wake
  val |= 0x01; //use gyro as clock
  mpu_write_8(MPU_PWR_MGMT_1_ADDRESS, val);
  delay(200);
}

/*----------------------------------------------------*/
uint8_t mpu_get_resolution(uint8_t reg)
{
  uint8_t val;
  val = mpu_read_8(reg);
  val &= 0xE7;
  return val;
}

/*----------------------------------------------------*/
void mpu_set_resolution(uint8_t reg, uint8_t resolution)
{
  uint8_t val = mpu_get_resolution(reg);
  val |= resolution; //set sensitivity bits
  mpu_write_8(reg, val); 
}

/*----------------------------------------------------*/
void mpu_set_gyro_resolution (uint8_t resolution)
{
  mpu_set_resolution(MPU_GYRO_CONFIG_ADDRESS, resolution);
}

/*----------------------------------------------------*/
void mpu_set_accel_resolution (uint8_t resolution)
{
  mpu_set_resolution(MPU_ACCEL_CONFIG_ADDRESS, resolution);
}

/*----------------------------------------------------*/
uint8_t  mpu_get_gyro_resolution ()
{
  return mpu_get_resolution(MPU_GYRO_CONFIG_ADDRESS);
}

/*----------------------------------------------------*/
uint8_t  mpu_get_accel_resolution ()
{
  return mpu_get_resolution(MPU_ACCEL_CONFIG_ADDRESS);
}

/*----------------------------------------------------*/
//accel, temperature, gyro (BIG ENDIAN) mag (LITTLE ENDIAN) all 2 bytes
void mpu_read_sensors(mpu_sensor_data_t* data)
{
  //accel, temperature, gyro, mag, mag_overflow, and High G ACCEL
  //mpu_read_buffer(MPU_ACCEL_X_ADDRESS, b, 27);
  
  unsigned char b[20];
  mpu_read_buffer(MPU_ACCEL_X_ADDRESS, b, 20);

  //adjust magnetometer with factory offsets
  /*
  uint8_t* _b = (uint8_t*)b;
  int16_t mx = (_b[15] << 8) | _b[14];
  int16_t my = (_b[17] << 8) | _b[16];
  int16_t mz = (_b[19] << 8) | _b[18];

  mx = round(mx * mpu_mag_factory_gain_x);
  my = round(my * mpu_mag_factory_gain_y);
  mz = round(mz * mpu_mag_factory_gain_z);
  
  //swap x and y
  _b[15] = my >> 8;
  _b[14] = my & 0xFF;
  _b[17] = mx >> 8;
  _b[16] = mx & 0xFF;
  _b[19] = mz >> 8;
  _b[18] = mz & 0xFF;
  */
  
  //a.multiply_real((16  / 32767.0), a);                  //convert to g from 16g mode
  //g.multiply_real(2000 * Math.PI / (32768.0 * 180), g); //gyro to rad / sec
  //m.multiply_real(0.02222 * 4900 / 32767.0, m);         //scale to +- 45 uT

  int16_t ax = (b[0] << 8) | b[1];
  int16_t ay = (b[2] << 8) | b[3];
  int16_t az = (b[4] << 8) | b[5];
  int16_t gx = (b[8] << 8) | b[9];
  int16_t gy = (b[10] << 8) | b[11];
  int16_t gz = (b[12] << 8) | b[13];
  int16_t mx = (b[15] << 8) | b[14];
  int16_t my = (b[17] << 8) | b[16];
  int16_t mz = (b[19] << 8) | b[18];
  
  //this assumes 16G mode and 2000 deg/sec mode. Todo: find a better way
  data->accel[0] = ax * 16.0  / 32767.0;
  data->accel[1] = ay * 16.0  / 32767.0;
  data->accel[2] = az * 16.0  / 32767.0;
  data->gyro [0] = gx * (2000 * M_PI) / (32768.0 * 180);
  data->gyro [1] = gy * (2000 * M_PI) / (32768.0 * 180);
  data->gyro [2] = gz * (2000 * M_PI) / (32768.0 * 180);
  //mag has x and y sqapped and negative z
  data->mag  [0] =  my * 0.02222 * 4900 / 32767.0;
  data->mag  [1] =  mx * 0.02222 * 4900 / 32767.0;
  data->mag  [2] = -mz * 0.02222 * 4900 / 32767.0;
}

#define AK8963_CNTL      0x0A 
#define AK8963_ASAX      0x10 
#define AK8963_ASAY      0x11 
#define AK8963_ASAZ      0x12

/*----------------------------------------------------*/
void mpu_config_i2c_devices()
{
  //int adjust_x, adjust_y, adjust_z;

  //genral config of the i2c bus
  mpu_write_8(MPU_I2C_MST_CTRL_ADDRESS          , 0x5D); //set i2c clock 400 kHz
  mpu_write_8(MPU_USER_CTRL_ADDRESS             , 0x20); //disconnect aux i2c from primary i2c
  mpu_write_8(MPU_WRITE_BIT | MPU_CONFIG_ADDRESS, 0x01); //set sensor sample freq to 1 HZ

  //magnetometer configure the magnetometer's registers
  mpu_magnetometer_write_8(AK8963_CNTL          , 0x00); //power the magnetometer off
  delayMicroseconds(100);
  mpu_magnetometer_write_8(AK8963_CNTL, 0x0F);           //enter FUSE ROM access mode
  //adjust_x = mpu_magnetometer_read_8(AK8963_ASAX);       //read the factory sensitivity adjustment values gains
  //adjust_y = mpu_magnetometer_read_8(AK8963_ASAY);
  //adjust_z = mpu_magnetometer_read_8(AK8963_ASAZ);
  //mpu_mag_factory_gain_x =  ((adjust_x - 128.0) / 256.0) + 1;   //calculate the gains
  //mpu_mag_factory_gain_y =  ((adjust_y - 128.0) / 256.0) + 1;
  //mpu_mag_factory_gain_z = -(((adjust_z - 128.0) / 256.0) + 1); //z axis is off wrt accelerometer, so make the gain negative
  mpu_magnetometer_write_8(AK8963_CNTL, 0x00);           //power the magnetometer off
  delayMicroseconds(100);
  mpu_magnetometer_write_8(AK8963_CNTL, 0x16);           //16-bit resolution at 100 Hz (continuous measurement mode 2)

  //enable mpu slave 0 for continuous reading of magnetometer data
  mpu_write_8(MPU_SLV0_ADDR_ADDRESS   , 0x80 | MPU_MAG_SLAVE_ADDRESS);
  mpu_write_8(MPU_SLV0_REG_ADDRESS    , MPU_MAG_X_NATIVE_ADDRESS);
  mpu_write_8(MPU_SLV0_CTRL_ADDRESS   , 0x87); //read 7 registers (including overflow status) not byte-swapped
}

/*----------------------------------------------------*/
uint8_t mpu_9250_init()
{
  pinMode(MPU_CS_PIN, OUTPUT);
  digitalWrite(MPU_CS_PIN, HIGH);
  pinMode(MPU_SCLK_PIN, OUTPUT);
  digitalWrite(MPU_SCLK_PIN, HIGH);
  pinMode(MPU_MOSI_PIN, OUTPUT);
  pinMode(MPU_MISO_PIN, OUTPUT);
  SPI.begin();
  
  uint8_t  err = 0;

  //mpu_9250_calibrate();
  mpu_set_accel_resolution(MPU_ACCEL_SENSITIVITY_16_G);
  mpu_set_gyro_resolution (MPU_GYRO_SENSITIVITY_2000);
  mpu_wake();
  mpu_config_i2c_devices();

  return err;
}

/*----------------------------------------------------*/
// Adapted from the Sparkfun MPU9250 library (simple modifications for SPI)
// https://github.com/sparkfun/MPU-9250_Breakout/blob/master/Libraries/Arduino/src/MPU9250.cpp
// This function is not deemed suitable for the present application
// as the accelerometer calibration doesn't really make sense conceptually,
// and does not work properly, and the register
// definitions in the comments do not agree completely with the 
// data sheet. Nonetheless, it is kept here for reference.
// MK
//
// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.

/*
void mpu_9250_calibrate()
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  mpu_write_8(MPU_PWR_MGMT_1_ADDRESS, 0x80);
  delay(100);
   
  // get stable time source; Auto select clock source to be PLL gyroscope
  // reference if ready else use the internal oscillator, bits 2:0 = 001
  mpu_write_8(MPU_PWR_MGMT_1_ADDRESS, 0x01);  
  mpu_write_8(MPU_WRITE_BIT | MPU_PWR_MGMT_2_ADDRESS, 0x00);
  delay(200);

  // Configure device for bias calculation
  mpu_write_8(MPU_INT_ENABLE_ADDRESS, 0x00);   // Disable all interrupts
  mpu_write_8(MPU_FIFO_EN_ADDRESS, 0x00);      // Disable FIFO
  mpu_write_8(MPU_PWR_MGMT_1_ADDRESS, 0x00);   // Turn on internal clock source
  mpu_write_8(MPU_I2C_MST_CTRL_ADDRESS, 0x00); // Disable I2C master
  mpu_write_8(MPU_USER_CTRL_ADDRESS, 0x00);    // Disable FIFO and I2C master modes
  mpu_write_8(MPU_USER_CTRL_ADDRESS, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
  // Configure MPU6050 gyro and accelerometer for bias calculation
  mpu_write_8(MPU_CONFIG_ADDRESS, 0x01);       // Set low-pass filter to 188 Hz
  mpu_write_8(MPU_SMPLRT_DIV_ADDRESS, 0x00);   // Set sample rate to 1 kHz
  mpu_write_8(MPU_GYRO_CONFIG_ADDRESS, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  mpu_write_8(MPU_ACCEL_CONFIG_ADDRESS, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  mpu_write_8(MPU_USER_CTRL_ADDRESS, 0x40);   // Enable FIFO  
  mpu_write_8(MPU_FIFO_EN_ADDRESS, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  mpu_write_8(MPU_FIFO_EN_ADDRESS, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  mpu_read_buffer(MPU_FIFO_COUNTH_ADDRESS, &data[0], 2); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    mpu_read_buffer(MPU_READ_BIT | MPU_FIFO_R_W_ADDRESS, &data[0], 12); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
  // Push gyro biases to hardware registers
  mpu_write_8(MPU_XG_OFFSET_H_ADDRESS, data[0]);
  mpu_write_8(MPU_XG_OFFSET_L_ADDRESS, data[1]);
  mpu_write_8(MPU_YG_OFFSET_H_ADDRESS, data[2]);
  mpu_write_8(MPU_YG_OFFSET_L_ADDRESS, data[3]);
  mpu_write_8(MPU_ZG_OFFSET_H_ADDRESS, data[4]);
  mpu_write_8(MPU_ZG_OFFSET_L_ADDRESS, data[5]);

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  mpu_read_buffer(MPU_READ_BIT | MPU_XA_OFFSET_H_ADDRESS, &data[0], 2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  mpu_read_buffer(MPU_READ_BIT | MPU_YA_OFFSET_H_ADDRESS, &data[0], 2);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  mpu_read_buffer(MPU_READ_BIT | MPU_ZA_OFFSET_H_ADDRESS, &data[0], 2);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  mpu_write_8(MPU_XA_OFFSET_H_ADDRESS, data[0]);
  mpu_write_8(MPU_XA_OFFSET_L_ADDRESS, data[1]);
  mpu_write_8(MPU_YA_OFFSET_H_ADDRESS, data[2]);
  mpu_write_8(MPU_YA_OFFSET_L_ADDRESS, data[3]);
  mpu_write_8(MPU_ZA_OFFSET_H_ADDRESS, data[4]);
  mpu_write_8(MPU_ZA_OFFSET_L_ADDRESS, data[5]);
}
*/
