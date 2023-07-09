#ifndef __MMAP__
#define __MMAP__

#include "Arduino.h" 
#include "EEPROM.h" 

#define MMAP_MAGIC_NUMBER 0xDEAD
#define MMAP_MAX_ADDRESS  512

typedef enum mmap_address_enum
{
  MMAP_MAGIC_NUMBER_ADDR       = 0,
  
  MMAP_GYRO_BASE_ADDR        = MMAP_MAGIC_NUMBER_ADDR + 2,
  MMAP_GYRO_CALIBRATED_ADDR  = MMAP_GYRO_BASE_ADDR + 0,  //1 byte
  MMAP_GYRO_OFFSET_QUAT_ADDR = MMAP_GYRO_BASE_ADDR + 1,  //16 bytes

  MMAP_ACCEL_BASE_ADDR       = MMAP_GYRO_OFFSET_QUAT_ADDR + 16,
  MMAP_ACCEL_CALIBRATED_ADDR = MMAP_ACCEL_BASE_ADDR + 0, //1 byte
  MMAP_ACCEL_B_ADDR          = MMAP_ACCEL_BASE_ADDR + 1,  //16 bytes
  MMAP_ACCEL_G_INV_ADDR      = MMAP_ACCEL_BASE_ADDR + 17, //16 bytes
  
  MMAP_MAG_OFFSET_BASE_ADDR  = MMAP_ACCEL_G_INV_ADDR + 16,
  MMAP_MAG_OFFSET_CALIBRATED = MMAP_MAG_OFFSET_BASE_ADDR + 0, //1 byte
  MMAP_MAG_OFFSET_QUAT_ADDR  = MMAP_MAG_OFFSET_BASE_ADDR + 1, //16 bytes

  MMAP_MAG_BASE_ADDR         = MMAP_MAG_OFFSET_QUAT_ADDR + 16,
  MMAP_MAG_CALIBRATED_ADDR   = MMAP_MAG_BASE_ADDR + 0,
  MMAP_MAG_A_ADDR            = MMAP_MAG_BASE_ADDR + 1, //64 bytes
}mmap_address;

int      mmap_init();
//int    mmap_read (mmap_address address, int num_bytes);
//void   mmap_write(mmap_address address, int val, int num_bytes);
//float  mmap_read_f (mmap_address address);
//void   mmap_write_f(mmap_address address, float val);

void   mmap_read (mmap_address address, int num_bytes, void* returned_val);
void   mmap_write(mmap_address address, int num_bytes, void* val);

#endif   //__MMAP__
