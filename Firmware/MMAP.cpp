#include "MMap.h"

/*----------------------------------------------------------------*/
int mmap_init()
{
  int was_already_initalized = 1;
  uint16_t magic_johnson;
  mmap_read (MMAP_MAGIC_NUMBER_ADDR, 2, &magic_johnson);
  if(magic_johnson != MMAP_MAGIC_NUMBER)
    {
      magic_johnson = MMAP_MAGIC_NUMBER;
      mmap_write(MMAP_MAGIC_NUMBER_ADDR, 2, &magic_johnson);
      was_already_initalized = 0;
    }

  return was_already_initalized;
}

/*----------------------------------------------------------------*/
/*
int  mmap_read (mmap_address address, int num_bytes)
{
  if((address + num_bytes) >= 512)
    return 0;
  if(address < 0)
    return 0;
 
  int result = 0;
  int n = num_bytes;
  
  while(num_bytes > 0)
    {
      int temp = EEPROM.read(address);
      temp <<= 8 * (n-num_bytes);
      result |= temp;
      address = (mmap_address)((int)address + 1);
      --num_bytes;
    }

   return result;
}
*/
/*----------------------------------------------------------------*/
/*
void  mmap_write(mmap_address address, int value, int num_bytes)
{
  if((address + num_bytes) >= 512)
    return;
  if(address < 0)
    return;
  
  while(num_bytes > 0)
    {
      EEPROM.write(address, value & 0xFF);
      value >>= 8;
      address = (mmap_address)((int)address + 1);
      --num_bytes;
    }
}
*/

/*----------------------------------------------------------------*/
/*
float  mmap_read_f (mmap_address address)
{
  //assumes float and int are both 4 bytes
  int pun = mmap_read (address, 4);
  return *((float*)(&pun));
}
*/
/*----------------------------------------------------------------*/
/*
void  mmap_write_f(mmap_address address, float value)
{
  //assumes float and int are both 4 bytes
  int pun = *((int*)(&value))
  mmap_write(address, pun, 4);
}
*/

/*----------------------------------------------------------------*/
void   mmap_read (mmap_address address, int num_bytes, void* returned_val)
{
  while(num_bytes > 0)
    {
      if(address >= MMAP_MAX_ADDRESS)
        *(uint8_t*)returned_val = 0;
      if(address < 0)
        *(uint8_t*)returned_val = 0;
      else
        *(uint8_t*)returned_val = EEPROM.read(address);
      
      ++returned_val;
      address = (mmap_address)((int)address + 1);
      --num_bytes;
    }
}

/*----------------------------------------------------------------*/
void   mmap_write(mmap_address address, int num_bytes, void* val)
{
  if((address + num_bytes) >= 512)
    return;
  if(address < 0)
    return;
  
  while(num_bytes > 0)
    {
      EEPROM.write(address, *(char*)val);
      ++val;
      address = (mmap_address)((int)address + 1);
      --num_bytes;
    }
}
