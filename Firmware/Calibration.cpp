/*--____------_-_-_---------------_---_----------------------------------
   / ___|__ _| (_) |__  _ __ __ _| |_(_) ___  _ __    ___ 
  | |   / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \  / __|
  | |__| (_| | | | |_) | | | (_| | |_| | (_) | | | || (__ 
   \____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_(_)___|
------------------------------------------------------------------------
Created by Michael Krzyzaniak and Sergiu Baluta at Inside Coach
Copyright Inside Coach, 2017 .All rights reserved
----------------------------------------------------------------------*/
#include "Calibration.h"
#include "Statistics.h"
#include "MMap.h"

//#include <Arduino.h>

/*--------------------------------------------------------------------*/
struct opaque_calibration_struct
{
  OffsetCalibrator*     gyro_calibrator;
  WonCalibrator*        acc_calibrator;
  OffsetCalibrator*     mag_offset_calibrator;
  BalutaCalibrator*     mag_calibrator;
  
  Quaternion            filtered_gyro;
  ic_float_t            gyro_motion_threshold;
  ic_float_t            accel_gyro_motion_threshold;
  ic_float_t            mag_gyro_motion_threshold;
  
  BOOL                  was_moving;
};

BOOL calibration_monitor_and_pick_samples(Calibration* self, 
                                          Quaternion gyro_corrected, 
                                          Quaternion gyro_uncorrected, 
                                          Quaternion acc_corrected, 
                                          Quaternion acc_uncorrected,
                                          Quaternion mag_first_corrected,  
                                          Quaternion mag_uncorrected);

/*--------------------------------------------------------------------*/
Calibration* calibration_new()
{
  Calibration* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->gyro_calibrator        = offset_calibrator_new(5000/*10000*/);
      self->acc_calibrator         = won_calibrator_new();
      self->mag_offset_calibrator  = offset_calibrator_new(5000/*10000*/);
      self->mag_calibrator         = baluta_calibrator_new(3);
      
      if((self->gyro_calibrator       == NULL) ||
         (self->acc_calibrator        == NULL) ||
         (self->mag_offset_calibrator == NULL) ||
         (self->mag_calibrator        == NULL))
        return calibration_destroy(self);
        
      self->gyro_motion_threshold         = 0.04;
      self->mag_gyro_motion_threshold     = 3;
      self->accel_gyro_motion_threshold   = 0.002;

      calibration_init(self);
      
      int mmap_was_already_initalized = mmap_init();
      if(!mmap_was_already_initalized)
        calibration_save(self);
      else
        calibration_restore(self);
    }
  return self;  
}

/*--------------------------------------------------------------------*/
void calibration_init(Calibration* self)
{
  offset_calibrator_init     (self->gyro_calibrator);
  won_calibrator_init        (self->acc_calibrator);
  offset_calibrator_init     (self->mag_offset_calibrator);
  baluta_calibrator_init     (self->mag_calibrator);
  
  quaternion_set(self->filtered_gyro, 0, 10, 10, 10);
  self->was_moving = YES;
}

/*--------------------------------------------------------------------*/
Calibration* calibration_destroy(Calibration* self)
{
  if(self != NULL)
    {
      self->gyro_calibrator       = offset_calibrator_destroy     (self->gyro_calibrator);
      self->acc_calibrator        = won_calibrator_destroy        (self->acc_calibrator);
      self->mag_offset_calibrator = offset_calibrator_destroy     (self->mag_offset_calibrator);
      self->mag_calibrator        = baluta_calibrator_destroy     (self->mag_calibrator);
      
      free(self);
    }
  return (Calibration*) NULL;
}

/*--------------------------------------------------------------------*/
void calibration_save(Calibration* self)
{
  //Serial.println("saving Calibration");
  offset_calibrator_save     (self->gyro_calibrator, MMAP_GYRO_BASE_ADDR);
  won_calibrator_save        (self->acc_calibrator, MMAP_ACCEL_BASE_ADDR);
  offset_calibrator_save     (self->mag_offset_calibrator, MMAP_MAG_OFFSET_BASE_ADDR);
  baluta_calibrator_save     (self->mag_calibrator, MMAP_MAG_BASE_ADDR);
}

/*--------------------------------------------------------------------*/
void calibration_restore(Calibration* self)
{
  //Serial.println("Restoring Calibration");
  offset_calibrator_restore     (self->gyro_calibrator, MMAP_GYRO_BASE_ADDR);
  won_calibrator_restore        (self->acc_calibrator, MMAP_ACCEL_BASE_ADDR);
  offset_calibrator_restore     (self->mag_offset_calibrator, MMAP_MAG_OFFSET_BASE_ADDR);
  baluta_calibrator_restore     (self->mag_calibrator, MMAP_MAG_BASE_ADDR);
}

/*--------------------------------------------------------------------*/
BOOL calibration_calibrated(Calibration* self)
{
  return offset_calibrator_calibrated    (self->gyro_calibrator)       &
         won_calibrator_calibrated       (self->acc_calibrator)        &
         offset_calibrator_calibrated    (self->mag_offset_calibrator) &
         baluta_calibrator_calibrated    (self->mag_calibrator)        ;
}

/*--------------------------------------------------------------------*/
uint8_t calibration_calibrated_mask(Calibration* self)
{
  return offset_calibrator_calibrated    (self->gyro_calibrator)       << 0 |
         won_calibrator_calibrated       (self->acc_calibrator)        << 1 |
         offset_calibrator_calibrated    (self->mag_offset_calibrator) << 2 |
         baluta_calibrator_calibrated    (self->mag_calibrator)        << 3;
}

/*--------------------------------------------------------------------*/
//returns YES if a new sample was picked, which could mean calibration status changed
BOOL calibration_update(Calibration* self, Quaternion gyro, Quaternion acc, Quaternion mag)
{
  BOOL result = NO;
  
  Quaternion            gyro_uncorrected;
  Quaternion            acc_uncorrected;
  Quaternion            mag_uncorrected;

  quaternion_copy(gyro, gyro_uncorrected);
  quaternion_copy(acc , acc_uncorrected );
  quaternion_copy(mag , mag_uncorrected );

  offset_calibrator_correct_measurement     (self->gyro_calibrator      , gyro);
  won_calibrator_correct_measurement        (self->acc_calibrator       , acc );
  offset_calibrator_correct_measurement     (self->mag_offset_calibrator, mag);
  result = calibration_monitor_and_pick_samples(self, gyro /*corrected*/, gyro_uncorrected, acc /*corrected*/, acc_uncorrected, mag, mag_uncorrected);
  baluta_calibrator_correct_measurement     (self->mag_calibrator       , mag);


/*
  if(self->mag_calibrator.calibrated)
    self->mag_error.update(mag.norm());
  
  var norm = self->filtered_gyro.norm();
  if(self->gyro_calibrator.calibrated && (norm < self->gyro_motion_threshold))
    self->gyro_error.update(gyro.norm());

  if(self->mag_calibrator.calibrated && (norm > self->mag_gyro_motion_threshold))
    self->mag_error.update(mag.norm());

  if(self->acc_calibrator.calibrated && (norm < self->accel_gyro_motion_threshold))
    self->acc_error.update(acc.norm());
*/

  return result;
}

/*--------------------------------------------------------------------*/
/* arguments are corrected readings */
BOOL calibration_monitor_and_pick_samples(Calibration* self, Quaternion gyro_corrected, Quaternion gyro_uncorrected, Quaternion acc_corrected, Quaternion acc_uncorrected,Quaternion mag_first_corrected, Quaternion mag_uncorrected)
{
  int already_calibrated = calibration_calibrated(self);
  BOOL sample_added = NO;
  
  self->filtered_gyro[1] = (self->filtered_gyro[1] * 0.9) + (gyro_corrected[1] * 0.1);
  self->filtered_gyro[2] = (self->filtered_gyro[2] * 0.9) + (gyro_corrected[2] * 0.1);
  self->filtered_gyro[3] = (self->filtered_gyro[3] * 0.9) + (gyro_corrected[3] * 0.1);
  ic_float_t angular_vel = quaternion_norm(self->filtered_gyro);
  
  if(angular_vel < self->gyro_motion_threshold)
    self->was_moving = NO;
  else
    {
      //give the filter a kick in the pants so it doesn't oscillate about the threshhold
      if(!self->was_moving)
        quaternion_set(self->filtered_gyro, 0, 10, 10, 10);
      self->was_moving = YES;
    }
  
  if(!offset_calibrator_calibrated(self->gyro_calibrator))
    if(angular_vel < self->gyro_motion_threshold)
      sample_added = sample_added || offset_calibrator_add_sample(self->gyro_calibrator, gyro_uncorrected);

  //Serial.println(won_calibrator_calibrated(self->acc_calibrator));
  
  if(!won_calibrator_calibrated(self->acc_calibrator))
    if(angular_vel < self->accel_gyro_motion_threshold)  
      sample_added = sample_added || won_calibrator_monitor_and_pick_samples(self->acc_calibrator, acc_uncorrected);
  
  if(!offset_calibrator_calibrated(self->mag_offset_calibrator))
    if(angular_vel > self->mag_gyro_motion_threshold)
      sample_added = sample_added || offset_calibrator_add_sample(self->mag_offset_calibrator, mag_uncorrected);
      
  if(!baluta_calibrator_calibrated(self->mag_calibrator) && offset_calibrator_calibrated(self->mag_offset_calibrator))
    sample_added = sample_added || baluta_calibrator_monitor_and_pick_samples(self->mag_calibrator, mag_first_corrected);
  
  //todo: do this on a per-calibrator basis
  if((!already_calibrated) & calibration_calibrated(self))
    calibration_save(self);

  return sample_added;
};

/*--------------------------------------------------------------------*/
/*--___---__--__---------_------___------_-_-_-------------_------------
   / _ \ / _|/ _|___ ___| |_   / __|__ _| (_) |__ _ _ __ _| |_ ___ _ _ 
  | (_) |  _|  _(_-</ -_)  _| | (__/ _` | | | '_ \ '_/ _` |  _/ _ \ '_|
   \___/|_| |_| /__/\___|\__|  \___\__,_|_|_|_.__/_| \__,_|\__\___/_| 
------------------------------------------------------------------------
  find the average of the signal and subtract it out of subsequent 
  samples. Used for gyro calibration.
----------------------------------------------------------------------*/
struct opaque_offset_calibrator_struct
{
  Quaternion offsets;
  BOOL       calibrated;
  unsigned   maxnum_samples;
  unsigned   num_collected_samples;
};

/*--------------------------------------------------------------------*/
OffsetCalibrator* offset_calibrator_new(unsigned maxnum_samples)
{
  OffsetCalibrator* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->maxnum_samples = maxnum_samples;
      offset_calibrator_init(self);
    }
  return self;  
}

/*--------------------------------------------------------------------*/
void offset_calibrator_init(OffsetCalibrator* self)
{
  quaternion_zero(self->offsets);
  self->calibrated = NO;
  self->num_collected_samples = 0;
}

/*--------------------------------------------------------------------*/
OffsetCalibrator* offset_calibrator_destroy(OffsetCalibrator* self)
{
  if(self != NULL)
    {
      free(self);
    }
  return (OffsetCalibrator*) NULL;
}

/*--------------------------------------------------------------------*/
void offset_calibrator_save(OffsetCalibrator* self, int base_address)
{
  char calibrated = self->calibrated;
  mmap_write(base_address+0, 1, &calibrated);
  mmap_write(base_address+1, 16, self->offsets);
}

/*--------------------------------------------------------------------*/
void offset_calibrator_restore(OffsetCalibrator* self, int base_address)
{
  //load self->offsets as initial estimate
  //then let new sampes be grabbed
  //self->num_collected_samples = 0;
  char calibrated;
  mmap_read(base_address+0, 1,  &calibrated);
  mmap_read(base_address+1, 16, self->offsets);
  self->calibrated = calibrated;
}

/*--------------------------------------------------------------------*/
BOOL offset_calibrator_calibrated(OffsetCalibrator* self)
{
  return self->calibrated;
}

/*--------------------------------------------------------------------*/
BOOL offset_calibrator_add_sample(OffsetCalibrator* self, Quaternion q)
{
  if(!self->calibrated)
    {
      ++self->num_collected_samples;
  
      if(self->num_collected_samples == 1)
        quaternion_copy(q, self->offsets);
      else
        {
          Quaternion temp;
          quaternion_copy          (q, temp);
          quaternion_subtract      (temp, self->offsets, temp);
          quaternion_multiply_real (temp, 1.0/self->num_collected_samples, temp);
          quaternion_add           (self->offsets, temp, self->offsets);
      
          if(self->num_collected_samples >= self->maxnum_samples)
            self->calibrated = YES;
        }
    }
  return self->calibrated;
}

//PUBLIC
/*--------------------------------------------------------------------*/
void offset_calibrator_correct_measurement(OffsetCalibrator* self, Quaternion q)
{
  quaternion_subtract(q, self->offsets, q);
}

/*--------------------------------------------------------------------*/
/*-___-------_------_-----------___------_-_-_-------------_------------
  | _ ) __ _| |_  _| |_ __ _   / __|__ _| (_) |__ _ _ __ _| |_ ___ _ _ 
  | _ \/ _` | | || |  _/ _` | | (__/ _` | | | '_ \ '_/ _` |  _/ _ \ '_|
  |___/\__,_|_|\_,_|\__\__,_|  \___\__,_|_|_|_.__/_| \__,_|\__\___/_| 
------------------------------------------------------------------------
  Used for magnetometer calibration.
----------------------------------------------------------------------*/
struct opaque_baluta_calibrator_struct
{
  unsigned    num_reference_vectors;
  Quaternion* reference_vectors;
  BOOL        calibrated;
  unsigned    samples_per_reference_vector;
  
  unsigned    total_collected_samples;
  unsigned*   num_collected_samples;
  unsigned    n;
  unsigned*   i;
  unsigned    prev_reference_vector;
  
  Matrix*     samples;
  Matrix*     H;
  Matrix*     b;
  Matrix*     A;
  Matrix*     v;
  Matrix*     r;
  ic_float_t  epsilon;
  
  /* used only by baluta_calibrator_calculate_calibration_parameters */
  Matrix*     H_t;
  Matrix*     T;
  Matrix*     temp;
  Matrix*     T_inv;
  Matrix*     T_H_t;
  Matrix*     x; 
  Matrix*     _A;
};

void              baluta_calibrator_add_sample                        (BalutaCalibrator* self, Quaternion q, unsigned reference_vector);
unsigned          baluta_calibrator_index_of_nearest_reference_vector (BalutaCalibrator* self, Quaternion q);
void              baluta_calibrator_calculate_calibration_parameters  (BalutaCalibrator* self);
ic_float_t        baluta_calibrator_verify_calibration                (BalutaCalibrator* self);

/*--------------------------------------------------------------------*/
BalutaCalibrator* baluta_calibrator_new(unsigned samples_per_reference_vector)
{
  BalutaCalibrator* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->num_reference_vectors        = 20;
      self->samples_per_reference_vector = samples_per_reference_vector;
      
      self->reference_vectors       = calloc(self->num_reference_vectors, sizeof(*(self->reference_vectors)));
      self->num_collected_samples   = calloc(self->num_reference_vectors, sizeof(*(self->num_collected_samples)));
      self->i                       = calloc(self->num_reference_vectors, sizeof(*(self->i)));
      self->n                       = self->num_reference_vectors * self->samples_per_reference_vector;
      self->prev_reference_vector   = 0;
      self->samples                 = matrix_new(self->n, 3);
      self->H                       = matrix_new(self->n, 9);
      self->b                       = matrix_new(self->n, 1);
      self->A                       = matrix_new(4, 4);
      self->v                       = matrix_new(4, 1);
      self->r                       = matrix_new(4, 1);
      self->epsilon                 = -1;
      self->H_t                     = matrix_new(9, self->n); //H.columns(), H.rows
      self->T                       = matrix_new(9, 9);       //H.columns(), H.columns
      self->temp                    = matrix_new(9, 9);
      self->T_inv                   = matrix_new(9, 9);
      self->T_H_t                   = matrix_new(9, self->n);
      self->x                       = matrix_new(9, 1);
      self->_A                      = matrix_new(4, 4);
        
      if((self->reference_vectors           == NULL) ||
         (self->num_collected_samples       == NULL) ||
         (self->i                           == NULL) ||
         (self->H                           == NULL) ||
         (self->b                           == NULL) ||
         (self->A                           == NULL) ||
         (self->v                           == NULL) ||
         (self->r                           == NULL) ||
         (self->samples                     == NULL) ||
         (self->H_t                         == NULL) ||
         (self->T                           == NULL) ||
         (self->temp                        == NULL) ||
         (self->T_inv                       == NULL) ||
         (self->T_H_t                       == NULL) ||
         (self->x                           == NULL) ||
         (self->_A                          == NULL)  )
        return baluta_calibrator_destroy(self);

      quaternion_set(self->reference_vectors[0] , 0,  1,  1,  1);
      quaternion_set(self->reference_vectors[1] , 0,  1,  1, -1);
      quaternion_set(self->reference_vectors[2] , 0,  1, -1,  1);
      quaternion_set(self->reference_vectors[3] , 0,  1, -1, -1);
      quaternion_set(self->reference_vectors[4] , 0, -1,  1,  1);
      quaternion_set(self->reference_vectors[5] , 0, -1,  1, -1);
      quaternion_set(self->reference_vectors[6] , 0, -1, -1,  1);
      quaternion_set(self->reference_vectors[7] , 0, -1, -1, -1);
      quaternion_set(self->reference_vectors[8] , 0,  0,  0.61803398875,  1.61803398875);
      quaternion_set(self->reference_vectors[9] , 0,  0,  0.61803398875, -1.61803398875);
      quaternion_set(self->reference_vectors[10], 0,  0, -0.61803398875,  1.61803398875);
      quaternion_set(self->reference_vectors[11], 0,  0, -0.61803398875, -1.61803398875);
      quaternion_set(self->reference_vectors[12], 0,  0.61803398875,  1.61803398875, 0 );
      quaternion_set(self->reference_vectors[13], 0,  0.61803398875, -1.61803398875, 0 );
      quaternion_set(self->reference_vectors[14], 0, -0.61803398875,  1.61803398875, 0 );
      quaternion_set(self->reference_vectors[15], 0, -0.61803398875, -1.61803398875, 0 );
      quaternion_set(self->reference_vectors[16], 0,  1.61803398875,  0,  0.61803398875);
      quaternion_set(self->reference_vectors[17], 0,  1.61803398875,  0, -0.61803398875);
      quaternion_set(self->reference_vectors[18], 0, -1.61803398875,  0,  0.61803398875);
      quaternion_set(self->reference_vectors[19], 0, -1.61803398875,  0, -0.61803398875);      

      baluta_calibrator_init(self); 
    }
  return self;  
}

/*--------------------------------------------------------------------*/
void baluta_calibrator_init(BalutaCalibrator* self)
{
  unsigned i;  
  for(i=0; i<self->num_reference_vectors; i++)
    {
      self->num_collected_samples[i] = 0;
      self->i[i]                     = 0;
    }

  self->calibrated = NO;
  self->total_collected_samples = 0;
  self->epsilon = -1;
  matrix_fill_identity (self->A);
}

/*--------------------------------------------------------------------*/
BalutaCalibrator* baluta_calibrator_destroy(BalutaCalibrator* self)
{
  if(self != NULL)
    {
      if(self->reference_vectors != NULL)
        free(self->reference_vectors);
      
      if(self->num_collected_samples != NULL)
        free(self->num_collected_samples);
      
      if(self->i != NULL)
        free(self->i);
    
      self->H = matrix_destroy(self->H);
      self->b = matrix_destroy(self->b);
      self->A = matrix_destroy(self->A);
      self->v = matrix_destroy(self->v);
      self->r = matrix_destroy(self->r);
      
      free(self);
    }
  return (BalutaCalibrator*) NULL;
}

/*--------------------------------------------------------------------*/
void baluta_calibrator_save(BalutaCalibrator* self, int base_address)
{
  char calibrated = self->calibrated;
  mmap_write(base_address+0 , 1, &calibrated);
  mmap_write(base_address+1 , 64, matrix_values(self->A));
}

/*--------------------------------------------------------------------*/
void baluta_calibrator_restore(BalutaCalibrator* self, int base_address)
{
  char calibrated;
  mmap_write(base_address+0 , 1, &calibrated);
  mmap_write(base_address+1 , 64, matrix_values(self->A));
  self->calibrated = calibrated;
}

/*--------------------------------------------------------------------*/
BOOL baluta_calibrator_calibrated(BalutaCalibrator* self)
{
  return self->calibrated;
}

/*--------------------------------------------------------------------*/
BOOL baluta_calibrator_monitor_and_pick_samples(BalutaCalibrator* self, Quaternion uncorrected)
{
  BOOL should_add_sample = NO;
  
  if(!self->calibrated)
    {
      unsigned reference_vector = baluta_calibrator_index_of_nearest_reference_vector(self, uncorrected);
    
      if(reference_vector != self->prev_reference_vector)
        {
          self->prev_reference_vector = reference_vector;
          if(!self->calibrated)
            {
              if(self->num_collected_samples[reference_vector] < self->samples_per_reference_vector)
                should_add_sample = YES;
            }
        }
     
      if(should_add_sample)
        {
          baluta_calibrator_add_sample(self, uncorrected, reference_vector);
          if(self->total_collected_samples >= self->n)
            {
              baluta_calibrator_calculate_calibration_parameters(self);
              if(!self->calibrated) baluta_calibrator_init(self);
            }
        }
    }
  return should_add_sample;
}

/*--------------------------------------------------------------------*/
ic_float_t baluta_calibrator_verify_calibration(BalutaCalibrator* self)
{
  unsigned   i;
  Quaternion q;
  ic_float_t g; 
  ic_float_t error = 0;
  
  q[0] = 0;
  
  for(i=0; i<self->n; i++)
    {
      q[1] = matrix_get_value(self->samples, i, 0, NULL);
      q[2] = matrix_get_value(self->samples, i, 1, NULL);
      q[3] = matrix_get_value(self->samples, i, 2, NULL);
      baluta_calibrator_correct_measurement(self, q);
      g = quaternion_norm(q);
      error += fabs(1-g);
    }
  error /= self->n;
  return error;
}

/*--------------------------------------------------------------------*/
void baluta_calibrator_correct_measurement(BalutaCalibrator* self, Quaternion q)
{
  matrix_set_value(self->v, 0, 0, q[1]);
  matrix_set_value(self->v, 1, 0, q[2]);
  matrix_set_value(self->v, 2, 0, q[3]);
  matrix_set_value(self->v, 3, 0, 1   );
  
  matrix_multiply(self->A, self->v, self->r);
  
  q[0] = 0;
  q[1] = matrix_get_value(self->r, 0, 0, NULL);
  q[2] = matrix_get_value(self->r, 1, 0, NULL);
  q[3] = matrix_get_value(self->r, 2, 0, NULL);
}

//PRIVATE
/*--------------------------------------------------------------------*/
void baluta_calibrator_add_sample(BalutaCalibrator* self, Quaternion q, unsigned reference_vector)
{
  unsigned i = reference_vector * self->samples_per_reference_vector + self->i[reference_vector];
  
  ic_float_t x = q[1];
  ic_float_t y = q[2];
  ic_float_t z = q[3];
  
  //I once saw spurious -1 in self->samples; I don't know where they came from...
  //possibly sensor error, so, uh, reject samples containing -1 for now
  if((x==-1.0) || (y==-1.0) || (z==-1.0))
    return;
  
  matrix_set_value(self->samples, i, 0, x);
  matrix_set_value(self->samples, i, 1, y);
  matrix_set_value(self->samples, i, 2, z);
  
  matrix_set_value(self->H, i, 0, y*y    );
  matrix_set_value(self->H, i, 1, z*z    );
  matrix_set_value(self->H, i, 2, 2*x    );
  matrix_set_value(self->H, i, 3, 2*y    );
  matrix_set_value(self->H, i, 4, 2*z    );
  matrix_set_value(self->H, i, 5, 2*x*y  );
  matrix_set_value(self->H, i, 6, 2*x*z  );
  matrix_set_value(self->H, i, 7, 2*y*z  );
  matrix_set_value(self->H, i, 8, 1      );
  matrix_set_value(self->b, i, 0, -(x*x) );
  
  ++self->i[reference_vector]; 
  self->i[reference_vector] %= self->samples_per_reference_vector;
  
  if(self->num_collected_samples[reference_vector] < self->samples_per_reference_vector)
    ++self->num_collected_samples[reference_vector];
  
  if(self->total_collected_samples < self->n)
    ++self->total_collected_samples;
}

/*--------------------------------------------------------------------*/
//returns index of reference vector
unsigned baluta_calibrator_index_of_nearest_reference_vector(BalutaCalibrator* self, Quaternion q)
{
  unsigned i;
  unsigned index = 0;
  ic_float_t max_dot_product = 0;
  Quaternion q_normal;
  quaternion_normalize(q, q_normal);
  
  for(i=0; i<self->num_reference_vectors; i++)
    {
      ic_float_t dot = quaternion_vector_dot_product(q_normal, self->reference_vectors[i]);
      if(dot > max_dot_product)
        {
          max_dot_product = dot;
          index = i;
        }
    }

  return index;
}

/*--------------------------------------------------------------------*/
void baluta_calibrator_calculate_calibration_parameters(BalutaCalibrator* self)
{
  ic_float_t sanity;
  ic_float_t x_2_0 = matrix_get_value(self->x, 2, 0, NULL);
  ic_float_t x_5_0 = matrix_get_value(self->x, 5, 0, NULL);
  ic_float_t x_6_0 = matrix_get_value(self->x, 6, 0, NULL);
  ic_float_t temp;

  matrix_transpose(self->H, self->H_t);
  matrix_multiply (self->H_t, self->H, self->T);

  if(!matrix_invert(self->T, self->temp, self->T_inv))
    {baluta_calibrator_init(self); return;}

  matrix_multiply (self->T_inv, self->H_t, self->T_H_t);
  matrix_multiply (self->T_H_t, self->b  , self->x    );
  
  matrix_set_value(self->_A, 0, 0, 1);
  matrix_set_value(self->_A, 0, 1, x_5_0);
  matrix_set_value(self->_A, 0, 2, matrix_get_value(self->x, 6, 0, NULL));
  matrix_set_value(self->_A, 0, 3, matrix_get_value(self->x, 2, 0, NULL));
  matrix_set_value(self->_A, 1, 0, 0);

  sanity = matrix_get_value(self->x, 0, 0, NULL) - (x_5_0 * x_5_0);
  if(sanity <= 0) {baluta_calibrator_init(self); return;}
  sanity = sqrt(sanity);

  matrix_set_value(self->_A, 1, 1, sanity);
  matrix_set_value(self->_A, 1, 2, (matrix_get_value(self->x, 7, 0, NULL) - (x_6_0*x_5_0)) / sanity);
  matrix_set_value(self->_A, 1, 3, (matrix_get_value(self->x, 3, 0, NULL) - (x_5_0*x_2_0)) / sanity);
  matrix_set_value(self->_A, 2, 0, 0);
  matrix_set_value(self->_A, 2, 1, 0);

  temp =  matrix_get_value(self->_A, 1, 2, NULL);
  sanity = matrix_get_value(self->x, 1, 0, NULL) - (x_6_0*x_6_0) - (temp*temp);
  if(sanity <= 0) {baluta_calibrator_init(self); return;}
  sanity = sqrt(sanity);

  matrix_set_value(self->_A, 2, 2, sanity);
  matrix_set_value(self->_A, 2, 3, (matrix_get_value(self->x, 4, 0, NULL) - (x_6_0*x_2_0) -
                                            (temp*matrix_get_value(self->_A, 1, 3, NULL))) / sanity);
  matrix_set_value(self->_A, 3, 0, 0);
  matrix_set_value(self->_A, 3, 1, 0);
  matrix_set_value(self->_A, 3, 2, 0);
  
  temp    = matrix_get_value(self->_A, 1, 3, NULL);
  temp   *= temp;
  sanity  = matrix_get_value(self->_A, 2, 3, NULL);
  sanity *= sanity; 
  sanity  = (x_2_0*x_2_0) + temp + sanity - matrix_get_value(self->x, 8, 0, NULL);
  if(sanity <= 0) {baluta_calibrator_init(self); return;}
  sanity = sqrt(sanity);

  matrix_set_value(self->_A, 3, 3, sanity);
  
  matrix_multiply_scalar(self->_A, 1.0/sanity, self->A);

  self->epsilon = baluta_calibrator_verify_calibration(self);

  if(self->epsilon > 0.25) //todo: what is a good upper bound?
    {baluta_calibrator_init(self); return;}

  self->calibrated = YES;
}

/*--------------------------------------------------------------------*/
/*-___------------------------_-------------___------_-_---------------_       
  | _ \___ __ _ _ _ ___ _____(_)___ _ _    / __|__ _| (_) |__ _ _ __ _| |_ ___ _ _ 
  |   / -_) _` | '_/ -_|_-<_-< / _ \ ' \  | (__/ _` | | | '_ \ '_/ _` |  _/ _ \ '_|
  |_|_\___\__, |_| \___/__/__/_\___/_||_|  \___\__,_|_|_|_.__/_| \__,_|\__\___/_|  
----------|___/---------------------------------------------------------
  this module transfers calibration from one sensor to another 
  via linear regression. It is used for high-g accel
----------------------------------------------------------------------*/
struct opaque_regression_calibrator_struct
{
  unsigned          maxnum_samples;
  ic_float_t        reference_min;
  ic_float_t        reference_max;
  BOOL              calibrated[3];
  OnlineRegression* regression_vars[3];
  int               sign[3];
};

/*--------------------------------------------------------------------*/
RegressionCalibrator* regression_calibrator_new(ic_float_t reference_min, ic_float_t reference_max, int maxnum_samples)
{
  RegressionCalibrator* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->maxnum_samples     = maxnum_samples;
      self->reference_min      = reference_min;
      self->reference_max      = reference_max;
      self->regression_vars[0] = online_regression_new();
      self->regression_vars[1] = online_regression_new();
      self->regression_vars[2] = online_regression_new();
      
      if((self->regression_vars[0] == NULL) ||
         (self->regression_vars[0] == NULL) ||
         (self->regression_vars[0] == NULL))
        return regression_calibrator_destroy(self);
      
      regression_calibrator_init(self); 
    }
  return self;  
}

/*--------------------------------------------------------------------*/
void regression_calibrator_init(RegressionCalibrator* self)
{
  int i;
  for(i=0; i<3; i++)
    {
      self->sign[i]       = 0;
      self->calibrated[i] = 0;
      online_regression_init(self->regression_vars[i]);
    }
}

/*--------------------------------------------------------------------*/
RegressionCalibrator* regression_calibrator_destroy(RegressionCalibrator* self)
{
  if(self != NULL)
    {
      online_regression_destroy(self->regression_vars[0]);
      online_regression_destroy(self->regression_vars[1]);
      online_regression_destroy(self->regression_vars[2]);
      
      free(self);
    }
  return (RegressionCalibrator*) NULL;
}

/*--------------------------------------------------------------------*/
void regression_calibrator_save(RegressionCalibrator* self, int base_address)
{

}

/*--------------------------------------------------------------------*/
void regression_calibrator_restore(RegressionCalibrator* self, int base_address)
{

}

/*--------------------------------------------------------------------*/
BOOL regression_calibrator_calibrated(RegressionCalibrator* self)
{
  return self->calibrated[0] && self->calibrated[1] && self->calibrated[2];
}

/*--------------------------------------------------------------------*/
void regression_calibrator_correct_measurement(RegressionCalibrator* self, Quaternion q)
{
  int i;
  ic_float_t m;
  
  for(i=0; i<3; i++)
    {
      q[i+1] -= online_regression_y_intercept(self->regression_vars[i]);
      m = online_regression_slope(self->regression_vars[i]);
      if(m != 0)
        q[i+1] /= m;
    }
};

/*--------------------------------------------------------------------*/
BOOL regression_calibrator_monitor_and_pick_samples(RegressionCalibrator* self, Quaternion uncorrected, Quaternion reference)
{
  int  i;
  BOOL sample_added = NO;
 
  for(i=0; i<3; i++)
    {
      ic_float_t magnitude = fabs(reference[i+1]);
      if((uncorrected != 0) && (magnitude > self->reference_min) && (magnitude < self->reference_max) && ((reference[i+1]>0)==(self->sign[i]>0)))
        {
          //alternately pick positive and negative samples
          self->sign[i] *= -1;
          online_regression_update(self->regression_vars[i], reference[i+1], uncorrected[i+1]);
          self->calibrated[i] = online_regression_n(self->regression_vars[i]) >= self->maxnum_samples;
          sample_added = YES;
        }
    }
  return sample_added;
}

/*--------------------------------------------------------------------*/
/*__------__-----------___------_-_-_-------------_---------------------
  \ \    / /__ _ _    / __|__ _| (_) |__ _ _ __ _| |_ ___ _ _ 
   \ \/\/ / _ \ ' \  | (__/ _` | | | '_ \ '_/ _` |  _/ _ \ '_|
    \_/\_/\___/_||_|  \___\__,_|_|_|_.__/_| \__,_|\__\___/_|  
------------------------------------------------------------------------
// Seong-hoon Peter Won and Farid Golnaraghi
// A Triaxial Accelerometer Calibration Method Using a Mathematical Model
// IEEE TRANSACTIONS ON INSTRUMENTATION AND MEASUREMENT
// VOL. 59, NO. 8, AUGUST 2010 pp 2144 - 2153
----------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/* the algorithm iterates until the error falls below this threshold */
#define WON_CALIBRATOR_MAX_ERROR 0.0000000001

/*--------------------------------------------------------------------*/
struct opaque_won_calibrator_struct
{
  //the final calibration parameters (Bias xyz and Gain xyz)
  // 1 / Gain is stored for computational efficiency
  Quaternion B;
  Quaternion G;
  Quaternion G_inverse;
  ic_float_t epsilon;
  
  unsigned   num_collected_samples;
  Quaternion samples[6];
  BOOL       reference_vector_has_sample[6];
  BOOL       calibrated;
  
  /* used only by won_calibrator_calculate_calibration_parameters */
  Matrix*    Cal;
  Matrix*    Error;
  Matrix*    Accel;
  Matrix*    A_inv;
  Matrix*    A_temp;
};

/*--------------------------------------------------------------------*/
unsigned won_calibrator_index_of_nearest_reference_vector(WonCalibrator* self, Quaternion q);
BOOL     won_calibrator_calculate_calibration_parameters (WonCalibrator* self);

/*--------------------------------------------------------------------*/
WonCalibrator* won_calibrator_new()
{
  WonCalibrator* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->Cal     = matrix_new(6, 1);
      self->Error   = matrix_new(6, 1);
      self->Accel   = matrix_new(6, 6);
      self->A_inv   = matrix_new(6, 6);
      self->A_temp  = matrix_new(6, 6);
      
      if((self->Cal    == NULL) ||
         (self->Error  == NULL) ||
         (self->Accel  == NULL) ||
         (self->A_inv  == NULL) ||
         (self->A_temp == NULL)  )
        return won_calibrator_destroy(self);
        
      won_calibrator_init(self);
    }
  return self;  
}

/*--------------------------------------------------------------------*/
void won_calibrator_init(WonCalibrator* self)
{
  int i;
  
  quaternion_zero(self->B);
  quaternion_set (self->G, 0, 1, 1, 1);
  quaternion_set (self->G_inverse, 0, 1, 1, 1);
  self->epsilon = -1;
  self->num_collected_samples = 0;
  self->calibrated = NO; //DISABLE THIS FOR NOW BY MAKING THIS YES
  matrix_zero(self->Cal);
  matrix_zero(self->Error);
  matrix_zero(self->Accel);
  matrix_zero(self->A_inv);
  matrix_zero(self->A_temp);
  
  for(i=0; i<6; i++)
    {
      self->reference_vector_has_sample[i] = NO;
      quaternion_zero(self->samples[i]);
    }
}

/*--------------------------------------------------------------------*/
WonCalibrator* won_calibrator_destroy(WonCalibrator* self)
{
  if(self != NULL)
    { 
      matrix_destroy(self->Cal);
      matrix_destroy(self->Error);
      matrix_destroy(self->Accel);
      matrix_destroy(self->A_inv);
      matrix_destroy(self->A_temp);
      free(self);
    }
  return (WonCalibrator*) NULL;
}

/*--------------------------------------------------------------------*/
void won_calibrator_save(WonCalibrator* self, int base_address)
{
  char calibrated = self->calibrated;
  mmap_write(base_address+0 , 1, &calibrated);
  mmap_write(base_address+1 , 16, self->B);
  mmap_write(base_address+17, 16, self->G_inverse);
}

/*--------------------------------------------------------------------*/
void won_calibrator_restore(WonCalibrator* self, int base_address)
{
  char calibrated;
  mmap_read(base_address+0 , 1, &calibrated);
  mmap_read(base_address+1 , 16, self->B);
  mmap_read(base_address+17, 16, self->G_inverse);
  self->calibrated = calibrated;
}

/*--------------------------------------------------------------------*/
BOOL won_calibrator_calibrated(WonCalibrator* self)
{
  return self->calibrated;
}

/*--------------------------------------------------------------------*/
void won_calibrator_correct_measurement(WonCalibrator* self, Quaternion q)
{
  //equation 5;
  quaternion_subtract(q, self->B, q);
  quaternion_multiply_pointwise(q, self->G_inverse, q);
}

/*--------------------------------------------------------------------*/
//this is only called when the ball is not rotating for accelerometer
BOOL won_calibrator_monitor_and_pick_samples(WonCalibrator* self, Quaternion uncorrected)
{
  BOOL     did_add_sample = NO;
  
  if(!self->calibrated)
    {
      unsigned reference_vector = won_calibrator_index_of_nearest_reference_vector(self, uncorrected);
     
      if(!self->reference_vector_has_sample[reference_vector])
        {
          self->reference_vector_has_sample[reference_vector] = YES;
          quaternion_copy(uncorrected, self->samples[reference_vector]);
          ++self->num_collected_samples;
          did_add_sample = YES;
     
          if(self->num_collected_samples == 6)
            {
              self->calibrated = won_calibrator_calculate_calibration_parameters(self);
              if(!self->calibrated)
                won_calibrator_init(self);
            }
        }
    }
  return did_add_sample;
}

/*--------------------------------------------------------------------*/
//returns reference vector index
//i.e. 0, 1, or 2 according to whether q is closest to positive x, y, or z axis,
//or 3, 4, or 5 for negative x, y, or z, respectively.
unsigned won_calibrator_index_of_nearest_reference_vector(WonCalibrator* self, Quaternion q)
{
  unsigned index;
  
  ic_float_t x = fabs(q[1]);
  ic_float_t y = fabs(q[2]);
  ic_float_t z = fabs(q[3]);
  index = (x > y) ? ((x > z) ? 0 : 2) : ((y > z) ? 1 : 2);
  if(q[index+1] < 0) index += 3;

  return index;
}

/*--------------------------------------------------------------------*/
BOOL won_calibrator_calculate_calibration_parameters(WonCalibrator* self)
{
  BOOL       success = NO;
  int        i, iteration;
  Quaternion G_tilde;
  Quaternion B_tilde;
  Quaternion temp_q;
  ic_float_t epsilon;
  ic_float_t temp;
  
  //the paper finds 3 iterations to be plenty
  //this loop will break if it converges before 6
  for(iteration=0; iteration<6; iteration++)
    {
      for(i=0; i<6; i++)
        {
          //equation 5
          quaternion_copy(self->samples[i], temp_q);
          won_calibrator_correct_measurement(self, temp_q);

          //equation 10, second term
          matrix_set_value(self->Accel, i, 0, temp_q[1]*temp_q[1]);
          matrix_set_value(self->Accel, i, 1, temp_q[2]*temp_q[2]);
          matrix_set_value(self->Accel, i, 2, temp_q[3]*temp_q[3]);
          matrix_set_value(self->Accel, i, 3, temp_q[1]);
          matrix_set_value(self->Accel, i, 4, temp_q[2]);
          matrix_set_value(self->Accel, i, 5, temp_q[3]);
          
          //equation 6
          temp = matrix_get_value(self->Accel, i, 0, NULL) + 
                 matrix_get_value(self->Accel, i, 1, NULL) + 
                 matrix_get_value(self->Accel, i, 2, NULL) - 1;
          matrix_set_value(self->Error, i, 0, temp);
        }
   
      //equation 11
      if(!matrix_invert(self->Accel, self->A_temp, self->A_inv)) break;
      matrix_multiply(self->A_inv, self->Error, self->Cal);
    
      //todo: can Cal[0][x] be 1 -- that would result in divide by 0
      G_tilde[1] = 1.0 / sqrt(fabs(1 - matrix_get_value(self->Cal, 0, 0, NULL)));
      G_tilde[2] = 1.0 / sqrt(fabs(1 - matrix_get_value(self->Cal, 1, 0, NULL)));
      G_tilde[3] = 1.0 / sqrt(fabs(1 - matrix_get_value(self->Cal, 2, 0, NULL)));
      
      //equation 10, third term
      B_tilde[1] = 0.5 * matrix_get_value(self->Cal, 3, 0, NULL) * self->G[1] * G_tilde[1] * G_tilde[1];
      B_tilde[2] = 0.5 * matrix_get_value(self->Cal, 4, 0, NULL) * self->G[2] * G_tilde[2] * G_tilde[2];
      B_tilde[3] = 0.5 * matrix_get_value(self->Cal, 5, 0, NULL) * self->G[3] * G_tilde[3] * G_tilde[3];
       
      //equation 4
      quaternion_multiply_pointwise(self->G, G_tilde, self->G);
      quaternion_add(self->B, B_tilde, self->B);
      
      //store inverse of gain for computatoinal efficiency
      if((self->G[1] <= 0) || (self->G[2] <= 0) || (self->G[3] <= 0))
        break;
      self->G_inverse[1] = 1.0 / self->G[1];
      self->G_inverse[2] = 1.0 / self->G[2];
      self->G_inverse[3] = 1.0 / self->G[3];
      
      quaternion_multiply_pointwise(self->G, self->G, G_tilde);
      quaternion_multiply_pointwise(B_tilde, B_tilde, B_tilde);
      epsilon  = B_tilde[1] / G_tilde[1];
      epsilon += B_tilde[2] / G_tilde[2];
      epsilon += B_tilde[3] / G_tilde[3];
      
      if(epsilon < WON_CALIBRATOR_MAX_ERROR)
        {
          self->epsilon = epsilon;
          success = YES;
          break;
        }
    }
    
  return success;
}
