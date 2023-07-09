/*--___-------_------------_--------_---_---------------_---------------
   / _ \ _ __(_) ___ _ __ | |_ __ _| |_(_) ___  _ __   | |__  
  | | | | '__| |/ _ \ '_ \| __/ _` | __| |/ _ \| '_ \  | '_ \ 
  | |_| | |  | |  __/ | | | || (_| | |_| | (_) | | | |_| | | |
   \___/|_|  |_|\___|_| |_|\__\__,_|\__|_|\___/|_| |_(_)_| |_|
------------------------------------------------------------------------
Created by Michael Krzyzaniak at Inside Coach
Copyright Inside Coach, 2017. All rights reserved
----------------------------------------------------------------------*/
#ifndef __IC_ORIENTATION__
#define __IC_ORIENTATION__
  
#if defined(__cplusplus)
extern "C"{
#endif   //(__cplusplus)

#include "Types.h"
#include "Algebra.h"

/*--------------------------------------------------------------------*/
/*-__--__---------_-------------_------------_---_--_-___--___----------
  |  \/  |__ _ __| |__ ___ __ _(_)__| |__   /_\ | || | _ \/ __|
  | |\/| / _` / _` / _` \ V  V / / _| / /  / _ \| __ |   /\__ \
  |_|  |_\__,_\__,_\__, |\_/\_/|_\__|_\_\ /_/ \_\_||_|_|_\|___/
                   |___/                                       
  Sebastian O.H. Madgwick
  An efficient orientation filter for inertial and inertial/magnetic sensor arra[2]s
  April 30, 2010
  Internal Report
  
  Implementation of Madgwick's IMU and AHRS algorithms.
  See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
  Date      Author          Notes
  29/09/2011  SOH Madgwick  Initial release
  02/10/2011  SOH Madgwick  Optimised for reduced CPU load
  19/02/2012  SOH Madgwick  Magnetometer measurement is normalised
  22/10/2017  M Krzyzaniak  Ignore large accelerations, get initial conditions
----------------------------------------------------------------------*/
typedef struct opaque_madgwick_ahrs_struct MadgwickAHRS;
MadgwickAHRS*  madgwick_ahrs_new           ();
void           madgwick_ahrs_init          (MadgwickAHRS* self);
void           madgwick_ahrs_clear         (MadgwickAHRS* self);
MadgwickAHRS*  madgwick_ahrs_destroy       (MadgwickAHRS* self);
void           madgwick_ahrs_update_marg   (MadgwickAHRS* self, ic_float_t sample_interval, Quaternion g, Quaternion a, Quaternion m);
void           madgwick_ahrs_update_imu    (MadgwickAHRS* self, ic_float_t sample_interval, Quaternion g, Quaternion a);
void           madgwick_ahrs_get_orientation(MadgwickAHRS* self, Quaternion copied_result);

#if defined(__cplusplus)
}
#endif   //(__cplusplus)

#endif   //__IC_ORIENTATION__
