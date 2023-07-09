/*--____------_-_-_---------------_---_---------------_------------------
   / ___|__ _| (_) |__  _ __ __ _| |_(_) ___  _ __   | |__  
  | |   / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \  | '_ \ 
  | |__| (_| | | | |_) | | | (_| | |_| | (_) | | | |_| | | |
   \____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_(_)_| |_|
------------------------------------------------------------------------
Created by Michael Krzyzaniak and Sergiu Baluta at Inside Coach
Copyright Inside Coach, 2017. All rights reserved.
----------------------------------------------------------------------*/
#ifndef __IC_CALIBRATION__
#define __IC_CALIBRATION__
  
#if defined(__cplusplus)
extern "C"{
#endif   //(__cplusplus)

#include "Types.h"
#include "Algebra.h"

#include <stdint.h>

/*--------------------------------------------------------------------*/
typedef struct        opaque_calibration_struct                     Calibration;
Calibration*          calibration_new                               ();
void                  calibration_init                              (Calibration* self);
Calibration*          calibration_destroy                           (Calibration* self);
void                  calibration_save                              (Calibration* self);
void                  calibration_restore                           (Calibration* self);
BOOL                  calibration_calibrated                        (Calibration* self);
uint8_t               calibration_calibrated_mask                   (Calibration* self);
BOOL                  calibration_update                            (Calibration* self, Quaternion gyro, Quaternion acc, Quaternion mag);

/*--------------------------------------------------------------------*/
/*--___---__--__---------_------___------_-_-_-------------_------------
   / _ \ / _|/ _|___ ___| |_   / __|__ _| (_) |__ _ _ __ _| |_ ___ _ _ 
  | (_) |  _|  _(_-</ -_)  _| | (__/ _` | | | '_ \ '_/ _` |  _/ _ \ '_|
   \___/|_| |_| /__/\___|\__|  \___\__,_|_|_|_.__/_| \__,_|\__\___/_| 
------------------------------------------------------------------------
  find the average of the signal and subtract it out of subsequent 
  samples. Used for gyro calibration.
----------------------------------------------------------------------*/
typedef struct        opaque_offset_calibrator_struct               OffsetCalibrator;
OffsetCalibrator*     offset_calibrator_new                         (unsigned maxnum_samples);
void                  offset_calibrator_init                        (OffsetCalibrator* self);
OffsetCalibrator*     offset_calibrator_destroy                     (OffsetCalibrator* self);
void                  offset_calibrator_save                        (OffsetCalibrator* self, int base_address);
void                  offset_calibrator_restore                     (OffsetCalibrator* self, int base_address);
BOOL                  offset_calibrator_calibrated                  (OffsetCalibrator* self);
BOOL                  offset_calibrator_add_sample                  (OffsetCalibrator* self, Quaternion uncorrected);
void                  offset_calibrator_correct_measurement         (OffsetCalibrator* self, Quaternion q);

/*--------------------------------------------------------------------*/
/*-___-------_------_-----------___------_-_-_-------------_------------
  | _ ) __ _| |_  _| |_ __ _   / __|__ _| (_) |__ _ _ __ _| |_ ___ _ _ 
  | _ \/ _` | | || |  _/ _` | | (__/ _` | | | '_ \ '_/ _` |  _/ _ \ '_|
  |___/\__,_|_|\_,_|\__\__,_|  \___\__,_|_|_|_.__/_| \__,_|\__\___/_| 
------------------------------------------------------------------------
  Used for magnetometer calibration.
----------------------------------------------------------------------*/
typedef struct        opaque_baluta_calibrator_struct               BalutaCalibrator;
BalutaCalibrator*     baluta_calibrator_new                         (unsigned samples_per_reference_vector);
void                  baluta_calibrator_init                        (BalutaCalibrator* self);
BalutaCalibrator*     baluta_calibrator_destroy                     (BalutaCalibrator* self);
void                  baluta_calibrator_save                        (BalutaCalibrator* self, int base_address);
void                  baluta_calibrator_restore                     (BalutaCalibrator* self, int base_address);
BOOL                  baluta_calibrator_calibrated                  (BalutaCalibrator* self);
BOOL                  baluta_calibrator_monitor_and_pick_samples    (BalutaCalibrator* self, Quaternion uncorrected);
void                  baluta_calibrator_correct_measurement         (BalutaCalibrator* self, Quaternion q);

/*--------------------------------------------------------------------*/
/*-___------------------------_-------------___------_-_---------------_       
  | _ \___ __ _ _ _ ___ _____(_)___ _ _    / __|__ _| (_) |__ _ _ __ _| |_ ___ _ _ 
  |   / -_) _` | '_/ -_|_-<_-< / _ \ ' \  | (__/ _` | | | '_ \ '_/ _` |  _/ _ \ '_|
  |_|_\___\__, |_| \___/__/__/_\___/_||_|  \___\__,_|_|_|_.__/_| \__,_|\__\___/_|  
----------|___/---------------------------------------------------------
  this module transfers calibration from one sensor to another 
  via linear regression. It is used for high-g and accel-w gyro
----------------------------------------------------------------------*/
typedef struct        opaque_regression_calibrator_struct           RegressionCalibrator;
RegressionCalibrator* regression_calibrator_new                     (ic_float_t reference_min, ic_float_t reference_max, int maxnum_samples);
void                  regression_calibrator_init                    (RegressionCalibrator* self);
RegressionCalibrator* regression_calibrator_destroy                 (RegressionCalibrator* self);
void                  regression_calibrator_save                    (RegressionCalibrator* self, int base_address);
void                  regression_calibrator_restore                 (RegressionCalibrator* self, int base_address);
BOOL                  regression_calibrator_calibrated              (RegressionCalibrator* self);
BOOL                  regression_calibrator_monitor_and_pick_samples(RegressionCalibrator* self, Quaternion uncorrected, Quaternion reference);
void                  regression_calibrator_correct_measurement     (RegressionCalibrator* self, Quaternion q);

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
typedef struct        opaque_won_calibrator_struct                  WonCalibrator;
WonCalibrator*        won_calibrator_new                            ();
void                  won_calibrator_init                           (WonCalibrator* self);
WonCalibrator*        won_calibrator_destroy                        (WonCalibrator* self);
void                  won_calibrator_save                           (WonCalibrator* self, int base_address);
void                  won_calibrator_restore                        (WonCalibrator* self, int base_address);
BOOL                  won_calibrator_calibrated                     (WonCalibrator* self);
BOOL                  won_calibrator_monitor_and_pick_samples       (WonCalibrator* self, Quaternion uncorrected);
void                  won_calibrator_correct_measurement            (WonCalibrator* self, Quaternion q);

#if defined(__cplusplus)
}
#endif   //(__cplusplus)

#endif   //__IC_CALIBRATION__
