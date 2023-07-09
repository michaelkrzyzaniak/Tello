/*-____--_--------_---_-----_---_------------_--------------------------
  / ___|| |_ __ _| |_(_)___| |_(_) ___ ___  | |__  
  \___ \| __/ _` | __| / __| __| |/ __/ __| | '_ \ 
   ___) | || (_| | |_| \__ \ |_| | (__\__ \_| | | |
  |____/ \__\__,_|\__|_|___/\__|_|\___|___(_)_| |_|
------------------------------------------------------------------------
Created by Michael Krzyzaniak at Inside Coach
Copyright Inside Coach, 2017. All rights reserved
----------------------------------------------------------------------*/
#ifndef __IC_STATISTICS__
#define __IC_STATISTICS__
  
#if defined(__cplusplus)
extern "C"{
#endif   //(__cplusplus)

#include "Types.h"

/*--------------------------------------------------------------------*/
/*--___-------_-_--------------_----------------------------------------
   / _ \ _ _ | (_)_ _  ___    /_\__ _____ _ _ __ _ __ _ ___ 
  | (_) | ' \| | | ' \/ -_)  / _ \ V / -_) '_/ _` / _` / -_);
   \___/|_||_|_|_|_||_\___| /_/ \_\_/\___|_| \__,_\__, \___|
--------------------------------------------------|___/-----------------
  Calculate the mean and variance of a signal one sample at a time.
----------------------------------------------------------------------*/
typedef struct     opaque_online_average_struct     OnlineAverage;
OnlineAverage*     online_average_new               ();
void               online_average_init              (OnlineAverage* self);
OnlineAverage*     online_average_destroy           (OnlineAverage* self);
int                online_average_n                 (OnlineAverage* self);
ic_float_t         online_average_mean              (OnlineAverage* self);
ic_float_t         online_average_variance          (OnlineAverage* self);
ic_float_t         online_average_std_dev           (OnlineAverage* self);
void               online_average_update            (OnlineAverage* self, ic_float_t x);

/*--------------------------------------------------------------------*/
/*-__--__---------_---------------_-------------------------------------
  |  \/  |_____ _(_)_ _  __ _    /_\__ _____ _ _ __ _ __ _ ___ 
  | |\/| / _ \ V / | ' \/ _` |  / _ \ V / -_) '_/ _` / _` / -_);
  |_|  |_\___/\_/|_|_||_\__, | /_/ \_\_/\___|_| \__,_\__, \___|
------------------------|___/------------------------|___/--------------
  Calculate the moving or rolling mean and variance over the past N
  samples of a signal. This algorithm is online and updates one sample
  at a time.
----------------------------------------------------------------------*/
typedef struct     opaque_moving_average_struct     MovingAverage;
MovingAverage*     moving_average_new               (unsigned N);
void               moving_average_init              (MovingAverage* self);
MovingAverage*     moving_average_destroy           (MovingAverage* self);
int                moving_average_N                 (MovingAverage* self);
int                moving_average_n                 (MovingAverage* self);
ic_float_t         moving_average_mean              (MovingAverage* self);
ic_float_t         moving_average_variance          (MovingAverage* self);
ic_float_t         moving_average_std_dev           (MovingAverage* self);
void               moving_average_update            (MovingAverage* self, ic_float_t x);

/*--------------------------------------------------------------------*/
/*--___-------_-_------------___------------------------_---------------
   / _ \ _ _ | (_)_ _  ___  | _ \___ __ _ _ _ ___ _____(_)___ _ _  
  | (_) | ' \| | | ' \/ -_) |   / -_) _` | '_/ -_|_-<_-< / _ \ ' \ 
   \___/|_||_|_|_|_||_\___| |_|_\___\__, |_| \___/__/__/_\___/_||_|
------------------------------------|___/-------------------------------
  Calculate the means, variances, covariance, slope, y-intercept and 
  correlation coeficient of two signals. This is an online algorithm
  that operates one sample at a time.
----------------------------------------------------------------------*/
typedef struct     opaque_online_regression_struct  OnlineRegression;
OnlineRegression*  online_regression_new            ();
void               online_regression_init           (OnlineRegression* self);
OnlineRegression*  online_regression_destroy        (OnlineRegression* self);
int                online_regression_n              (OnlineRegression* self);
ic_float_t         online_regression_covariance     (OnlineRegression* self);
ic_float_t         online_regression_slope          (OnlineRegression* self);
ic_float_t         online_regression_y_intercept    (OnlineRegression* self);
ic_float_t         online_regression_r_squared      (OnlineRegression* self);
void               online_regression_update         (OnlineRegression* self, ic_float_t a_data, ic_float_t b_data);

/*--------------------------------------------------------------------*/
/*----------_-----------_---_-----------_____-_---------------_----_----    _    _ 
    /_\  __| |__ _ _ __| |_(_)_ _____  |_   _| |_  _ _ ___ __| |_ | |_  ___| |__| |
   / _ \/ _` / _` | '_ \  _| \ V / -_)   | | | ' \| '_/ -_|_-< ' \| ' \/ _ \ / _` |
  /_/ \_\__,_\__,_| .__/\__|_|\_/\___|   |_| |_||_|_| \___/__/_||_|_||_\___/_\__,_|
------------------|_|---------------------------------------------------
  Adaptive threshhold tells you when your signal goes a specified number
  of standard deviations above or below a moving average of a filtered
  version of your signal. update() returns +1 or -1 whenever your signal
  transitions to being above or below the threshold.
----------------------------------------------------------------------*/
typedef struct     opaque_adaptive_threshold_struct AdaptiveThreshold;
AdaptiveThreshold* adaptive_threshold_new           (unsigned N);
void               adaptive_threshold_init          (AdaptiveThreshold* self);
void               adaptive_threshold_clear         (AdaptiveThreshold* self);
AdaptiveThreshold* adaptive_threshold_destroy       (AdaptiveThreshold* self);
ic_float_t         adaptive_threshold_smoothing     (AdaptiveThreshold* self);
void               adaptive_threshold_set_smoothing (AdaptiveThreshold* self, ic_float_t coefficient);
ic_float_t         adaptive_threshold_threshold     (AdaptiveThreshold* self);
void               adaptive_threshold_set_threshold (AdaptiveThreshold* self, ic_float_t std_devs);
ic_float_t         adaptive_threshold_onset_signal  (AdaptiveThreshold* self);
ic_float_t         adaptive_threshold_update        (AdaptiveThreshold* self, ic_float_t x);

ic_float_t         random_0_1();
ic_float_t         random_normal(ic_float_t mean, ic_float_t std_dev);


#if defined(__cplusplus)
}
#endif   //(__cplusplus)

#endif   //__IC_STATISTICS__

