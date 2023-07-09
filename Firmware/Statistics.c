/*-____--_--------_---_-----_---_--------------------------------------- 
  / ___|| |_ __ _| |_(_)___| |_(_) ___ ___   ___ 
  \___ \| __/ _` | __| / __| __| |/ __/ __| / __|
   ___) | || (_| | |_| \__ \ |_| | (__\__ \| (__ 
  |____/ \__\__,_|\__|_|___/\__|_|\___|___(_)___|
------------------------------------------------------------------------
Created by Michael Krzyzaniak at Inside Coach
Copyright Inside Coach, 2017. All rights reserved
----------------------------------------------------------------------*/
#include "Statistics.h"

/*--------------------------------------------------------------------*/
/*--___-------_-_-------------------------------------------------------
   / _ \ _ _ | (_)_ _  ___    /_\__ _____ _ _ __ _ __ _ ___ 
  | (_) | ' \| | | ' \/ -_)  / _ \ V / -_) '_/ _` / _` / -_)
   \___/|_||_|_|_|_||_\___| /_/ \_\_/\___|_| \__,_\__, \___|
--------------------------------------------------|___/-----------------
  Calculate the mean and variance of a signal one sample at a time.
----------------------------------------------------------------------*/
struct opaque_online_average_struct
{
  unsigned     n;
  ic_float_t   mean;
  ic_float_t   variance;
  ic_float_t   old_mean;
  ic_float_t   s;
  ic_float_t   old_s;
};

/*--------------------------------------------------------------------*/
OnlineAverage* online_average_new()
{
  OnlineAverage* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      online_average_init(self);
    }
  return self;
}

/*--------------------------------------------------------------------*/
void online_average_init(OnlineAverage* self)
{
  self->n          = 0;
  self->mean       = 0;
  self->variance   = 0;
  self->old_mean   = 0;
  self->s          = 0;
  self->old_s      = 0;
}

/*--------------------------------------------------------------------*/
OnlineAverage* online_average_destroy(OnlineAverage* self)
{
  if(self != NULL)
    {
      free(self);
    }
  return (OnlineAverage*) NULL;
}

/*--------------------------------------------------------------------*/
int online_average_n(OnlineAverage* self)
{
  return self->n;
}

/*--------------------------------------------------------------------*/
ic_float_t online_average_mean(OnlineAverage* self)
{
  return self->mean;
}

/*--------------------------------------------------------------------*/
ic_float_t online_average_variance(OnlineAverage* self)
{
  return self->variance;
}

/*--------------------------------------------------------------------*/
ic_float_t online_average_std_dev(OnlineAverage* self)
{
  return sqrt(self->variance);
}

/*--------------------------------------------------------------------*/
void online_average_update(OnlineAverage* self, ic_float_t x)
{
  ++self->n;
  
  if (self->n == 1)
    {
      self->old_mean = self->mean = x;
      self->old_s    = 0.0;
    }
  else
    {
      self->mean     = self->old_mean + (x - self->old_mean) / self->n;
      self->s        = self->old_s + ((x - self->old_mean) * (x - self->mean));
      self->old_mean = self->mean; 
      self->old_s    = self->s;
      self->variance = self->s / (self->n - 1);
    }
}

/*--------------------------------------------------------------------*/
/*-__--__---------_---------------_-------------------------------------
  |  \/  |_____ _(_)_ _  __ _    /_\__ _____ _ _ __ _ __ _ ___ 
  | |\/| / _ \ V / | ' \/ _` |  / _ \ V / -_) '_/ _` / _` / -_)
  |_|  |_\___/\_/|_|_||_\__, | /_/ \_\_/\___|_| \__,_\__, \___|
------------------------|___/------------------------|___/--------------
  Calculate the moving or rolling mean and variance over the past N
  samples of a signal. This algorithm is online and updates one sample
  at a time.
----------------------------------------------------------------------*/
struct opaque_moving_average_struct
{
  unsigned       N;  // the nominal window size
  ic_float_t     mean;
  ic_float_t     variance;
  ic_float_t*    past_samples;
  unsigned       current_sample_index;
  ic_float_t     s;
  OnlineAverage* online;  //used until there are at least self->N samples
};

/*--------------------------------------------------------------------*/
MovingAverage* moving_average_new(unsigned N)
{
  MovingAverage* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->N = N;
      self->past_samples = calloc(self->N, sizeof(*(self->past_samples)));
      if(self->past_samples == NULL)
        return moving_average_destroy(self);
      
      self->online = online_average_new();
      if(self->online == NULL)
        return moving_average_destroy(self);
      
      moving_average_init(self);
    }
  return self;
}

/*--------------------------------------------------------------------*/
void moving_average_init(MovingAverage* self)
{
  self->mean                 = 0;
  self->variance             = 0;
  self->current_sample_index = 0;
  self->s                    = 0;
  online_average_init(self->online);  
}

/*--------------------------------------------------------------------*/
MovingAverage* moving_average_destroy(MovingAverage* self)
{
  if(self != NULL)
    {
      if(self->past_samples != NULL)
        free(self->past_samples);
        
      self->online = online_average_destroy(self->online);
      
      free(self);
    }
  return (MovingAverage*) NULL;
}

/*--------------------------------------------------------------------*/
int moving_average_N(MovingAverage* self)
{
  return self->N;
}

/*--------------------------------------------------------------------*/
int moving_average_n(MovingAverage* self)
{
  return online_average_n(self->online);
}

/*--------------------------------------------------------------------*/
ic_float_t moving_average_mean(MovingAverage* self)
{
  return self->mean;
}

/*--------------------------------------------------------------------*/
ic_float_t moving_average_variance(MovingAverage* self)
{
  return self->variance;
}

/*--------------------------------------------------------------------*/
ic_float_t moving_average_std_dev(MovingAverage* self)
{
  return sqrt(self->variance);
}
/*--------------------------------------------------------------------*/
void moving_average_update(MovingAverage* self, ic_float_t x)
{
  ic_float_t new_mean;
  ic_float_t oldest_sample = self->past_samples[self->current_sample_index];
  
  self->past_samples[self->current_sample_index] = x;
  ++self->current_sample_index;
  self->current_sample_index %= self->N;

  if(online_average_n(self->online) < self->N)
    {
      online_average_update(self->online, x);
      self->variance = online_average_variance(self->online);
      self->mean     = online_average_mean(self->online);
      self->s        = self->variance * self->N;
    }
  else
    {
      new_mean       = self->mean + (x - oldest_sample) / self->N;
      self->s       += (x + oldest_sample - self->mean - new_mean) * (x - oldest_sample);
      self->variance = self->s / self->N;
      self->mean     = new_mean;
    }
};

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
struct opaque_online_regression_struct
{
  unsigned     n         ;
  ic_float_t   a_mean    ;
  ic_float_t   b_mean    ;
  ic_float_t   a_variance;
  ic_float_t   b_variance;
  ic_float_t   covariance;
  
  //slope, y-intercept, correlation
  ic_float_t   m         ;
  ic_float_t   b         ;
  ic_float_t   r_2       ;
};

/*--------------------------------------------------------------------*/
OnlineRegression* online_regression_new()
{
  OnlineRegression* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      online_regression_init(self);
    }
  return self;
}

/*--------------------------------------------------------------------*/
void online_regression_init(OnlineRegression* self)
{
  self->n          = 0;
  self->a_mean     = 0;
  self->b_mean     = 0;
  self->a_variance = 0;
  self->b_variance = 0;
  self->covariance = 0;
  self->m          = 1;
  self->b          = 0;
  self->r_2        = 0;
}

/*--------------------------------------------------------------------*/
OnlineRegression* online_regression_destroy(OnlineRegression* self)
{
  if(self != NULL)
    {
      free(self);
    }
  return (OnlineRegression*) NULL;
}

/*--------------------------------------------------------------------*/
int online_regression_n(OnlineRegression* self)
{
  return self->n;
}

/*--------------------------------------------------------------------*/
ic_float_t online_regression_covariance(OnlineRegression* self)
{
  return self->covariance;
}

/*--------------------------------------------------------------------*/
ic_float_t online_regression_slope(OnlineRegression* self)
{
  return self->m;
}

/*--------------------------------------------------------------------*/
ic_float_t online_regression_y_intercept(OnlineRegression* self)
{
  return self->b;
}

/*--------------------------------------------------------------------*/
ic_float_t online_regression_r_squared(OnlineRegression* self)
{
  return self->r_2;
}

/*--------------------------------------------------------------------*/
void online_regression_update(OnlineRegression* self, ic_float_t a_data, ic_float_t b_data)
{
  ++self->n;
  ic_float_t one_over_n = 1 / (double)self->n;
  
  ic_float_t n_minus_1_over_n = (self->n-1) * one_over_n;
  ic_float_t d_a = a_data - self->a_mean;
  ic_float_t d_b = b_data - self->b_mean;
  
  ic_float_t new_m, new_b;
  self->a_variance += (n_minus_1_over_n * d_a * d_a - self->a_variance) * one_over_n;
  self->b_variance += (n_minus_1_over_n * d_b * d_b - self->b_variance) * one_over_n;
  self->covariance += (n_minus_1_over_n * d_a * d_b - self->covariance) * one_over_n;
  
  self->a_mean += d_a * one_over_n;
  self->b_mean += d_b * one_over_n;
  
  if((self->a_variance == 0) || (self->covariance == 0))
    return;
  
  new_m = self->covariance / self->a_variance;
  new_b = self->b_mean - (new_m * self->a_mean);
  self->m = 1/new_m;
  self->b = new_b;
  self->r_2 = (self->covariance * self->covariance) / (self->a_variance * self->b_variance);
}

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
struct opaque_adaptive_threshold_struct
{
  ic_float_t     smoothing; //coefficient 0 ~ 1
  ic_float_t     threshold; //standard deviations from filtered signal
  
  // step function (-1, 0, 1) that will hold its value while input is 
  // above threshold. update() only returns +1 or -1 at the moment
  // onset_signal transitions to +1 or -1;
  ic_float_t     onset_signal;
  
  //(start with high variance to avoid spurious onsets on first iterations)
  MovingAverage* avg;
  ic_float_t     filtered_x;
};

/*--------------------------------------------------------------------*/
AdaptiveThreshold* adaptive_threshold_new(unsigned N)
{
  AdaptiveThreshold* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->avg = moving_average_new(N);
      if(self->avg == NULL)
        return adaptive_threshold_destroy(self);

      adaptive_threshold_init(self);
    }
  return self;
}

/*--------------------------------------------------------------------*/
void adaptive_threshold_init(AdaptiveThreshold* self)
{
  self->smoothing  = 0.5; //coefficient 0 ~ 1
  self->threshold  = 3.5; //standard deviations from filtered signal
  adaptive_threshold_clear(self);
}

/*--------------------------------------------------------------------*/
void adaptive_threshold_clear(AdaptiveThreshold* self)
{
  self->onset_signal = 0;
  moving_average_init(self->avg);
  self->filtered_x   = 0;
}

/*--------------------------------------------------------------------*/
AdaptiveThreshold* adaptive_threshold_destroy(AdaptiveThreshold* self)
{
  if(self != NULL)
    {
      self->avg = moving_average_destroy(self->avg);
      free(self);
    }
  return (AdaptiveThreshold*) NULL;
}

/*--------------------------------------------------------------------*/
ic_float_t adaptive_threshold_smoothing(AdaptiveThreshold* self)
{
  return self->smoothing;
}

/*--------------------------------------------------------------------*/
void adaptive_threshold_set_smoothing(AdaptiveThreshold* self, ic_float_t coefficient)
{
  coefficient = (coefficient > 1) ? 1 : ((coefficient < 0) ? 0 : coefficient);
  self->smoothing = coefficient;
}

/*--------------------------------------------------------------------*/
ic_float_t adaptive_threshold_threshold(AdaptiveThreshold* self)
{
  return self->threshold;
}

/*--------------------------------------------------------------------*/
void adaptive_threshold_set_threshold(AdaptiveThreshold* self, ic_float_t std_devs)
{
  self->threshold = std_devs;
}

/*--------------------------------------------------------------------*/
ic_float_t adaptive_threshold_onset_signal(AdaptiveThreshold* self)
{
  return self->onset_signal;
}

/*--------------------------------------------------------------------*/
//returns 1 or -1 to indicate onset signal transitioned past threshhold, else 0
ic_float_t adaptive_threshold_update(AdaptiveThreshold* self, ic_float_t x)
{
  ic_float_t next   = 0;
  ic_float_t filter = x;
  ic_float_t result = 0;

  self->filtered_x = (x * (1-self->smoothing)) + (self->filtered_x * self->smoothing);
  moving_average_update(self->avg, self->filtered_x);
  
  if((fabs(x) - moving_average_mean(self->avg)) > (moving_average_std_dev(self->avg) * self->threshold))
    next = (x > moving_average_mean(self->avg)) ? 1 : -1;
  
  //ignore the first few samples so we get a stable std dev
  if(moving_average_n(self->avg) > 10)
    if((next != 0) && (self->onset_signal != next))
      result = next;

  self->onset_signal = next;
  
  return result;
};


/*--------------------------------------------------------------------*/
ic_float_t random_0_1()
{
  return random() / (double)RAND_MAX;
}

/*--------------------------------------------------------------------*/
ic_float_t random_normal(ic_float_t mean, ic_float_t std_dev)
{
  ic_float_t u, v, r;
  
  do{
    u = 2 * random_0_1() - 1;
    v = 2 * random_0_1() - 1;
    r = u*u + v*v;
  }while(r == 0 || r > 1);

  ic_float_t c = sqrt(-2 * log(r) / r);
  return mean + u * c * std_dev;
}


