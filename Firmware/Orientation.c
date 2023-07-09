/*--___-------_------------_--------_---_---------------------------------
   / _ \ _ __(_) ___ _ __ | |_ __ _| |_(_) ___  _ __    ___ 
  | | | | '__| |/ _ \ '_ \| __/ _` | __| |/ _ \| '_ \  / __|
  | |_| | |  | |  __/ | | | || (_| | |_| | (_) | | | || (__ 
   \___/|_|  |_|\___|_| |_|\__\__,_|\__|_|\___/|_| |_(_)___|
------------------------------------------------------------------------
Created by Michael Krzyzaniak at Inside Coach
Copyright Inside Coach, 2017. All rights reserved
----------------------------------------------------------------------*/
#include "Orientation.h"

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
struct opaque_madgwick_ahrs_struct
{
  Quaternion  orientation;
  BOOL        first_time;
  ic_float_t  beta;
  ic_float_t  kGravity;
  ic_float_t  kAccelerationThreshold;
};

/*--------------------------------------------------------------------*/
void madgwick_ahrs_get_absolute_orientation(MadgwickAHRS* self, Quaternion a, Quaternion m, Quaternion result);

/*--------------------------------------------------------------------*/
MadgwickAHRS* madgwick_ahrs_new()
{
  MadgwickAHRS* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->kGravity = 1;
      madgwick_ahrs_init(self);
    }
  return self;
}

/*--------------------------------------------------------------------*/
void madgwick_ahrs_init(MadgwickAHRS* self)
{
  self->beta = 1.5;
  self->kAccelerationThreshold = 0.13 * self->kGravity;
  madgwick_ahrs_clear(self);
}

/*--------------------------------------------------------------------*/
void madgwick_ahrs_clear(MadgwickAHRS* self)
{
  quaternion_set(self->orientation, 1, 0, 0, 0);
  self->first_time  = YES;
}

/*--------------------------------------------------------------------*/
MadgwickAHRS* madgwick_ahrs_destroy(MadgwickAHRS* self)
{
  if(self != NULL)
    {
      free(self);
    }
  return (MadgwickAHRS*) NULL;
};

/*--------------------------------------------------------------------*/
void madgwick_ahrs_get_absolute_orientation(MadgwickAHRS* self, Quaternion a, Quaternion m, Quaternion result)
{
  Quaternion _m;
  Quaternion _a;
  
  quaternion_normalize(a, _a);
  
  if (_a[3] >=0)
    {
      _a[0] =  sqrt((_a[3] + 1) * 0.5);	
      _a[1] = -_a[2]/(2.0 * _a[0]);
      _a[2] =  _a[1]/(2.0 * _a[0]);
      _a[3] =  0;
    }
    else 
    {
      ic_float_t X = sqrt((1 - _a[3]) * 0.5);
      _a[0] = -_a[2]/(2.0 * X);
      _a[1] =  X;
      _a[2] =  0;
      _a[3] =  _a[1]/(2.0 * X);
    }  
  
  ic_float_t lx = (_a[0]*_a[0] + _a[1]*_a[1] - _a[2]*_a[2])*m[1] + 2.0 * (_a[1]*_a[2])*m[2] - 2.0 * (_a[0]*_a[2])*m[3];
  ic_float_t ly = 2.0 * (_a[1]*_a[2])*m[1] + (_a[0]*_a[0] - _a[1]*_a[1] + _a[2]*_a[2])*m[3] + 2.0 * (_a[0]*_a[1])*m[3];
  
	ic_float_t gamma = lx*lx + ly*ly;
	ic_float_t beta = sqrt(gamma + lx*sqrt(gamma));
  quaternion_set(_m, beta / (sqrt(2.0 * gamma)), 0, 0, ly / (sqrt(2.0) * beta));

  quaternion_multiply(_a, _m, result);
};

/*--------------------------------------------------------------------*/
void madgwick_ahrs_update_marg(MadgwickAHRS* self, ic_float_t sample_interval, Quaternion g, Quaternion a, Quaternion m)
{
  if(self->first_time)
    {
      if((m[1] == 0.0) && (m[2] == 0.0) && (m[3] == 0.0))
        return;
      else
        {
          madgwick_ahrs_get_absolute_orientation(self, a, g, self->orientation);
          quaternion_conjugate(self->orientation, self->orientation);
          self->first_time = NO;
          return;
        }
    }
 
  if(sample_interval <= 0)
    return;
    
  ic_float_t q0 = self->orientation[0];
  ic_float_t q1 = self->orientation[1];
  ic_float_t q2 = self->orientation[2];
  ic_float_t q3 = self->orientation[3];
  ic_float_t sample_freq = 1.0/sample_interval;

  ic_float_t recipNorm;
  ic_float_t s0, s1, s2, s3;
  ic_float_t qDot1, qDot2, qDot3, qDot4;
  ic_float_t hx, hy;
  ic_float_t _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, 
      _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((m[1] == 0.0) && (m[2] == 0.0) && (m[3] == 0.0)) {
    madgwick_ahrs_update_imu(self, sample_interval, g, a);
    return;
  }

  // Rate of change of quaternion from g[2]roscope
  qDot1 = 0.5 * (-q1 * g[1] - q2 * g[2] - q3 * g[3]);
  qDot2 = 0.5 * (q0 * g[1] + q2 * g[3] - q3 * g[2]);
  qDot3 = 0.5 * (q0 * g[2] - q1 * g[3] + q3 * g[1]);
  qDot4 = 0.5 * (q0 * g[3] + q1 * g[2] - q2 * g[1]);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((a[1] == 0.0) && (a[2] == 0.0) && (a[3] == 0.0))) 
    {
      //also, compute only if accel is not too large.
      ic_float_t acc_magnitude = sqrt(a[1]*a[1] + a[2]*a[2] + a[3]*a[3]);
      if (fabs(acc_magnitude - self->kGravity) <= self->kAccelerationThreshold)
        {
          // Normalise accelerometer measurement
          recipNorm = algebra_inverse_sqrt(a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
          a[1] *= recipNorm;
          a[2] *= recipNorm;
          a[3] *= recipNorm;   

          // Normalise magnetometer measurement
          recipNorm = algebra_inverse_sqrt(m[1] * m[1] + m[2] * m[2] + m[3] * m[3]);
          m[1] *= recipNorm;
          m[2] *= recipNorm;
          m[3] *= recipNorm;

          // Auxiliary ic_float_tiables to avoid repeated arithmetic
          _2q0mx = 2.0 * q0 * m[1];
          _2q0my = 2.0 * q0 * m[2];
          _2q0mz = 2.0 * q0 * m[3];
          _2q1mx = 2.0 * q1 * m[1];
          _2q0 = 2.0 * q0;
          _2q1 = 2.0 * q1;
          _2q2 = 2.0 * q2;
          _2q3 = 2.0 * q3;
          _2q0q2 = 2.0 * q0 * q2;
          _2q2q3 = 2.0 * q2 * q3;
          q0q0 = q0 * q0;
          q0q1 = q0 * q1;
          q0q2 = q0 * q2;
          q0q3 = q0 * q3;
          q1q1 = q1 * q1;
          q1q2 = q1 * q2;
          q1q3 = q1 * q3;
          q2q2 = q2 * q2;
          q2q3 = q2 * q3;
          q3q3 = q3 * q3;

          // Reference direction of Earth's magnetic field
          hx = m[1] * q0q0 - _2q0my * q3 + _2q0mz * q2 + m[1] * q1q1 + _2q1 * m[2] * q2 + _2q1 * m[3] * q3 - m[1] * q2q2 - m[1] * q3q3;
          hy = _2q0mx * q3 + m[2] * q0q0 - _2q0mz * q1 + _2q1mx * q2 - m[2] * q1q1 + m[2] * q2q2 + _2q2 * m[3] * q3 - m[2] * q3q3;
          _2bx = sqrt(hx * hx + hy * hy);
          _2bz = -_2q0mx * q2 + _2q0my * q1 + m[3] * q0q0 + _2q1mx * q3 - m[3] * q1q1 + _2q2 * m[2] * q3 - m[3] * q2q2 + m[3] * q3q3;
          _4bx = 2.0 * _2bx;
          _4bz = 2.0 * _2bz;

          // Gradient decent algorithm corrective step
          s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - a[1]) + _2q1 * (2.0 * q0q1 + _2q2q3 - a[2]) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[1]) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[2]) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m[3]);
          s1 = _2q3  * (2.0 * q1q3 - _2q0q2 - a[1]) + _2q0 * (2.0 * q0q1 + _2q2q3 - a[2]) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - a[3]) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[1]) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[2]) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m[3]);
          s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - a[1]) + _2q3 * (2.0 * q0q1 + _2q2q3 - a[2]) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - a[3]) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[1]) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[2]) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m[3]);
          s3 = _2q1  * (2.0 * q1q3 - _2q0q2 - a[1]) + _2q2 * (2.0 * q0q1 + _2q2q3 - a[2]) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[1]) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[2]) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m[3]);
          recipNorm = algebra_inverse_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
          s0 *= recipNorm;
          s1 *= recipNorm;
          s2 *= recipNorm;
          s3 *= recipNorm;

          // Apply feedback step
          qDot1 -= self->beta * s0;
          qDot2 -= self->beta * s1;
          qDot3 -= self->beta * s2;
          qDot4 -= self->beta * s3;
        }
    }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0 / sample_freq);
  q1 += qDot2 * (1.0 / sample_freq);
  q2 += qDot3 * (1.0 / sample_freq);
  q3 += qDot4 * (1.0 / sample_freq);

  // Normalise quaternion
  recipNorm = algebra_inverse_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  quaternion_set(self->orientation, q0, q1, q2, q3);
}

/*--------------------------------------------------------------------*/
void madgwick_ahrs_update_imu(MadgwickAHRS* self, ic_float_t sample_interval, Quaternion g, Quaternion a)
{
  ic_float_t q0 = self->orientation[0];
  ic_float_t q1 = self->orientation[1];
  ic_float_t q2 = self->orientation[2];
  ic_float_t q3 = self->orientation[3];
  ic_float_t sample_freq = 1.0/sample_interval;
  
  ic_float_t recipNorm;
  ic_float_t s0, s1, s2, s3;
  ic_float_t qDot1, qDot2, qDot3, qDot4;
  ic_float_t _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from g[2]roscope
  qDot1 = 0.5 * (-q1 * g[1] - q2 * g[2] - q3 * g[3]);
  qDot2 = 0.5 * (q0  * g[1] + q2 * g[3] - q3 * g[2]);
  qDot3 = 0.5 * (q0  * g[2] - q1 * g[3] + q3 * g[1]);
  qDot4 = 0.5 * (q0  * g[3] + q1 * g[2] - q2 * g[1]);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((a[1] == 0.0) && (a[2] == 0.0) && (a[3] == 0.0))) 
    {
      //also, compute only if accel is not too large.
      ic_float_t acc_magnitude = sqrt(a[1]*a[1] + a[2]*a[2] + a[3]*a[3]);
      if (fabs(acc_magnitude - self->kGravity) > self->kAccelerationThreshold)
        {
          // Normalise accelerometer measurement
          recipNorm = algebra_inverse_sqrt(a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
          a[1] *= recipNorm;
          a[2] *= recipNorm;
          a[3] *= recipNorm;   

          // Auxiliary ic_float_tiables to avoid repeated arithmetic
          _2q0 = 2.0 * q0;
          _2q1 = 2.0 * q1;
          _2q2 = 2.0 * q2;
          _2q3 = 2.0 * q3;
          _4q0 = 4.0 * q0;
          _4q1 = 4.0 * q1;
          _4q2 = 4.0 * q2;
          _8q1 = 8.0 * q1;
          _8q2 = 8.0 * q2;
          q0q0 = q0 * q0;
          q1q1 = q1 * q1;
          q2q2 = q2 * q2;
          q3q3 = q3 * q3;

          // Gradient decent algorithm corrective step
          s0 = _4q0 * q2q2 + _2q2 * a[1] + _4q0 * q1q1 - _2q1 * a[2];
          s1 = _4q1 * q3q3 - _2q3 * a[1] + 4.0 * q0q0 * q1 - _2q0 * a[2] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a[3];
          s2 = 4.0 * q0q0 * q2 + _2q0 * a[1] + _4q2 * q3q3 - _2q3 * a[2] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a[3];
          s3 = 4.0 * q1q1 * q3 - _2q1 * a[1] + 4.0 * q2q2 * q3 - _2q2 * a[2];
          recipNorm = algebra_inverse_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
          s0 *= recipNorm;
          s1 *= recipNorm;
          s2 *= recipNorm;
          s3 *= recipNorm;

          // Apply feedback step
          qDot1 -= self->beta * s0;
          qDot2 -= self->beta * s1;
          qDot3 -= self->beta * s2;
          qDot4 -= self->beta * s3;
        }
    }
  
  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0 / sample_freq);
  q1 += qDot2 * (1.0 / sample_freq);
  q2 += qDot3 * (1.0 / sample_freq);
  q3 += qDot4 * (1.0 / sample_freq);

  // Normalise quaternion
  recipNorm = algebra_inverse_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  quaternion_set(self->orientation, q0, q1, q2, q3);  
}

/*--------------------------------------------------------------------*/
void madgwick_ahrs_get_orientation(MadgwickAHRS* self, Quaternion copied_result)
{
  quaternion_copy(self->orientation, copied_result);
}
