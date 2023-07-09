/*---------_------------_-----------------------------------------------
     / \  | | __ _  ___| |__  _ __ __ _   ___ 
    / _ \ | |/ _` |/ _ \ '_ \| '__/ _` | / __|
   / ___ \| | (_| |  __/ |_) | | | (_| || (__ 
  /_/   \_\_|\__, |\___|_.__/|_|  \__,_(_)___|
-------------|___/------------------------------------------------------
Created by Michael Krzyzaniak at Inside Coach
Copyright Inside Coach, 2017. All rights reserved
----------------------------------------------------------------------*/

#include "Algebra.h"

/*--------------------------------------------------------------------*/
ic_float_t algebra_inverse_sqrt(ic_float_t x)
{
  return (x > 0) ? 1.0 / sqrt(x) : 0;
}

ic_float_t algebra_scalef(ic_float_t x, ic_float_t in_min, ic_float_t in_max, ic_float_t out_min, ic_float_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*---------------------------------_------------------------------------
   __ _ _  _ __ _| |_ ___ _ _ _ _ (_)___ _ _  ___
  / _` | || / _` |  _/ -_) '_| ' \| / _ \ ' \(_-<
  \__, |\_,_\__,_|\__\___|_| |_||_|_\___/_||_/__/
-----|_|--------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
void quaternion_zero(Quaternion self)
{
  self[0] = self[1] = self[2] = self[3] = 0;
}

/*--------------------------------------------------------------------*/
void quaternion_copy(Quaternion self, Quaternion result)
{
  result[0] = self[0];
  result[1] = self[1];
  result[2] = self[2];
  result[3] = self[3];
}

/*--------------------------------------------------------------------*/
void quaternion_set(Quaternion self, ic_float_t w, ic_float_t x, ic_float_t y, ic_float_t z)
{
  self[0] = w; self[1] = x; self[2] = y; self[3] = z;
}

/*--------------------------------------------------------------------*/
BOOL quaternion_is_equal_to(Quaternion self, Quaternion q)
{
  return (self[0]==q[0]) && (self[1]==q[1]) && (self[2]==q[2]) && (self[3]==q[3]);
}

/*--------------------------------------------------------------------*/
void quaternion_set_angle_axis(Quaternion self, ic_float_t theta, ic_float_t vx, ic_float_t vy, ic_float_t vz)
{
  theta *= 0.5;
  ic_float_t cos_theta_2 = cos(theta);
  ic_float_t sin_theta_2 = sin(theta);
  
  self[0] = cos_theta_2;
  self[1] = sin_theta_2 * vx;
  self[2] = sin_theta_2 * vy;
  self[3] = sin_theta_2 * vz;
}

/*--------------------------------------------------------------------*/
// returns angle
ic_float_t quaternion_get_angle_axis(Quaternion self, Quaternion result /*pure vector quaternion*/)
{
  ic_float_t s = sqrt(1 - (self[0]*self[0]));
  if(s != 0)
    {
      result[0] = 0;
      result[1] = self[1] / s;
      result[2] = self[2] / s;
      result[3] = self[3] / s;
    }
  else
    quaternion_zero(result);
    
  return 2 * acos(self[0]); 
}

/*--------------------------------------------------------------------*/
void quaternion_multiply(Quaternion self, Quaternion multiplicand, Quaternion result)
{
  result[0] = self[0]*multiplicand[0] - self[1]*multiplicand[1] - self[2]*multiplicand[2] - self[3]*multiplicand[3];
  result[1] = self[0]*multiplicand[1] + self[1]*multiplicand[0] + self[2]*multiplicand[3] - self[3]*multiplicand[2];  
  result[2] = self[0]*multiplicand[2] - self[1]*multiplicand[3] + self[2]*multiplicand[0] + self[3]*multiplicand[1];
  result[3] = self[0]*multiplicand[3] + self[1]*multiplicand[2] - self[2]*multiplicand[1] + self[3]*multiplicand[0];
}

/*--------------------------------------------------------------------*/
void quaternion_vector_cross_product(Quaternion self, Quaternion multiplicand, Quaternion result)
{
  result[0] =   0;
  result[1] =  (self[2]*multiplicand[3]) - (multiplicand[2]*self[3]);
  result[2] = -(self[1]*multiplicand[3]) + (multiplicand[1]*self[3]);
  result[3] =  (self[1]*multiplicand[2]) - (multiplicand[1]*self[2]);
}

/*--------------------------------------------------------------------*/
ic_float_t quaternion_dot_product(Quaternion self, Quaternion multiplicand)
{
  return (self[0]*multiplicand[0]) + (self[1]*multiplicand[1]) + (self[2]*multiplicand[2]) + (self[3]*multiplicand[3]);
}

/*--------------------------------------------------------------------*/
ic_float_t quaternion_vector_dot_product(Quaternion self, Quaternion multiplicand)
{
  return (self[1]*multiplicand[1]) + (self[2]*multiplicand[2]) + (self[3]*multiplicand[3]);
}

/*--------------------------------------------------------------------*/
void quaternion_multiply_real(Quaternion self, ic_float_t real, Quaternion result)
{
  result[0] = self[0] * real;
  result[1] = self[1] * real;
  result[2] = self[2] * real;
  result[3] = self[3] * real;
}

/*--------------------------------------------------------------------*/
void quaternion_multiply_pointwise(Quaternion self, Quaternion multiplicand, Quaternion result)
{
  result[0] = self[0] * multiplicand[0];
  result[1] = self[1] * multiplicand[1];
  result[2] = self[2] * multiplicand[2];
  result[3] = self[3] * multiplicand[3];
}

/*--------------------------------------------------------------------*/
void quaternion_add(Quaternion self, Quaternion addend, Quaternion result)
{
  result[0] = self[0] + addend[0];
  result[1] = self[1] + addend[1];
  result[2] = self[2] + addend[2];
  result[3] = self[3] + addend[3];
}

/*--------------------------------------------------------------------*/
void quaternion_subtract(Quaternion self, Quaternion subtrahend, Quaternion result)
{
  result[0] = self[0] - subtrahend[0];
  result[1] = self[1] - subtrahend[1];
  result[2] = self[2] - subtrahend[2];
  result[3] = self[3] - subtrahend[3];
}

/*--------------------------------------------------------------------*/
void quaternion_invert(Quaternion self, Quaternion result)
{
  ic_float_t inv_norm = quaternion_dot_product(self, self);
  if(inv_norm != 0)
    inv_norm = 1.0 / inv_norm;
  quaternion_conjugate(self, result);
  quaternion_multiply_real(result, inv_norm, result);
}

/*--------------------------------------------------------------------*/
void quaternion_conjugate(Quaternion self, Quaternion result)
{
  result[0] =  self[0];  
  result[1] = -self[1];
  result[2] = -self[2];
  result[3] = -self[3];
}

/*--------------------------------------------------------------------*/
ic_float_t quaternion_norm(Quaternion self)
{
  return sqrt(self[0]*self[0] + self[1]*self[1] + self[2]*self[2] + self[3]*self[3]);
}

/*--------------------------------------------------------------------*/
ic_float_t quaternion_inverse_norm(Quaternion self)
{
  return algebra_inverse_sqrt(self[0]*self[0] + self[1]*self[1] + self[2]*self[2] + self[3]*self[3]);
}

/*--------------------------------------------------------------------*/
void quaternion_normalize(Quaternion self, Quaternion result)
{
  ic_float_t inv_norm = quaternion_inverse_norm(self);
  quaternion_multiply_real(self, inv_norm, result);
}

/*--------------------------------------------------------------------*/
//rotate vector around self -- self must lie on the unit hypersphere and vector must be pure
void quaternion_rotate_vector(Quaternion self, Quaternion vector, Quaternion result)
{
  Quaternion product;
  Quaternion inv_norm;
  
  quaternion_normalize(self, inv_norm);
  quaternion_invert   (inv_norm, inv_norm); 
  quaternion_multiply (inv_norm, vector, product);
  quaternion_multiply (product, inv_norm, result);
}

/*--------------------------------------------------------------------*/
void quaternion_to_euler_angles(Quaternion self, Quaternion result /* will be returned as (q[0] q[1] q[2] q[3]) = (rho theta phi psi) */)
{
  ic_float_t q0 = self[0];
  ic_float_t q1 = self[1];
  ic_float_t q2 = self[2];
  ic_float_t q3 = self[3];

  //Euler angles
  ic_float_t rho   = sqrt((q0*q0) + (q1*q1) + (q2*q2) + (q3*q3));
  ic_float_t phi   = atan2 (2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
  ic_float_t theta = asin  (2*(q0*q2 - q3*q1));
  ic_float_t psi   = atan2 (2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
  /*
  ic_float_t phi   = -atan2 (2*(q2*q3 + q0*q1), -(q0*q0) + (q1*q1) + (q2*q2) - (q3*q3));
  ic_float_t theta = -asin  (2*(q0*q2 - q1*q3));
  ic_float_t psi   =  atan2 (2*(q1*q2 + q0*q3), (q0*q0) + (q1*q1) - (q2*q2) - (q3*q3));
  */
  result[0] = rho;
  result[1] = theta;
  result[2] = phi;
  result[3] = psi;
}

/*--------------------------------------------------------------------*/
void quaternion_print(Quaternion self)
{
  fprintf(stderr, "w: %lf\tx: %lf\ty: %lf\tz: %lf\r\n", self[0], self[1], self[2], self[3]);
}

/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
struct opaque_quaternion_trapezoid_integral_struct
{ 
  Quaternion previous;
};

/*--------------------------------------------------------------------*/
QuaternionTrapezoidIntegral* quaternion_ti_new()
{
  QuaternionTrapezoidIntegral* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      quaternion_ti_init(self);
    }
  return self;
}

/*--------------------------------------------------------------------*/
void quaternion_ti_init(QuaternionTrapezoidIntegral* self)
{
  quaternion_zero(self->previous);
}

/*--------------------------------------------------------------------*/
QuaternionTrapezoidIntegral* quaternion_ti_destroy(QuaternionTrapezoidIntegral* self)
{
  if(self != NULL)
    {
      free(self);
    }
  return (QuaternionTrapezoidIntegral*) NULL;
}

/*--------------------------------------------------------------------*/
void quaternion_ti_integrate(QuaternionTrapezoidIntegral* self, ic_float_t delta_x, Quaternion next_sample, Quaternion running_result)
{
  quaternion_add           (next_sample   , self->previous, self->previous);
  quaternion_multiply_real (self->previous, 0.5*delta_x   , self->previous);
  quaternion_add           (running_result, self->previous, running_result);
  quaternion_copy          (next_sample   , self->previous);
}

/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
struct opaque_quaternion_simpsons_integral_struct
{ 
  Quaternion previous;
  Quaternion prev_prev;
  Quaternion scratch;
};

/*--------------------------------------------------------------------*/
QuaternionSimpsonsIntegral* quaternion_si_new()
{
  QuaternionSimpsonsIntegral* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      quaternion_si_init(self);
    }
  return self;
}

/*--------------------------------------------------------------------*/
void quaternion_si_init(QuaternionSimpsonsIntegral* self)
{
  quaternion_zero(self->previous);
  quaternion_zero(self->prev_prev);
  quaternion_zero(self->scratch);
}

/*--------------------------------------------------------------------*/
QuaternionSimpsonsIntegral* quaternion_si_destroy(QuaternionSimpsonsIntegral* self)
{
  if(self != NULL)
    {
      free(self);
    }
  return (QuaternionSimpsonsIntegral*) NULL;
}

/*--------------------------------------------------------------------*/
void quaternion_si_integrate(QuaternionSimpsonsIntegral* self, ic_float_t delta_x, Quaternion next_sample, Quaternion running_result)
{
  quaternion_multiply_real (self->previous, 4, self->scratch);
  quaternion_add           (self->scratch, next_sample, self->scratch);
  quaternion_add           (self->scratch, self->prev_prev, self->scratch);
  //3.0 b/c delta_x is the time from this to prev,i.e. (b-a)/2
  quaternion_multiply_real (self->scratch, delta_x / 3.0, self->scratch);
  quaternion_copy          (self->previous, self->prev_prev);
  quaternion_copy          (next_sample, self->previous);
  quaternion_add           (running_result, self->scratch, running_result);
}

/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
struct opaque_quaternion_derivative_struct
{ 
  int           order     ;
  Quaternion*   previous  ;
  ic_float_t* prev_delta;
  ic_float_t  delta_sum ;
  int           index     ;
};

/*--------------------------------------------------------------------*/
QuaternionDerivative* quaternion_derivative_new(int order)
{
  QuaternionDerivative* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->order = order;
      self->previous = calloc(self->order, sizeof(*(self->previous)));
      if(self->previous == NULL)
        return quaternion_derivative_destroy(self);

      self->prev_delta = calloc(self->order, sizeof(*(self->prev_delta)));
      if(self->prev_delta == NULL)
        return quaternion_derivative_destroy(self);
        
      quaternion_derivative_init(self);
    }
  return self;
}

/*--------------------------------------------------------------------*/
void quaternion_derivative_init(QuaternionDerivative* self)
{
  int i;
  self->delta_sum  = 0;
  self->index      = 0;
  
  for(i=0; i<self->order; i++)
    {
      quaternion_zero(self->previous[i]);
      self->prev_delta[i] = 0;
    }
}

/*--------------------------------------------------------------------*/
QuaternionDerivative* quaternion_derivative_destroy(QuaternionDerivative* self)
{
  if(self != NULL)
    {
      if(self->previous != NULL)
        free(self->previous);

      if(self->prev_delta != NULL)
        free(self->prev_delta);
              
      free(self);
    }
  return (QuaternionDerivative*) NULL;
}

/*--------------------------------------------------------------------*/
void quaternion_derivative_differentiate(QuaternionDerivative* self, ic_float_t delta_x, Quaternion next_sample, Quaternion result)
{
  if(delta_x == 0)
    quaternion_zero(result);
  else
    {
      self->delta_sum -= self->prev_delta[self->index];
      self->delta_sum += delta_x;
      quaternion_subtract(next_sample, self->previous[self->index], result);
      quaternion_multiply_real (result, 1.0/self->delta_sum, result);
      quaternion_copy(next_sample, self->previous[self->index]);
      self->prev_delta[self->index] = delta_x;
      ++self->index;
      self->index %= self->order;
    }
};


/*----------_--------------------_----------_----------------------------
 _ __  __ _| |_ _ _(_)_ __  __ _| |__ _ ___| |__ _ _ __ _ 
| '  \/ _` |  _| '_| \ \ / / _` | / _` / -_) '_ \ '_/ _` |
|_|_|_\__,_|\__|_| |_/_\_\ \__,_|_\__, \___|_.__/_| \__,_|
-----------------------------------|___/-------------------------------*/
struct opaque_matrix_struct
{
  ic_float_t* vals;
  unsigned rows, cols;
};

#define IX(self, row, col) (self->vals[row * self->cols + col])

/*-----------------------------------------------------------------------*/
Matrix* matrix_new(unsigned rows, unsigned cols)
{
  Matrix* self = calloc(1, sizeof(*self));
  if(self != NULL)
    {
      self->rows = rows;
      self->cols = cols;
      self->vals = calloc(rows * cols, sizeof(ic_float_t));
      if(self->vals == NULL)
        return matrix_destroy(self);
    }
  return self;
}

/*-----------------------------------------------------------------------*/
Matrix* matrix_destroy(Matrix* self)
{
  if(self != NULL)
    {
      if(self->vals != NULL)
        free(self->vals);
      free(self);
    }
  return (Matrix*)NULL;
}

/*-----------------------------------------------------------------------*/
unsigned          matrix_rows   (Matrix* self)
{
  return self->rows;
}

/*-----------------------------------------------------------------------*/
unsigned          matrix_cols   (Matrix* self)
{
  return self->cols;
}

/*-----------------------------------------------------------------------*/
ic_float_t* matrix_values          (Matrix* self)
{
  return self->vals;
}

/*-----------------------------------------------------------------------*/
ic_float_t matrix_get_value        (Matrix* self, unsigned row, unsigned col, matrix_ret_t* result)
{
  matrix_ret_t ret = MATRIX_NOT_CONFORMABLE;
  ic_float_t res = 0;

  if((row < self->rows) && (col < self->cols))
    {
      ret = MATRIX_CONFORMABLE;
      res = IX(self, row, col);
    }

  if(result != NULL)
    *result = ret;
  return res;
}

/*-----------------------------------------------------------------------*/
matrix_ret_t matrix_set_value    (Matrix* self, unsigned row, unsigned col, ic_float_t val)
{
  matrix_ret_t ret = MATRIX_NOT_CONFORMABLE;

  if((row < self->rows) && (col < self->cols))
    {
      ret = MATRIX_CONFORMABLE;
      IX(self, row, col) = val;
    }

  return ret;
}

/*--------------------------------------------------------------------*/
unsigned      matrix_num_entries(Matrix* self)
{
  return self->rows * self->cols;
}

/*-----------------------------------------------------------------------*/
matrix_ret_t  matrix_set       (Matrix* self, ic_float_t* vals)
{
  memcpy(self->vals, vals, matrix_num_entries(self) * sizeof(ic_float_t));
  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
matrix_ret_t  matrix_copy            (Matrix* self, Matrix* result)
{
  unsigned rows = self->rows, cols = self->cols;

  if((rows != result->rows) || (cols != result->cols))
    return MATRIX_NOT_CONFORMABLE;

  memcpy(result->vals, self->vals, rows * cols * sizeof(ic_float_t));
  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
matrix_ret_t  matrix_fill_identity   (Matrix* self)
{
  unsigned i;
  if(self->rows != self->cols)
    return MATRIX_NOT_CONFORMABLE;
    
  matrix_zero(self);
  
  for(i=0; i<self->rows; i++)
    IX(self, i, i) = 1;

  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
void   matrix_zero      (Matrix* self)
{
  memset(self->vals, 0, self->rows * self->cols * sizeof(ic_float_t));
}

/*-----------------------------------------------------------------------*/
// |a b|^T = |a c|
// |c d|     |b d|
matrix_ret_t matrix_transpose(Matrix* self, Matrix* result)
{
  if((self->rows != result->cols) || (self->cols != result->rows))
    return MATRIX_NOT_CONFORMABLE;

  unsigned i, j;
  
  for(i=0; i<self->rows; i++)
    for(j=0; j<self->cols; j++)
      IX(result, j, i) = IX(self, i, j);

  return MATRIX_CONFORMABLE;
}

/*--------------------------------------------------------------------*/
// |a b c|   |g h|   |ag+bi+ck ah+bj+cl|
// |d e f| x |i j| = |dg+ei+fk dh+ej+fl|
//           |k l| 
//a.cols must equal b.rows, and result.cols = a.rows and result.rows = b.cols 
matrix_ret_t matrix_multiply(Matrix* self, Matrix* multiplicand, Matrix* result)
{
  if(self->cols != multiplicand->rows)
    return MATRIX_NOT_CONFORMABLE;
  if((self->rows != result->rows) || (multiplicand->cols != result->cols))
    return MATRIX_NOT_CONFORMABLE;

  unsigned i, j, k;

  for(i=0; i<self->rows; i++)
    {
      for(k=0; k<multiplicand->cols; k++)
        {
          IX(result, i, k) = 0;
          for(j=0; j<multiplicand->rows; j++)
            IX(result, i, k) += IX(self, i, j) * IX(multiplicand, j, k);
        }
    }
  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
matrix_ret_t  matrix_multiply_scalar (Matrix* self, ic_float_t multiplicand, Matrix* result)
{
  if((self->rows != result->rows) || (self->cols != result->cols))
    return MATRIX_NOT_CONFORMABLE;

  unsigned i, j;
  
  for(i=0; i<self->rows; i++)
    for(j=0; j<self->cols; j++)
      IX(result, i, j) = IX(self, i, j) * multiplicand;

  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
matrix_ret_t matrix_add(Matrix* self, Matrix* addend, Matrix* result)
{
  if((self->rows != addend->rows) || (self->rows != result->rows) || (self->cols != addend->cols) || (self->cols != result->cols))
    return MATRIX_NOT_CONFORMABLE;

  unsigned i, j;
  
  for(i=0; i<self->rows; i++)
    for(j=0; j<self->cols; j++)
      IX(result, i, j) = IX(self, i, j) + IX(addend, i, j);

  return MATRIX_CONFORMABLE;
}

/*--------------------------------------------------------------------*/
matrix_ret_t matrix_add_I(Matrix* self, Matrix* result)
{
  unsigned i, j;
  
  if(self->rows != self->cols)
    return MATRIX_NOT_CONFORMABLE;
    
  for(i=0; i<self->rows; i++)
    ++IX(self, i, i);
  
  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
matrix_ret_t matrix_subtract(Matrix* self, Matrix* subtrehend, Matrix* result)
{
  if((self->rows != subtrehend->rows) || (self->rows != result->rows) || (self->cols != subtrehend->cols) || (self->cols != result->cols))
    return MATRIX_NOT_CONFORMABLE;

  unsigned i, j;

  for(i=0; i<self->rows; i++)
    for(j=0; j<self->cols; j++)
      IX(result, i, j) = IX(self, i, j) - IX(subtrehend, i, j);

  return MATRIX_CONFORMABLE;
}

/*--------------------------------------------------------------------*/
matrix_ret_t matrix_subtract_I(Matrix* self, Matrix* result)
{
  unsigned i, j;
  
  if(self->rows != self->cols)
    return MATRIX_NOT_CONFORMABLE;
    
  for(i=0; i<self->rows; i++)
    --IX(self, i, i);
  
  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
void matrix_swap_rows(Matrix* self, unsigned row_i, unsigned row_k)
{
  unsigned j;
  ic_float_t temp;
  
  for(j=0; j<self->cols; j++)
    {
      temp               = IX(self, row_i, j);
      IX(self, row_i, j) = IX(self, row_k, j);
      IX(self, row_k, j) = IX(self, row_i, j);
    }
}

/*-----------------------------------------------------------------------*/
  //matrix_invert (following 3 functions)
  
 /* Copyright 2015 Chandra Shekhar (chandraiitk AT yahoo DOT co DOT in).
   Homepage: https://sites.google.com/site/chandraacads
  * * */


 /* This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.
  * * */
  
 /* This function decomposes the matrix 'A' into L, U, and P. If successful,  
  * the L and the U are stored in 'A', and information about the pivot in 'P'.
  * The diagonal elements of 'L' are all 1, and therefore they are not stored. */
/*-----------------------------------------------------------------------*/  
matrix_ret_t matrix_invert(Matrix* self, Matrix* temp, Matrix* result)
{
  matrix_copy(self, result);
  int P[result->rows];
  
  matrix_ret_t nonsingular = matrix_lup_decompose(result, P);
  if(nonsingular == MATRIX_CONFORMABLE)
    return matrix_lup_inverse(result, P, temp);
  else
    nonsingular = matrix_invert_gauss_jordan_1(self, result);
    
  return nonsingular;
}

/*-----------------------------------------------------------------------*/
//result is returned in self
matrix_ret_t matrix_lup_decompose(Matrix* self, int* P/*pivot vector*/)  
{  
  int i, j, k, kd = 0, T;
  ic_float_t p, t;
  int size = self->rows;

  for(i=0; i<size; i++) P[i] = i;

  for(k=0; k<size-1; k++)
    {
      p = 0;
      for(i=k; i<size; i++)
        {
          t = IX(self, i, k);
          if(t < 0) t *= -1; //Abosolute value of 't'.
          if(t > p)
            {
              p = t;
              kd = i;
            }
        }
      if(p == 0)  //singular matrix
        return 0;

      T = P[kd];
      P[kd] = P[k];
      P[k] = T;
      for(i=0; i<size; i++)
        {
          t = IX(self, kd, i);
          IX(self, kd, i) = IX(self, k, i);
          IX(self, k, i) = t;
         }

      for(i=k+1; i<size; i++) //Performing substraction to decompose A as LU.
        {
          IX(self, i, k) /= IX(self, k, k);
          for(j=k+1; j<size; j++) IX(self, i, j) -= IX(self, i, k)*IX(self, k, j);
         }
    }
    //Now, 'A' contains the L (without the diagonal elements, which are all 1)  
    //and the U.  
    
  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
//result is returned in a
matrix_ret_t matrix_lup_inverse(Matrix* self/*lup matrix*/, int* P, Matrix* temp)
{
  if(self->rows != self->cols)
    return MATRIX_NOT_CONFORMABLE;
  if(temp->rows != temp->cols)
    return MATRIX_NOT_CONFORMABLE;  
  if(temp->rows != self->rows)
    return MATRIX_NOT_CONFORMABLE;
    
  int i, j, n, m;
  ic_float_t t;
  int size = self->rows;
  ic_float_t X[size];
  ic_float_t Y[size];

  for(n=0; n<size; n++) X[n] = Y[n] = 0;
  
  for(i=0; i<size; i++)
    {
      for(j = 0; j<size; j++) IX(temp, i, j) = 0;
      IX(temp, i, i) = 1;
 
      for(n=0; n<size; n++)
        {
          t = 0;
          for(m=0; m<=n-1; m++) t += IX(self, n, m) * Y[m];
          Y[n] = IX(temp, i, P[n]) - t;
        }

      for(n=size-1; n>=0; n--)
        {
          t = 0;
          for(m = n+1; m < size; m++) t += IX(self, n, m) * X[m];
          X[n] = (Y[n]-t)/IX(self, n, n);
        }//Now, X contains the solution.

      for(j = 0; j<size; j++) IX(temp, i, j) = X[j]; //Copying 'X' into the same row of 'B'.
    } //Now, 'temp' the transpose of the inverse of 'A'.

  /* Copying transpose of 'B' into 'a', which would the inverse of 'A'. */
  for(i=0; i<size; i++) for(j=0; j<size; j++) IX(self, i, j) = IX(temp, j, i);

  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
// Matrix Inversion Routine from http://www.arduino.cc/playground/Code/MatrixMath
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in 
// * NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
matrix_ret_t matrix_invert_gauss_jordan_1(Matrix* self, Matrix* result)
{
  if(self->rows != self->cols)
    return MATRIX_NOT_CONFORMABLE;

  matrix_copy(self, result);
  int n = self->rows;
  int pivrow;
  int i, j, k;
  int pivrows[n];
  ic_float_t temp, temp2;

  for(k=0; k<n; k++)
    {
      temp = 0;
      for(i=k; i<n; i++)
        {
          temp2 = fabs(IX(result, i, k));
          if(temp2 >= temp)
            {
              temp = temp2;
              pivrow = i;
             }
        }
    
      if(IX(result, pivrow, k) == 0) 
        return MATRIX_NOT_CONFORMABLE;
        
      if(pivrow != k)
        {
          for (j=0; j<n; j++)
            {
              temp = IX(result, k, j);
              IX(result, k, j) = IX(result, pivrow, j);
              IX(result, pivrow, j) = temp;
            }
        }
      pivrows[k] = pivrow;
    
      temp = 1 / IX(result, k, k);
      IX(result, k, k) = 1;
    
      for (j=0; j<n; j++) 
        IX(result, k, j) *= temp;
    
      for(i=0; i<n; i++)
        if(i != k)
          {
            temp = IX(result, i, k);
            IX(result, i, k) = 0;
            for(j=0; j<n; j++) 
              IX(result, i, j) -= IX(result, k, j) * temp;
          }
    }

  for (k=n-1; k>=0; k--)
    if(pivrows[k] != k)
      for (i = 0; i < n; i++)
        {
          temp = IX(result, i, k);
          IX(result, i, k) = IX(result, i, pivrows[k]);
          IX(result, i, pivrows[k]) = temp;
        }

  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
matrix_ret_t matrix_invert_gauss_jordan_2(Matrix* self, Matrix* result)
{
  //dimensions are checked in called functions
  matrix_fill_identity(result);
  return matrix_gauss_jordan_solve(self, result);
}

/*-----------------------------------------------------------------------*/
//Gauss-Jordan elimination method
// |a b| x |a b|^-1 = |1 0|
// |c d|   |c d|      |0 1|
matrix_ret_t matrix_gauss_jordan_solve(Matrix* self, Matrix* result)
{
  //only defined for square matrices with det(a) != 0
  if(self->rows != self->cols)
    return MATRIX_NOT_CONFORMABLE;
  if(result->rows != result->cols)
    return MATRIX_NOT_CONFORMABLE;
  if(result->rows != self->rows)
    return MATRIX_NOT_CONFORMABLE;

  unsigned i, j, k, n = self->rows;
  ic_float_t coefficient;
  //matrix_fill_identity(result);

  for(i=0; i<n; i++)
    {
      for(k=i; k<n; k++)
        if(IX(self, k, i) != 0)
          {
            if(k != i)
              {
                matrix_swap_rows(self, i, k);
                matrix_swap_rows(result, i, k);
              }
            break;
          }
      fprintf(stderr, "HERE\r\n");
      if(k==n)
        return MATRIX_NOT_CONFORMABLE; // det a == 0
    fprintf(stderr, "HERE2\r\n");
      if(IX(self, i, i) != 1)
        {
          coefficient = 1.0 / IX(self, i, i);
          for(j=0; j<self->cols; j++)
            IX(self     , i, j) *= coefficient;
          for(j=0; j<result->cols; j++)  
            IX(result, i, j) *= coefficient;
        }
      for(k=0; k<n; k++)
        {
          if((k != i) && (IX(self, k, i)) != 0)
            {
              coefficient = IX(self, k, i);
              for(j=0; j<self->cols; j++)
                IX(self     , k, j) -= coefficient * IX(self     , i, j);
              for(j=0; j<result->cols; j++)
                IX(result, k, j) -= coefficient * IX(result, i, j);
            }
        }
    }
  return MATRIX_CONFORMABLE;
}

/*--------------------------------------------------------------------*/
//https://rosettacode.org/wiki/Cholesky_decomposition
matrix_ret_t matrix_cholesky_decomposition(Matrix* self, Matrix* result) 
{
  if(self->rows != self->cols)
    return MATRIX_NOT_CONFORMABLE;
  if(result->rows != result->cols)
    return MATRIX_NOT_CONFORMABLE;  
  if(result->rows != self->rows)
    return MATRIX_NOT_CONFORMABLE;
    
  unsigned i, j, k;
 
  for(i=0; i<self->cols; i++)
    for(j=0; j<(i+1); j++) 
      {
        ic_float_t s = 0;
        for(k=0; k<j; k++)
          s += IX(result, i, k) * IX(result, j, k);
        IX(result, i, j) = (i == j) ?
          sqrt(IX(self, i, i) - s) :
          (1.0 / IX(result, j, j) * (IX(self, i, j) - s));
      }
      
  return MATRIX_CONFORMABLE;
}

/*--------------------------------------------------------------------*/
//https://stackoverflow.com/questions/4372224/fast-method-for-computing-3x3-symmetric-matrix-spectral-decomposition
  //all arguments are 3x3 matrices
  // A (self) must be a symmetric matrix.
  // returns Q and D such that 
  // Diagonal matrix D = QT * A * Q;  and  A = Q*D*QT
matrix_ret_t matrix_3_3_eigndecomposition(Matrix* self, Matrix* AQ /*result AQ*/, Matrix* Q /*result eigenvectors*/, Matrix* D /*result eigenvalues*/)
{
  const int maxsteps = 24;  // certainly wont need that many.
  int i, k0, k1, k2;
  ic_float_t o[]  = {0.0, 0.0, 0.0}; 
  ic_float_t m[]  = {0.0, 0.0, 0.0};
  ic_float_t q[]  = {0.0, 0.0, 0.0, 1.0};
  ic_float_t jr[] = {0.0, 0.0, 0.0, 0.0};
  ic_float_t sqw, sqx, sqy, sqz;
  ic_float_t temp1, temp2, mq;
  ic_float_t thet, sgn, t, c;
  for(i=0; i<maxsteps; ++i)
    {
      // quat to matrix
      sqx          = q[0]*q[0];
      sqy          = q[1]*q[1];
      sqz          = q[2]*q[2];
      sqw          = q[3]*q[3];
      IX(Q, 0, 0)  = ( sqx - sqy - sqz + sqw);
      IX(Q, 1, 1)  = (-sqx + sqy - sqz + sqw);
      IX(Q, 2, 2)  = (-sqx - sqy + sqz + sqw);
      temp1        = q[0]*q[1];
      temp2        = q[2]*q[3];
      IX(Q, 1, 0)  = 2.0 * (temp1 + temp2);
      IX(Q, 0, 1)  = 2.0 * (temp1 - temp2);
      temp1        = q[0]*q[2];
      temp2        = q[1]*q[3];
      IX(Q, 2, 0)  = 2.0 * (temp1 - temp2);
      IX(Q, 0, 2)  = 2.0 * (temp1 + temp2);
      temp1        = q[1]*q[2];
      temp2        = q[0]*q[3];
      IX(Q, 2, 1)  = 2.0 * (temp1 + temp2);
      IX(Q, 1, 2)  = 2.0 * (temp1 - temp2);

      // AQ = A * Q
      IX(AQ, 0, 0) = IX(Q, 0, 0)*IX(self, 0, 0)+IX(Q, 1, 0)*IX(self, 0, 1)+IX(Q, 2, 0)*IX(self, 0, 2);
      IX(AQ, 0, 1) = IX(Q, 0, 1)*IX(self, 0, 0)+IX(Q, 1, 1)*IX(self, 0, 1)+IX(Q, 2, 1)*IX(self, 0, 2);
      IX(AQ, 0, 2) = IX(Q, 0, 2)*IX(self, 0, 0)+IX(Q, 1, 2)*IX(self, 0, 1)+IX(Q, 2, 2)*IX(self, 0, 2);
      IX(AQ, 1, 0) = IX(Q, 0, 0)*IX(self, 0, 1)+IX(Q, 1, 0)*IX(self, 1, 1)+IX(Q, 2, 0)*IX(self, 1, 2);
      IX(AQ, 1, 1) = IX(Q, 0, 1)*IX(self, 0, 1)+IX(Q, 1, 1)*IX(self, 1, 1)+IX(Q, 2, 1)*IX(self, 1, 2);
      IX(AQ, 1, 2) = IX(Q, 0, 2)*IX(self, 0, 1)+IX(Q, 1, 2)*IX(self, 1, 1)+IX(Q, 2, 2)*IX(self, 1, 2);
      IX(AQ, 2, 0) = IX(Q, 0, 0)*IX(self, 0, 2)+IX(Q, 1, 0)*IX(self, 1, 2)+IX(Q, 2, 0)*IX(self, 2, 2);
      IX(AQ, 2, 1) = IX(Q, 0, 1)*IX(self, 0, 2)+IX(Q, 1, 1)*IX(self, 1, 2)+IX(Q, 2, 1)*IX(self, 2, 2);
      IX(AQ, 2, 2) = IX(Q, 0, 2)*IX(self, 0, 2)+IX(Q, 1, 2)*IX(self, 1, 2)+IX(Q, 2, 2)*IX(self, 2, 2);
      
      // D = Qt * AQ
      IX(D, 0, 0)  = IX(AQ, 0, 0)*IX(Q, 0, 0)+IX(AQ, 1, 0)*IX(Q, 1, 0)+IX(AQ, 2, 0)*IX(Q, 2, 0); 
      IX(D, 0, 1)  = IX(AQ, 0, 0)*IX(Q, 0, 1)+IX(AQ, 1, 0)*IX(Q, 1, 1)+IX(AQ, 2, 0)*IX(Q, 2, 1); 
      IX(D, 0, 2)  = IX(AQ, 0, 0)*IX(Q, 0, 2)+IX(AQ, 1, 0)*IX(Q, 1, 2)+IX(AQ, 2, 0)*IX(Q, 2, 2); 
      IX(D, 1, 0)  = IX(AQ, 0, 1)*IX(Q, 0, 0)+IX(AQ, 1, 1)*IX(Q, 1, 0)+IX(AQ, 2, 1)*IX(Q, 2, 0); 
      IX(D, 1, 1)  = IX(AQ, 0, 1)*IX(Q, 0, 1)+IX(AQ, 1, 1)*IX(Q, 1, 1)+IX(AQ, 2, 1)*IX(Q, 2, 1); 
      IX(D, 1, 2)  = IX(AQ, 0, 1)*IX(Q, 0, 2)+IX(AQ, 1, 1)*IX(Q, 1, 2)+IX(AQ, 2, 1)*IX(Q, 2, 2); 
      IX(D, 2, 0)  = IX(AQ, 0, 2)*IX(Q, 0, 0)+IX(AQ, 1, 2)*IX(Q, 1, 0)+IX(AQ, 2, 2)*IX(Q, 2, 0); 
      IX(D, 2, 1)  = IX(AQ, 0, 2)*IX(Q, 0, 1)+IX(AQ, 1, 2)*IX(Q, 1, 1)+IX(AQ, 2, 2)*IX(Q, 2, 1); 
      IX(D, 2, 2)  = IX(AQ, 0, 2)*IX(Q, 0, 2)+IX(AQ, 1, 2)*IX(Q, 1, 2)+IX(AQ, 2, 2)*IX(Q, 2, 2);
      o[0]         = IX(D, 1, 2);
      o[1]         = IX(D, 0, 2);
      o[2]         = IX(D, 0, 1);
      m[0]         = fabs(o[0]);
      m[1]         = fabs(o[1]);
      m[2]         = fabs(o[2]);

      k0      = (m[0] > m[1] && m[0] > m[2])?0: (m[1] > m[2])? 1 : 2; // index of largest element of offdiag
      k1      = (k0+1)%3;
      k2      = (k0+2)%3;
      if(o[k0] == 0.0)
        break;  // diagonal already
        
      thet    = (IX(D, k2, k2)-IX(D, k1, k1))/(2.0*o[k0]);
      sgn     = (thet > 0.0)?1.0:-1.0;
      thet   *= sgn; // make it positive
      t       = sgn /(thet +((thet < 1.E6)?sqrt(thet*thet+1.0):thet)) ; // sign(T)/(|T|+sqrt(T^2+1))
      c       = 1.0/sqrt(t*t+1.0); //  c= 1/(t^2+1) , t=s/c 

      if(c==1.0)
        break;  // no room for improvement - reached machine precision.

      jr[0 ]  = jr[1] = jr[2] = jr[3] = 0.0;
      jr[k0]  = sgn*sqrt((1.0-c)/2.0);  // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)  
      jr[k0] *= -1.0; // since our quat-to-matrix convention was for v*M instead of M*v
      jr[3 ]  = sqrt(1.0 - jr[k0] * jr[k0]);
      if(jr[3]==1.0)
        break; // reached limits of floating point precision

      q[0]    = (q[3]*jr[0] + q[0]*jr[3] + q[1]*jr[2] - q[2]*jr[1]);
      q[1]    = (q[3]*jr[1] - q[0]*jr[2] + q[1]*jr[3] + q[2]*jr[0]);
      q[2]    = (q[3]*jr[2] + q[0]*jr[1] - q[1]*jr[0] + q[2]*jr[3]);
      q[3]    = (q[3]*jr[3] - q[0]*jr[0] - q[1]*jr[1] - q[2]*jr[2]);
      mq      = algebra_inverse_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
      q[0]   *= mq;
      q[1]   *= mq;
      q[2]   *= mq;
      q[3]   *= mq;
  }
  return MATRIX_CONFORMABLE;
}

/*--------------------------------------------------------------------*/
// Adapted from
// http://www.cmi.ac.in/~ksutar/NLA2013/iterativemethods.pdf
// Page 10
matrix_ret_t matrix_spectral_radius(Matrix* self /*square*/, Matrix* result_eigenvector /*column of size self->rows*/, ic_float_t* result_eigenvalue)
{
  //must be square?
  int max_iterations = 1000000;
  int i, j, flag, n = self->rows;
  ic_float_t x0[n], y[n], eps=1e-5;
  
  if(n != self->cols)
    return MATRIX_NOT_CONFORMABLE;
  if(n != result_eigenvector->rows)
    return MATRIX_NOT_CONFORMABLE;
  
  for(i=0; i<n; i++)
    IX(result_eigenvector, i, 0) = 1;

  do{
    flag=0;
    for(i=0; i<n; i++)
      x0[i] = IX(result_eigenvector, i, 0);
    /* matrix_multiply(self, x0, y)*/
    for(i=0; i<n; i++)
      {
        y[i]=0;
        for(j=0; j<n; j++)
          y[i] += IX(self, i, j) * x0[j];
      }
    *result_eigenvalue = y[0];
 
    for(i=1; i<n; i++)
      if(*result_eigenvalue < y[i])
        *result_eigenvalue = y[i];
    if(*result_eigenvalue == 0)
      return MATRIX_NOT_CONFORMABLE;
    for(i=0; i<n; i++)
      IX(result_eigenvector, i, 0) = y[i] / *result_eigenvalue;
    for(i=0; i<n; i++)
      if(fabs(x0[i] - IX(result_eigenvector, i, 0)) > eps)
        {flag = 1; break;}
    if(--max_iterations <= 0)
      return MATRIX_NOT_CONFORMABLE;
    }while(flag == 1);
  
  return MATRIX_CONFORMABLE;
}

/*--------------------------------------------------------------------*/
//MatToQuat(float m[4][4], QUAT * quat)
//adapted from Nick Bobic: Gamasutra - Rotating Objects Using Quaternions
void __matrix_to_quaternion(Matrix* self, Quaternion q)
{
  ic_float_t  tr, s;
  int  i, j, k;
  int  nxt[] = {1, 2, 0};
  
  tr = IX(self, 0, 0) + IX(self, 1, 1) + IX(self, 2, 2);
  
  if (tr > 0.0) // check the diagonal
    {
      s = sqrt(tr + 1.0);
      q[0] = s * 0.5;
      s = 0.5 / s;
      q[1] = (IX(self, 1, 2) - IX(self, 2, 1)) * s;
      q[2] = (IX(self, 2, 0) - IX(self, 0, 2)) * s;
      q[3] = (IX(self, 0, 1) - IX(self, 1, 0)) * s;
    }
  else // diagonal is negative 
    {
      i = 0;
      if (IX(self, 1, 1) > IX(self, 0, 0)) i = 1;
      if (IX(self, 2, 2) > IX(self, i, i)) i = 2;
      j = nxt[i];
      k = nxt[j];
      s = sqrt((IX(self, i, i) - (IX(self, j, j) + IX(self, k, k))) + 1.0);
      q[i] = s * 0.5;
      if (s != 0.0) s = 0.5 / s;
      q[3] = (IX(self, j, k) - IX(self, k, j)) * s;
      q[j] = (IX(self, i, j) + IX(self, j, i)) * s;
      q[k] = (IX(self, i, k) + IX(self, k, i)) * s;
    }
}

/*--------------------------------------------------------------------*/
//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
//Jay A. Farrell Computation of the Quaternion from a Rotation Matrix
matrix_ret_t matrix_to_quaternion(Matrix* self, Quaternion q)
{
  if((self->cols != 3) || (self->rows != 3))
    return MATRIX_NOT_CONFORMABLE;

  ic_float_t tr = IX(self, 0, 0) + IX(self, 1, 1) + IX(self, 2, 2);
  ic_float_t s;

  if (tr > 0) 
    { 
      s = sqrt(tr+1.0) * 2; // S=4*qw 
      q[0] = 0.25 * s;
      q[1] = (IX(self, 2, 1) - IX(self, 1, 2)) / s;
      q[2] = (IX(self, 0, 2) - IX(self, 2, 0)) / s; 
      q[3] = (IX(self, 1, 0) - IX(self, 0, 1)) / s; 
    } 
  else if ((IX(self, 0, 0) > IX(self, 1, 1))&(IX(self, 0, 0) > IX(self, 2, 2))) 
    { 
      s = sqrt(1.0 + IX(self, 0, 0) - IX(self, 1, 1) - IX(self, 2, 2)) * 2; // S=4*qx 
      q[0] = (IX(self, 2, 1) - IX(self, 1, 2)) / s;
      q[1] = 0.25 * s;
      q[2] = (IX(self, 0, 1) + IX(self, 1, 0)) / s; 
      q[3] = (IX(self, 0, 2) + IX(self, 2, 0)) / s; 
    } 
  else if (IX(self, 1, 1) > IX(self, 2, 2)) 
    { 
      s = sqrt(1.0 + IX(self, 1, 1) - IX(self, 0, 0) - IX(self, 2, 2)) * 2; // S=4*qy
      q[0] = (IX(self, 0, 2) - IX(self, 2, 0)) / s;
      q[1] = (IX(self, 0, 1) + IX(self, 1, 0)) / s; 
      q[2] = 0.25 * s;
      q[3] = (IX(self, 1, 2) + IX(self, 2, 1)) / s; 
    } 
  else 
   { 
      s = sqrt(1.0 + IX(self, 2, 2) - IX(self, 0, 0) - IX(self, 1, 1)) * 2; // S=4*qz
      q[0] = (IX(self, 1, 0) - IX(self, 0, 1)) / s;
      q[1] = (IX(self, 0, 2) + IX(self, 2, 0)) / s;
      q[2] = (IX(self, 1, 2) + IX(self, 2, 1)) / s;
      q[3] = 0.25 * s;
    }
  return MATRIX_CONFORMABLE;
}

/*-----------------------------------------------------------------------*/
void     matrix_print(Matrix* self)
{
  unsigned i, j;
  float val;
  for(i=0; i<self->rows; i++)
    {
      fprintf(stderr, "|");
      for(j=0; j<self->cols; j++)
        fprintf(stderr, " % .4f", IX(self, i, j));
     fprintf(stderr, " |\r\n");
    }
  fprintf(stderr, "\r\n");
}

/*--------------------------------------------------------------------*/
void quaternion_to_matrix(Quaternion self, Matrix* result /* 3x3 */)
{
  Quaternion q_2;
  quaternion_multiply_pointwise(self, self, q_2);

  ic_float_t m[9] = 
  {
    q_2[0] + q_2[1] - q_2[2] - q_2[3]    , 2*(self[1]*self[2] - self[0]*self[3]) , 2*(self[1]*self[3] + self[0]*self[2]) ,
    2*(self[1]*self[2] + self[0]*self[3]), q_2[0] - q_2[1] + q_2[2] - q_2[3]     , 2*(self[2]*self[3] - self[0]*self[1]) ,
    2*(self[1]*self[3] - self[0]*self[2]), 2*(self[2]*self[3] + self[0]*self[1]) , q_2[0] - q_2[1] - q_2[2] + q_2[3],
  };

  matrix_set(result, m);
}
