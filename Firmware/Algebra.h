/*---------_------------_-----------------_-----------------------------
     / \  | | __ _  ___| |__  _ __ __ _  | |__  
    / _ \ | |/ _` |/ _ \ '_ \| '__/ _` | | '_ \ 
   / ___ \| | (_| |  __/ |_) | | | (_| |_| | | |
  /_/   \_\_|\__, |\___|_.__/|_|  \__,_(_)_| |_|
-------------|___/------------------------------------------------------

Created by Michael Krzyzaniak at Inside Coach
Copyright Inside Coach, 2017. All rights reserved
----------------------------------------------------------------------*/
#ifndef __IC_ALGEBRA__
#define __IC_ALGEBRA__
  
#if defined(__cplusplus)
extern "C"{
#endif   //(__cplusplus)

#include "Types.h"

/*--------------------------------------------------------------------*/
ic_float_t  algebra_inverse_sqrt(ic_float_t x);
ic_float_t  algebra_scalef(ic_float_t x, ic_float_t in_min, ic_float_t in_max, ic_float_t out_min, ic_float_t out_max);

/*--------------------------------------------------------------------*/
/*---------------------------------_------------------------------------
   __ _ _  _ __ _| |_ ___ _ _ _ _ (_)___ _ _  ___
  / _` | || / _` |  _/ -_) '_| ' \| / _ \ ' \(_-<
  \__, |\_,_\__,_|\__\___|_| |_||_|_\___/_||_/__/
-----|_|--------------------------------------------------------------*/
typedef    ic_float_t                     Quaternion[4];
void       quaternion_zero                (Quaternion self);
void       quaternion_copy                (Quaternion self, Quaternion result);
void       quaternion_set                 (Quaternion self, ic_float_t w, ic_float_t x, ic_float_t y, ic_float_t z);
BOOL       quaternion_is_equal_to         (Quaternion self, Quaternion q);
void       quaternion_set_angle_axis      (Quaternion self, ic_float_t theta, ic_float_t vx, ic_float_t vy, ic_float_t vz);
ic_float_t quaternion_get_angle_axis      (Quaternion self, Quaternion result /*pure vector quaternion, returns angle*/);
void       quaternion_multiply            (Quaternion self, Quaternion multiplicand, Quaternion result);
void       quaternion_vector_cross_product(Quaternion self, Quaternion multiplicand, Quaternion result);
ic_float_t quaternion_dot_product         (Quaternion self, Quaternion multiplicand);
ic_float_t quaternion_vector_dot_product  (Quaternion self, Quaternion multiplicand);
void       quaternion_multiply_real       (Quaternion self, ic_float_t real, Quaternion result);
void       quaternion_multiply_pointwise  (Quaternion self, Quaternion multiplicand, Quaternion result);
void       quaternion_add                 (Quaternion self, Quaternion addend, Quaternion result);
void       quaternion_subtract            (Quaternion self, Quaternion subtrahend, Quaternion result);
void       quaternion_invert              (Quaternion self, Quaternion result);
void       quaternion_conjugate           (Quaternion self, Quaternion result);
ic_float_t quaternion_norm                (Quaternion self);
ic_float_t quaternion_inverse_norm        (Quaternion self);
void       quaternion_normalize           (Quaternion self, Quaternion result);
void       quaternion_rotate_vector       (Quaternion self, Quaternion vector, Quaternion result);
void       quaternion_to_euler_angles     (Quaternion self, Quaternion result);
void       quaternion_print               (Quaternion self);

/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
typedef struct               opaque_quaternion_trapezoid_integral_struct QuaternionTrapezoidIntegral;
QuaternionTrapezoidIntegral* quaternion_ti_new      ();
void                         quaternion_ti_init     (QuaternionTrapezoidIntegral* self);
QuaternionTrapezoidIntegral* quaternion_ti_destroy  (QuaternionTrapezoidIntegral* self);
void                         quaternion_ti_integrate(QuaternionTrapezoidIntegral* self, ic_float_t delta_x, Quaternion next_sample, Quaternion running_result);

typedef struct               opaque_quaternion_simpsons_integral_struct QuaternionSimpsonsIntegral;
QuaternionSimpsonsIntegral*  quaternion_si_new      ();
void                         quaternion_si_init     (QuaternionSimpsonsIntegral* self);
QuaternionSimpsonsIntegral*  quaternion_si_destroy  (QuaternionSimpsonsIntegral* self);
void                         quaternion_si_integrate(QuaternionSimpsonsIntegral* self, ic_float_t delta_x, Quaternion next_sample, Quaternion running_result);

typedef struct               opaque_quaternion_derivative_struct QuaternionDerivative;
QuaternionDerivative*        quaternion_derivative_new(int order);
void                         quaternion_derivative_init(QuaternionDerivative* self);
QuaternionDerivative*        quaternion_derivative_destroy(QuaternionDerivative* self);
void                         quaternion_derivative_differentiate(QuaternionDerivative* self, ic_float_t delta_x, Quaternion next_sample, Quaternion result);

/*--------------------------------------------------------------------*/
/*----------_--------------------_----------_----------------------------
 _ __  __ _| |_ _ _(_)_ __  __ _| |__ _ ___| |__ _ _ __ _ 
| '  \/ _` |  _| '_| \ \ / / _` | / _` / -_) '_ \ '_/ _` |
|_|_|_\__,_|\__|_| |_/_\_\ \__,_|_\__, \___|_.__/_| \__,_|
-----------------------------------|___/-------------------------------*/
typedef struct opaque_matrix_struct Matrix;

typedef enum
{
  MATRIX_NOT_CONFORMABLE,
  MATRIX_CONFORMABLE,
}matrix_ret_t;

Matrix*       matrix_new                   (unsigned rows, unsigned cols);
Matrix*       matrix_destroy               (Matrix* self);
unsigned      matrix_rows                  (Matrix* self);
unsigned      matrix_cols                  (Matrix* self);
ic_float_t*   matrix_values                (Matrix* self);
ic_float_t    matrix_get_value             (Matrix* self, unsigned row, unsigned col, matrix_ret_t* result);
matrix_ret_t  matrix_set_value             (Matrix* self, unsigned row, unsigned col, ic_float_t val);
unsigned      matrix_num_entries           (Matrix* self);
matrix_ret_t  matrix_set                   (Matrix* self, ic_float_t* vals);
matrix_ret_t  matrix_copy                  (Matrix* self, Matrix* result);
matrix_ret_t  matrix_fill_identity         (Matrix* self);
void          matrix_zero                  (Matrix* self);
matrix_ret_t  matrix_transpose             (Matrix* self, Matrix* result);
matrix_ret_t  matrix_multiply              (Matrix* self, Matrix* multiplicand, Matrix* result);
matrix_ret_t  matrix_multiply_scalar       (Matrix* self, ic_float_t multiplicand, Matrix* result);
matrix_ret_t  matrix_add                   (Matrix* self, Matrix* addend, Matrix* result);
matrix_ret_t  matrix_add_I                 (Matrix* self, Matrix* result);
matrix_ret_t  matrix_subtract              (Matrix* self, Matrix* subtrehend, Matrix* result);
matrix_ret_t  matrix_subtract_I            (Matrix* self, Matrix* result);
void          matrix_swap_rows             (Matrix* self, unsigned row_i, unsigned row_k);
matrix_ret_t  matrix_invert                (Matrix* self, Matrix* temp, Matrix* result);
matrix_ret_t  matrix_lup_decompose         (Matrix* self, int* P/*pivot vector*/); //result is returned in self 
matrix_ret_t  matrix_lup_inverse           (Matrix* self  /*lup matrix*/, int* P, Matrix* temp); //result is returned in self 
matrix_ret_t  matrix_invert_gauss_jordan_1 (Matrix* self, Matrix* result);
matrix_ret_t  matrix_invert_gauss_jordan_2 (Matrix* self, Matrix* result);
matrix_ret_t  matrix_gauss_jordan_solve    (Matrix* self, Matrix* result);
matrix_ret_t  matrix_cholesky_decomposition(Matrix* self, Matrix* result); 
matrix_ret_t  matrix_3_3_eigndecomposition (Matrix* self /*must be symmetric*/, Matrix* AQ /*result AQ*/, Matrix* Q /*result eigenvectors*/, Matrix* D /*result eigenvalues*/);
matrix_ret_t  matrix_spectral_radius       (Matrix* self /*square*/, Matrix* result_eigenvector /*column of size self->rows*/, ic_float_t* result_eigenvalue); //returns largest eigenvalue / vector
matrix_ret_t  matrix_to_quaternion         (Matrix* self, Quaternion q);
void          quaternion_to_matrix         (Quaternion self, Matrix* result /* 3x3 */);
void          matrix_print                 (Matrix* a);


#if defined(__cplusplus)
}
#endif   //(__cplusplus)

#endif   //__IC_ALGEBRA__
