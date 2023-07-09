typedef float ic_float_t;

//todo: Move some of this to IC_API.js
#include <math.h>
#include <stdio.h>   // NULL
#include <stdlib.h>  // calloc
#include <string.h>  // memcpy

#ifndef BOOL
#define BOOL int
#endif

#ifndef NO
#define NO   0
#define YES  (!NO)
#endif

//#define IC_BALL_MASS 
#define GRAVITY_METERS_PER_SECOND_2 9.81
