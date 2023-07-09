/*!
 * @author
 *  Written by Michael Krzyzaniak
 *
 * @unsorted
 */

#ifndef __TELLO__
#define __TELLO__

#ifdef cplusplus
extern "C"{
#endif /*cplusplus*/

#include "BOOL.h"

#define TELLO_SEND_IP_ADDRESS "192.168.10.1"
#define TELLO_RECEIVE_IP_ADDRESS "0.0.0.0"
#define TELLO_SEND_COMMAND_PORT 8889
#define TELLO_RECEIVE_RESPONSE_PORT 8889
#define TELLO_RECEIVE_SENSOR_PORT 8890
#define TELLO_RECEIVE_VIDEO_PORT 11111

typedef struct opaque_tello_struct Tello;

typedef struct opaque_tello_video_struct
{
  int w;
  int h;
  int* data;
}tello_video_t;

typedef struct opaque_tello_sensor_struct
{
/*
  int pitch;
  int roll;
  int yaw;
  int vgx;
  int vgy;
  int vgz;
  int temph;
  int templ;
  int tof;
  int h;
  int batt;
  float baro;
  int time;
  float agx;
  float agy;
  float agz;
*/
  float attitude_degrees[3]; //pitch, roll, yaw
  float attitude_radians[3]; //pitch, roll, yaw
  float velocity[3];         //x, y, z
  float acceleration[3];     //x, y, z
  float temp[2];             //celcius
  float tof_distance;        //cm
  float height;              //cm
  float barometer;           //meters above sea level
  float batt;                //percent
  float motors_on_time;      //?
}tello_sensor_t;



typedef void (*tello_sensor_callback_t) (void* SELF, tello_sensor_t* sensor_data);
typedef void (*tello_video_callback_t)  (void* SELF, tello_video_t* video_data);

Tello*  tello_new                ();
Tello*  tello_destroy            (Tello* self);

/* returns true on success */
BOOL    tello_send_message(Tello* self, float reply_timeout_ms, char** returned_reply, const char *message, ...);

BOOL    tello_start_streaming_sensors(Tello* self, tello_sensor_callback_t sensor_callback);
BOOL    tello_stop_streaming_sensors(Tello* self);

BOOL    tello_start_streaming_video(Tello* self, tello_video_callback_t video_callback);
BOOL    tello_stop_streaming_video(Tello* self);

BOOL    tello_is_connected(Tello* self);
BOOL    tello_connect(Tello* self, BOOL wait_for_connection);
BOOL    tello_disconnect(Tello* self);

#ifdef cplusplus
}
#endif /*cplusplus*/

#endif /*____TELLO__*/
