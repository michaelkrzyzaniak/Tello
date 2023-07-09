/*!
 * @author
 *  Written by Michael Krzyzaniak
 * @unsorted
 */

#include "Tello.h"
#include "Network.h"
#include <stdlib.h>  //calloc
#include <stdarg.h>  //va_start
#include <stdio.h>   //printf
#include <unistd.h>  //usleep
#include <math.h>    //M_PI
#include <pthread.h> //ptrhead

#define TELLO_SEND_COMMAND_BUFFER_SIZE       256
#define TELLO_RECEIVE_COMMAND_BUFFER_SIZE    256
#define TELLO_RECEIVE_SENSOR_BUFFER_SIZE     256
#define TELLO_RECEIVE_VIDEO_BUFFER_SIZE      256

void* tello_receive_response_run_loop(void* SELF);
void* tello_receive_sensor_run_loop  (void* SELF);
void* tello_receive_video_run_loop   (void* SELF);
  

struct opaque_tello_struct
{
  Network* command_net;
  Network* receive_sensor_net;
  Network* receive_video_net;
  
  char* send_command_buffer;
  char* receive_command_buffer;
  char* receive_command_buffer_2; //so we can pass the resulte to the user
  char* receive_sensor_buffer;
  char* receive_video_buffer;
  
  pthread_t receive_response_thread;
  pthread_t receive_sensor_thread;
  pthread_t receive_video_thread;
  
  pthread_mutex_t receive_response_mutex;
  pthread_mutex_t receive_sensor_mutex;
  pthread_mutex_t receive_video_mutex;
  
  tello_sensor_callback_t sensor_callback;
  tello_video_callback_t video_callback;
  
  tello_sensor_t  sensor_data;
  
  BOOL is_initalized;
  BOOL is_connected;
  
  BOOL message_did_arrive;
};


/*---------------------------------------------------------------------*/
Tello*  tello_new()
{
  Tello* self = (Tello*)calloc(1, sizeof(*self));
  if(self != NULL)
    {
      int error_1, error_2, error_3;
      
      self->command_net          = net_new();
      self->receive_sensor_net   = net_new();
      self->receive_video_net    = net_new();
      
      if((self->command_net   == NULL) || (self->receive_sensor_net == NULL) || (self->receive_video_net    == NULL))
        return tello_destroy(self);
        
      self->send_command_buffer     = calloc(TELLO_SEND_COMMAND_BUFFER_SIZE     , 1);
      self->receive_command_buffer  = calloc(TELLO_RECEIVE_COMMAND_BUFFER_SIZE  , 1);
      self->receive_command_buffer_2= calloc(TELLO_RECEIVE_COMMAND_BUFFER_SIZE  , 1);
      self->receive_sensor_buffer   = calloc(TELLO_RECEIVE_SENSOR_BUFFER_SIZE   , 1);
      self->receive_video_buffer    = calloc(TELLO_RECEIVE_VIDEO_BUFFER_SIZE    , 1);

      if((self->send_command_buffer   == NULL) || (self->receive_command_buffer == NULL) || (self->receive_command_buffer_2 == NULL) ||
         (self->receive_sensor_buffer == NULL) || (self->receive_video_buffer    == NULL))
        return tello_destroy(self);
      
      net_udp_connect (self->command_net         , TELLO_RECEIVE_RESPONSE_PORT);
      net_udp_connect (self->receive_sensor_net  , TELLO_RECEIVE_SENSOR_PORT  );
      net_udp_connect (self->receive_video_net   , TELLO_RECEIVE_VIDEO_PORT   );
      
      error_1 = pthread_mutex_init(&self->receive_response_mutex, NULL);
      error_2 = pthread_mutex_init(&self->receive_video_mutex, NULL);
      error_3 = pthread_mutex_init(&self->receive_sensor_mutex, NULL);
      
      if(error_1 || error_2 || error_2)
        return tello_destroy(self);
        
      error_1 = pthread_create(&self->receive_response_thread, NULL, tello_receive_response_run_loop, self);
      error_2 = pthread_create(&self->receive_video_thread   , NULL, tello_receive_video_run_loop   , self);
      error_3 = pthread_create(&self->receive_sensor_thread  , NULL, tello_receive_sensor_run_loop  , self);
      
      if(error_1 || error_2 || error_2)
        return tello_destroy(self);
      
      self->is_initalized = YES;
    }
  return self;
}

/*---------------------------------------------------------------------*/
Tello*  tello_destroy(Tello* self)
{
  if(self)
    {
      //safely land;
    
      if(self->receive_response_thread)
        {
          pthread_cancel(self->receive_response_thread);
          pthread_join(self->receive_response_thread, NULL);
        }
      if(self->receive_video_thread)
        {
          pthread_cancel(self->receive_video_thread);
          pthread_join(self->receive_video_thread, NULL);
        }
      if(self->receive_sensor_thread)
        {
          pthread_cancel(self->receive_sensor_thread);
          pthread_join(self->receive_sensor_thread, NULL);
        }
      
      self->command_net     = net_destroy(self->command_net);
      //self->receive_response_net = net_destroy(self->receive_response_net);
      self->receive_video_net    = net_destroy(self->receive_video_net);
      self->receive_sensor_net   = net_destroy(self->receive_sensor_net);

      if(self->send_command_buffer)      free(self->send_command_buffer);
      if(self->receive_command_buffer)   free(self->receive_command_buffer);
      if(self->receive_command_buffer_2) free(self->receive_command_buffer_2);
      if(self->receive_sensor_buffer)    free(self->receive_sensor_buffer);
      if(self->receive_video_buffer)     free(self->receive_video_buffer);
      
      free(self);
    }
  return (Tello*) NULL;
}

/*---------------------------------------------------------------------*/
BOOL    tello_send_message(Tello* self, float reply_timeout_ms, char** returned_reply, const char *message, ...)
{
  int n1, n2;
  BOOL result = NO;
  float ms;

  va_list args;
  va_start(args, message);
  
  self->receive_command_buffer_2[0] = '\0';
  if(returned_reply)
    *returned_reply = self->receive_command_buffer_2;

  n1 = vsnprintf(self->send_command_buffer, TELLO_SEND_COMMAND_BUFFER_SIZE-1, message, args);
  if((n1<1) || (n1>TELLO_SEND_COMMAND_BUFFER_SIZE-1))
    {
      sprintf(self->receive_command_buffer_2, "message too long");
      return NO;
    }
  self->send_command_buffer[n1] = '\0'; //just so I can printf it
  
  //lock thread
  self->message_did_arrive = NO;
  //unlock thread
  
  n2 = net_udp_send(self->command_net, self->send_command_buffer, n1, TELLO_SEND_IP_ADDRESS, TELLO_SEND_COMMAND_PORT);
  fprintf(stderr, "sending: %s\r\n", self->send_command_buffer);
  
  /*
  //just let it time out
  if(n2 != n1)
    {
      sprintf(self->receive_command_buffer_2, "unable to send");
      return NO;
    }
  */
  
  if(reply_timeout_ms <= 0)
    return YES;

  for(ms=0; ms<reply_timeout_ms; ms+=0.5)
  {
    if(self->message_did_arrive)
      {
        result = YES;
        break;
      }
    usleep(500);
  }

  if(!result)
    sprintf(self->receive_command_buffer_2, "timeout");
  else
    sprintf(self->receive_command_buffer_2, "%s", self->receive_command_buffer);

  return result;
}


/*---------------------------------------------------------------------*/
void* tello_receive_response_run_loop(void* SELF)
{
  Tello* self = (Tello*) SELF;
  char senders_address[16];
  
  for(;;)
    {
      int num_valid_bytes = net_udp_receive(self->command_net,
                                            self->receive_command_buffer,
                                            TELLO_RECEIVE_COMMAND_BUFFER_SIZE-1,
                                            senders_address);
      if(num_valid_bytes <= 0)
        continue; //return NULL ?
        
      self->receive_command_buffer[num_valid_bytes] = '\0';
      self->is_connected = self->message_did_arrive = YES;
      fprintf(stderr, "received: %s\r\n", self->receive_command_buffer);
    }
  return NULL;
}

/*---------------------------------------------------------------------*/
void* tello_receive_sensor_run_loop  (void* SELF)
{
  Tello* self = (Tello*) SELF;
  char senders_address[16];

  for(;;)
    {
      int num_valid_bytes = net_udp_receive(self->receive_sensor_net,
                                            self->receive_sensor_buffer,
                                            TELLO_RECEIVE_SENSOR_BUFFER_SIZE,
                                            senders_address);
      
      if(num_valid_bytes < 0)
        continue; //return NULL ?
      
      self->receive_sensor_buffer[num_valid_bytes-2] = '\0'; //remove the \r\n from the end so I can printf it
  
      int num_converstions = sscanf(self->receive_sensor_buffer,
        "pitch:%f;"
        "roll:%f;"
        "yaw:%f;"
        "vgx:%f;"
        "vgy:%f;"
        "vgz:%f;"
        "templ:%f;"
        "temph:%f;"
        "tof:%f;"
        "h:%f;"
        "bat:%f;"
        "baro:%f;"
        "time:%f;"
        "agx:%f;"
        "agy:%f;"
        "agz:%f;",
        &self->sensor_data.attitude_degrees[0],
        &self->sensor_data.attitude_degrees[1],
        &self->sensor_data.attitude_degrees[2],
        &self->sensor_data.velocity[0],
        &self->sensor_data.velocity[1],
        &self->sensor_data.velocity[2],
        &self->sensor_data.temp[0],
        &self->sensor_data.temp[1],
        &self->sensor_data.tof_distance,
        &self->sensor_data.height,
        &self->sensor_data.batt,
        &self->sensor_data.barometer,
        &self->sensor_data.motors_on_time,
        &self->sensor_data.acceleration[0],
        &self->sensor_data.acceleration[1],
        &self->sensor_data.acceleration[2]
        );

    //fprintf(stderr, "%s\r\n", self->receive_sensor_buffer);
    //fprintf(stderr, "num_converstions: %i\r\n", num_converstions);

    if(num_converstions == 16)
      {
        self->sensor_data.attitude_radians[0] = self->sensor_data.attitude_degrees[0] * M_PI / 180.0;
        self->sensor_data.attitude_radians[1] = self->sensor_data.attitude_degrees[1] * M_PI / 180.0;
        self->sensor_data.attitude_radians[2] = self->sensor_data.attitude_degrees[2] * M_PI / 180.0;
        //fprintf(stderr, "pitch: %f\troll: %f\tyaw: %f\r\n", self->sensor_data.attitude_radians[0], self->sensor_data.attitude_radians[1], self->sensor_data.attitude_radians[2]);
      }
      
      if(self->sensor_callback != NULL)
        self->sensor_callback(self, &self->sensor_data);

    }
  return NULL;
}

/*---------------------------------------------------------------------*/
void* tello_receive_video_run_loop   (void* SELF)
{
  Tello* self = (Tello*) SELF;
  char senders_address[16];
  
  for(;;)
    {
      int num_valid_bytes = net_udp_receive(self->receive_video_net,
                                            self->receive_video_buffer,
                                            TELLO_RECEIVE_VIDEO_BUFFER_SIZE,
                                            senders_address);
      if(num_valid_bytes < 0)
        continue; //return NULL ?
        
      fprintf(stderr, "video received %i bytes \r\n", num_valid_bytes);
    }
  return NULL;
}

/*---------------------------------------------------------------------*/
BOOL    tello_start_streaming_sensors(Tello* self, tello_sensor_callback_t sensor_callback)
{
  self->sensor_callback = sensor_callback;
  return tello_is_connected(self);
}

/*---------------------------------------------------------------------*/
BOOL    tello_stop_streaming_sensors(Tello* self)
{
  return YES;
}

/*---------------------------------------------------------------------*/
BOOL    tello_start_streaming_video(Tello* self, tello_video_callback_t video_callback)
{
  self->video_callback = video_callback;
  return tello_send_message(self, 0, NULL, "streamon");
}

/*---------------------------------------------------------------------*/
BOOL    tello_stop_streaming_video(Tello* self)
{
  return tello_send_message(self, 0, NULL, "streamoff");
}

/*---------------------------------------------------------------------*/
BOOL    tello_is_connected(Tello* self)
{
  return self->is_connected;
}

/*---------------------------------------------------------------------*/
BOOL    tello_connect(Tello* self, BOOL wait_for_connection)
{
  BOOL result = NO;
  char* error;
  if(wait_for_connection)
    {
      do{
        result = tello_send_message(self, 1000, &error, "command");
        if(!result)
          fprintf(stderr, "returned: %s\r\n", error);
      }while(!tello_is_connected(self));
    }
  else
    result = tello_send_message(self, 0, NULL, "command");
  
  //fprintf(stderr, "HEER2E: %s\r\n", self->receive_command_buffer_2);
  
  //remove this later
  //result = YES;
  
  return result;
}

/*---------------------------------------------------------------------*/
BOOL    tello_disconnect(Tello* self)
{
  return tello_is_connected(self);
}
