#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "Serial.h"
#include "Tello.h"

void i_hate_canonical_input_processing(void);
void make_stdin_cannonical_again();

void serial_receive_callback (void* SELF, void* data, int num_bytes);
void tello_sensor_callback   (void* SELF, tello_sensor_t* sensor_data);

int calibrated = 0;
int connected  = 0;
float tello_yaw = 0;
//float tello_init_height = -1000000;
//float tello_height = 0;
//int in_flight = 0;

/*--------------------------------------------------------------------*/
int main()
{
  Tello* tello = tello_new();
  if(!tello){perror("Unable to create Tello object\r\n"); return -1;}
  //Serial* serial = serial_new("/dev/cu.usbmodem14101", 1000000, serial_receive_callback, tello);
  Serial* serial = serial_new("/dev/cu.usbmodem78098001", 115200, serial_receive_callback, tello);
  if(!serial){perror("Unable to create Serial object\r\n"); return -1;}
  serial_set_input_mode(serial, 0, 0, YES);
  
  if(!serial_is_open(serial))
    {
      fprintf(stderr, "Unable to open Serial Port. Try one of these.\r\n");
      serial_print_ports(serial);
      return -2;
    }
  
  fprintf(stderr, "calibrating controller ... ");
  
  while(!calibrated)
    sleep(1);
  
  fprintf(stderr, "controller calibratd\r\n");
  
  tello_connect(tello, YES);
  tello_start_streaming_sensors(tello, tello_sensor_callback);
  
  connected = 1;
  fprintf(stderr, "tello connected\r\n");
  
  i_hate_canonical_input_processing();
  
  char c;
  for(;;)
    {
      c = getchar();
      
      switch(c)
        {
          case 't':
            tello_send_message(tello, 0, NULL, "takeoff");
          break;

          case 'l':
            tello_send_message(tello, 0, NULL, "land");
          break;

          case 'k':
            tello_send_message(tello, 0, NULL, "emergency");
          break;

          case 'q':
            goto out;
          
          default:
            continue;
        }
    }
    
  out:
  make_stdin_cannonical_again();
  tello_send_message(tello, 0, NULL, "emergency");
  tello = tello_destroy(tello);
  serial = serial_destroy(serial);
}

/*--------------------------------------------------------------------*/
void serial_receive_callback (void* SELF, void* data, int num_bytes)
{
  Tello* tello = (Tello*)SELF;
  int i;
  unsigned char* d = data;
  float attitude[3];
  int updown = 0;
  int tap = 0;
  int n = sscanf(data, "%f, %f, %f, %i, %i", &attitude[0], &attitude[1], &attitude[2], &updown, &tap);

  if(n==5)
    {
      //fprintf(stderr, "%f\t%f\t%f\t%i\r\n", attitude[0], attitude[1], attitude[2], updown);
      calibrated = 1;
      if(connected)
        {
          //controller: 0 is East(ish), North is positive, South negative, West is +- PI
          //tello: 0 random direction, turn right is positive, turn left negative
          if(updown == 2)
            {
              tello_send_message(tello, 0, NULL, "takeoff");
              updown = 0;
            }
          else if(tap > 0)
            {
              if(tap > 4) tap = 4;
              char directions[4] = {'l', 'b', 'r', 'f'};
              char direction = directions[tap-1];
              tello_send_message(tello, 0, NULL, "flip %c", direction);
            }
          else
            {
              attitude[2] *= -1;
  
              int left = round(-attitude[0] * 100);
              int forward = round(attitude[1] * 100);
              float heading_error;
              heading_error = (attitude[2] - tello_yaw);
              while(heading_error > M_PI)
                heading_error -= 2*M_PI;
              while(heading_error < -M_PI)
                heading_error += 2*M_PI;
              int turn_right = heading_error * 100;
          
              //float max = 100; //limited to 50 for indoor use
              float max = 100;
              updown *= max;
              if(left > max) left = max;
              if(left < -max) left = -max;
              if(forward > max) forward = max;
              if(forward < -max) forward = -max;
              if(turn_right > 100) turn_right = 100;
              if(turn_right < -100) turn_right = -100;
          
              tello_send_message(tello, 0, NULL, "rc %i %i %i %i", left, forward, updown, turn_right);
            }
        }
    }
  else
    fprintf(stderr, "%s", (char*)data);
}

/*--------------------------------------------------------------------*/
void tello_sensor_callback (void* SELF, tello_sensor_t* sensor_data)
{
  Tello* tello = (Tello*)SELF;
  
  //float yaw = sensor_data->attitude_degrees[2];
  float yaw = sensor_data->attitude_radians[2];
  
  tello_yaw = yaw;
  
  /*
  if(tello_init_height == -1000000)
    tello_init_height = sensor_data->barometer;
    
  
  //tello_height = sensor_data->height;
  tello_height = sensor_data->barometer;
  
  float accl_mag = sensor_data->velocity[0]*sensor_data->velocity[0];
  accl_mag += sensor_data->velocity[1]*sensor_data->velocity[1];
  accl_mag += sensor_data->velocity[2]*sensor_data->velocity[2];
  accl_mag = sqrt(accl_mag);
  fprintf(stderr, "height: %f\r\n", sensor_data->barometer);
  */
  //fprintf(stderr, "yaw: %f\r\n", tello_yaw);
}

/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
struct termios old_terminal_attributes;

void i_hate_canonical_input_processing(void)
{
  int error;
  struct termios new_terminal_attributes;
  
  int fd = fcntl(STDIN_FILENO,  F_DUPFD, 0);
  
  error = tcgetattr(fd, &(old_terminal_attributes));
  if(error == -1) {  fprintf(stderr, "Error getting serial terminal attributes\r\n"); return;}
  
  new_terminal_attributes = old_terminal_attributes;
  
  cfmakeraw(&new_terminal_attributes);
  
  error = tcsetattr(fd, TCSANOW, &new_terminal_attributes);
  if(error == -1) {  fprintf(stderr,  "Error setting serial attributes\r\n"); return; }
}

/*--------------------------------------------------------------------*/
void make_stdin_cannonical_again()
{
  int fd = fcntl(STDIN_FILENO,  F_DUPFD, 0);
  
  if (tcsetattr(fd, TCSANOW, &old_terminal_attributes) == -1)
    fprintf(stderr,  "Error setting serial attributes\r\n");
}
