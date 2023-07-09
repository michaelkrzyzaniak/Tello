#include "Serial.h"
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h> 
#include <libc.h>
#include <pthread.h>
#include <glob.h>

#define RECIEVE_BUFF_NUM_BYTES 256

/*------------------------------------------------------------------------------------*/
struct opaque_serial_struct
{
  int serial_file_descriptor;
  struct termios old_terminal_attributes;
  
  int preferred_baud;
  int preferred_should_listen;
  int preferred_vmin;
  int preferred_vtime;
  int preferred_icanon;
  
  pthread_t listen_thread;
  int thread_is_running;
  int thread_should_continue_running;
  
  serial_receive_callback_t receive_callback;
  void* serial_recieve_callback_self;
  
  void* receive_buffer;
};

/*------------------------------------------------------------------------------------*/
void* serial_listen_thread(void *SELF);
void  serial_signal_handler(int sig);
int   serial_write         (Serial* self, void* data, int numChars);
int   serial_read          (Serial* self, void* data, int numChars);

/*------------------------------------------------------------------------------------*/
Serial* serial_new(char* port_name, long baud, serial_receive_callback_t receive_callback, void* callback_self)
{
	Serial* self = (Serial*)calloc(1, sizeof(*self));
  
  signal(SIGUSR1, serial_signal_handler);
  
	if(self != NULL)
    {
      self->receive_buffer = malloc(RECIEVE_BUFF_NUM_BYTES);
      if(self->receive_buffer == NULL)
        return serial_destroy(self);
      
      self->receive_callback = receive_callback;
      self->serial_recieve_callback_self = callback_self;
      
      self->preferred_baud        = baud;
      self->preferred_should_listen= 1;
      self->preferred_vmin        = 0;
      self->preferred_vtime       = 0;
      self->preferred_icanon      = 0;
      
      self->serial_file_descriptor = -1;
      self->thread_is_running      = 0;
      self->thread_should_continue_running = 0;
      
      serial_open(self, port_name);
	  }
	return self;
}

/*------------------------------------------------------------------------------------*/
Serial* serial_destroy(Serial* self)
{
  if(self != NULL)
    {
      serial_close(self);
      
      if(self->receive_buffer != NULL)
        free(self->receive_buffer);
        
      free(self);
    }
  return (Serial*) NULL;
}

/*------------------------------------------------------------------------------------*/
serial_success_t serial_open(Serial* self, char* port)
{
	int error;
	unsigned long latency = 1;  //microseconds
	struct termios new_terminal_attributes;
    
  if(self->serial_file_descriptor != -1)
    serial_close(self);
  
	self->serial_file_descriptor = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK );
	if(self->serial_file_descriptor == -1) {  fprintf(stderr, "Cannot open %s\n", port); return FAILURE;  };
	
	error = ioctl(self->serial_file_descriptor, TIOCEXCL);
	if(error == -1) {  fprintf(stderr, "Error opening serial port %s\n", port); return FAILURE;  }
	
	fcntl(self->serial_file_descriptor, F_SETFL, 0);
	
	error = tcgetattr(self->serial_file_descriptor, &(self->old_terminal_attributes));
	if(error == -1) {  fprintf(stderr, "Error getting serial terminal attributes\n"); return FAILURE;  }
	
	//new_terminal_attributes = self->old_terminal_attributes;
	
	//cfmakeraw(&new_terminal_attributes);
	
	//error = tcsetattr(self->serial_file_descriptor, TCSANOW, &new_terminal_attributes);
	//if(error == -1) {  fprintf(stderr, "Error setting serial attributes\n"); return FAILURE;  }
	
  error = serial_set_input_mode(self, self->preferred_vtime, self->preferred_vmin, self->preferred_icanon);
  
	error = serial_set_baud(self, self->preferred_baud);
	
	error = ioctl(self->serial_file_descriptor, IOSSDATALAT, &latency);
	
  error = serial_flush(self);
  
  serial_listen(self, self->preferred_should_listen);

	fprintf(stderr, "Opened serial connection to %s\n", port);
	
	return SUCCESS;
}


/*------------------------------------------------------------------------------------*/
void serial_close(Serial* self)
{
  if(self->serial_file_descriptor != -1)
    { 
      int should_listen = self->preferred_should_listen;
      serial_listen(self, NO);
      self->preferred_should_listen = should_listen;
      tcsetattr(self->serial_file_descriptor, 0, &(self->old_terminal_attributes));
      close(self->serial_file_descriptor);
      self->serial_file_descriptor = -1;
    }
}

/*------------------------------------------------------------------------------------*/
serial_success_t serial_set_baud(Serial* self, int baud)
{
  int error;
  int previousBaud = self->preferred_baud;
  self->preferred_baud = baud;
  
  if(self->serial_file_descriptor != -1)
    { 
      error = ioctl(self->serial_file_descriptor, IOSSIOSPEED, &baud);
	    if(error == -1)
        {
	        fprintf(stderr, "Error setting serial baud rate\n" );
          self->preferred_baud = previousBaud;
        }
    }
  return SUCCESS;
}


/*------------------------------------------------------------------------------------*/
serial_success_t serial_flush(Serial* self)
{
  int error = SUCCESS;
  if(self->serial_file_descriptor != -1)
    {
	    int  whatToFlush = FREAD;
	    error = ioctl(self->serial_file_descriptor, TIOCFLUSH, &whatToFlush);
      error = (error != -1);
      if(error == FAILURE) fprintf(stderr, "Unable to flush serial input buffer\n");
    }
  return error;
}

/*------------------------------------------------------------------------------------*/
int serial_write (Serial* self, void* data, int num_chars)
{
  int i;
  return write(self->serial_file_descriptor, (void*)data, num_chars);
}

/*------------------------------------------------------------------------------------*/
int serial_read(Serial* self, void* data, int num_chars)
{
  return read(self->serial_file_descriptor, data, num_chars);
}


/*------------------------------------------------------------------------------------*/
/*
void serial_print(Serial* self, t_symbol *s, long argc, t_atom *argv)
{
  long i;
  t_atom *ap;
  char* data;
  int numBytes;
  
  for (i = 0, ap = argv; i < argc; i++, ap++) 
    {
      switch (atom_gettype(ap)) 
        {
          case A_LONG:
            asprintf(&data, "%li ", atom_getlong(ap));
            break;
          case A_FLOAT:
            asprintf(&data, "%f ", atom_getfloat(ap));
            break;
          case A_SYM:
            asprintf(&data, "%s ", atom_getsym(ap)->s_name);
            break;
          default:
            fprintf(stderr, "%ld: unknown atom type (%ld)", i+1, atom_gettype(ap));
            break;
         }
         
       numBytes = strlen(data);
       if(i==argc-1) numBytes--;
       serial_write(self, data, numBytes);
       free(data);
    }
}
*/

/*------------------------------------------------------------------------------------*/
serial_success_t serial_set_input_mode(Serial* self, int vtime, int vmin, BOOL icanon)
{
  int error = -1;
  int previousVtime  = self->preferred_vtime;
  int previousVmin   = self->preferred_vmin;
  int previousIcanon = self->preferred_icanon;
  self->preferred_vtime  = vtime;
  self->preferred_vmin   = vmin;
  self->preferred_icanon = icanon;
  
  if(self->serial_file_descriptor != -1)
    {
      struct termios attributes;
      memcpy(&attributes, &(self->old_terminal_attributes), sizeof(attributes));
      //if(tcgetattr(self->serial_file_descriptor, &attributes) != -1)
        {
          if(icanon)
            {
              attributes.c_lflag |= ICANON;
            }
          else
            {
              cfmakeraw(&attributes);
              attributes.c_lflag &= ~ICANON;
            }
          attributes.c_cc[VTIME] = vtime;
	        attributes.c_cc[VMIN ] = vmin ;
          error = tcsetattr(self->serial_file_descriptor, TCSANOW, &attributes);
          if(error == -1)
            {
              self->preferred_vtime  = previousVtime;
              self->preferred_vmin   = previousVmin;
              self->preferred_icanon = previousIcanon;
              perror("Unable to set serial input mode");
            }
        }
    }
  return error != -1;
}


/*------------------------------------------------------------------------------------*/
serial_success_t serial_listen(Serial* self, BOOL should_listen)
{
  self->preferred_should_listen = should_listen;
  if(should_listen)
    {
      if((self->serial_file_descriptor != -1) && (self->thread_is_running != 1))
        {
          self->thread_is_running = 1;
          pthread_create(&(self->listen_thread), NULL, serial_listen_thread, (void*)self);
        }
    }
  else //!should_listen
    {
      if(self->thread_is_running == 1)
        {
          self->thread_should_continue_running = 0;
          pthread_kill(self->listen_thread, SIGUSR1);
          pthread_join(self->listen_thread, NULL);
          self->thread_is_running = 0;
        }
    }
  return SUCCESS; 
}


/*------------------------------------------------------------------------------------*/
void* serial_listen_thread(void *SELF)
{
  Serial* self = (Serial*)SELF;
  
  self->thread_should_continue_running = 1;
  int num_bytes_read;
  
  while(self->thread_should_continue_running == 1)
  {
    num_bytes_read = serial_read (self, self->receive_buffer, RECIEVE_BUFF_NUM_BYTES-1);
    if(num_bytes_read < 0) //error or signal
      break;
    else if(num_bytes_read >= 1)
      {
        if(self->receive_callback != NULL)
          {
            //self->receive_buffer[num_bytes_read] = 0; //just for safety incase someone wants to printf
            *(((char*)self->receive_buffer) + num_bytes_read) = 0; //just for safety incase someone wants to printf
            self->receive_callback(self->serial_recieve_callback_self, self->receive_buffer, num_bytes_read);
          }
      }
  }

  return NULL;
}

/*------------------------------------------------------------------------------------*/
BOOL             serial_is_open       (Serial* self)
{
  return (self->serial_file_descriptor != -1);
}

/*------------------------------------------------------------------------------------*/
void serial_signal_handler(int sig)
{
  if(sig == SIGUSR1)
    return;
  return;
}

/*------------------------------------------------------------------------------------*/
void serial_print_ports(Serial* self)
{
  glob_t g;
  if (glob("/dev/cu*", 0, NULL, &g) == 0)
    {
      int i;
      for(i=0; i<g.gl_pathc; i++)
       {
         fprintf(stderr, "%s\n", g.gl_pathv[i]);
       }
    }
}
