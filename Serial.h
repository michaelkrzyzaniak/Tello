#ifndef __SERIAL__
#define __SERIAL__

#include "BOOL.h"

typedef enum serial_success_enum
{
  FAILURE = 0,
  SUCCESS = 1,
}serial_success_t;

typedef struct opaque_serial_struct Serial;

typedef void (*serial_receive_callback_t) (void* SELF, void* data, int num_bytes);

Serial*          serial_new           (char* port_name, long baud, serial_receive_callback_t receive_callback, void* callback_self);
Serial*          serial_destroy       (Serial* self);
serial_success_t serial_open          (Serial* self, char* name);
void             serial_close         (Serial* self);
BOOL             serial_is_open       (Serial* self);
serial_success_t serial_set_baud      (Serial* self, int baud);
serial_success_t serial_flush         (Serial* self);
serial_success_t serial_listen        (Serial* self, BOOL should_listen);
void             serial_print_ports   (Serial* self);


/*------------------------------------------------------------------------------------*/
/*!
 * @function    serial_setInputMode
 * @discussion  set the input processing mode. see the UNIX manual sub verbo 'termios' 
 *              for a more detailed description, and more information about how the 
 *              paramaters interact.
 * @param       x
 *                the [serial] object to operate upon.
 * @param       vtime
 *                milliseconds
 * @param       vmin
 *                byte count
 * @param       icanon
 *                true for canonical input processing, false for non-canonical
 * @result      SUCCESS or FAILURE. upon FAILURE, an error message is posted to the 
 *              'Max Window'
 */
serial_success_t serial_set_input_mode(Serial* self, int vtime, int vmin, BOOL icanon);

#endif //__SERIAL__
