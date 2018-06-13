#ifndef MBED_MAIN_H
#define MBED_MAIN_H

#define BAUD_RATE       230400

// NUCLEO-F767ZI
#define USB_TX          SERIAL_TX   // PD_8 USART3 TX
#define USB_RX          SERIAL_RX   // PD_9 USART3 RX

#define FFT_MAX_Y 250;

//#define TFT_HOR_RES 800
//#define TFT_VER_RES 480

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

typedef struct
{
  int16_t x; /*!< geometric X position of drawing */
  int16_t y; /*!< geometric Y position of drawing */

} myPoint;

#endif
