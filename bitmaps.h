#ifndef __BITMAPS_H
#define __BITMAPS_H

#ifdef __cplusplus
 extern "C" {
#endif

// $ convert -density 1200 -resize 480x480 theuy-logo-800x800.png -background white -flatten -alpha off theuy-logo-480x480.bmp

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

typedef struct _tBitmap
{
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
  uint16_t BitsPerPixel;	// 0x0888 = 24-bit, 0x0565 = 16-bit

} sBITMAP;

extern sBITMAP VV_740x178x16;

#ifdef __cplusplus
}
#endif

#endif /* __BITMAPS_H */
