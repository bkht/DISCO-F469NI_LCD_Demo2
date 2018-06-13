/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "LCD_DISCO_F469NI.h"

// Constructor
LCD_DISCO_F469NI::LCD_DISCO_F469NI()
{
  BSP_LCD_Init();
  BSP_DMA_Init();
  BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE);
  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND, LCD_FB_START_ADDRESS);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
}

// Destructor
LCD_DISCO_F469NI::~LCD_DISCO_F469NI()
{

}

//=================================================================================================================
// Public methods
//=================================================================================================================

uint8_t LCD_DISCO_F469NI::Init(void)
{
  return BSP_LCD_Init();
}

uint8_t LCD_DISCO_F469NI::InitEx(LCD_OrientationTypeDef orientation)
{
  return BSP_LCD_InitEx(orientation);
}

void LCD_DISCO_F469NI::Reset(void)
{
  BSP_LCD_Reset();
}

uint32_t LCD_DISCO_F469NI::GetXSize(void)
{
  return BSP_LCD_GetXSize();
}

uint32_t LCD_DISCO_F469NI::GetYSize(void)
{
  return BSP_LCD_GetYSize();
}

void LCD_DISCO_F469NI::SetXSize(uint32_t imageWidthPixels)
{
  BSP_LCD_SetXSize(imageWidthPixels);
}

void LCD_DISCO_F469NI::SetYSize(uint32_t imageHeightPixels)
{
  BSP_LCD_SetYSize(imageHeightPixels);
}

void LCD_DISCO_F469NI::LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address)
{
  BSP_LCD_LayerDefaultInit(LayerIndex, FB_Address);
}

void LCD_DISCO_F469NI::SelectLayer(uint32_t LayerIndex)
{
  BSP_LCD_SelectLayer(LayerIndex);
}

void LCD_DISCO_F469NI::SetLayerVisible(uint32_t LayerIndex, FunctionalState State)
{
  BSP_LCD_SetLayerVisible(LayerIndex, State);
}

void LCD_DISCO_F469NI::SetTransparency(uint32_t LayerIndex, uint8_t Transparency)
{
  BSP_LCD_SetTransparency(LayerIndex, Transparency);
}

void LCD_DISCO_F469NI::SetLayerAddress(uint32_t LayerIndex, uint32_t Address)
{
  BSP_LCD_SetLayerAddress(LayerIndex, Address);
}

void LCD_DISCO_F469NI::SetLayerWindow(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  BSP_LCD_SetLayerWindow(LayerIndex, Xpos, Ypos, Width, Height);
}

void LCD_DISCO_F469NI::SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue)
{
  BSP_LCD_SetColorKeying(LayerIndex, RGBValue);
}

void LCD_DISCO_F469NI::ResetColorKeying(uint32_t LayerIndex)
{
  BSP_LCD_ResetColorKeying(LayerIndex);
}

void LCD_DISCO_F469NI::SetTextColor(uint32_t Color)
{
  BSP_LCD_SetTextColor(Color);
}

uint32_t LCD_DISCO_F469NI::GetTextColor(void)
{
  return BSP_LCD_GetTextColor();
}

void LCD_DISCO_F469NI::SetBackColor(uint32_t Color)
{
  BSP_LCD_SetBackColor(Color);
}

uint32_t LCD_DISCO_F469NI::GetBackColor(void)
{
  return BSP_LCD_GetBackColor();
}

void LCD_DISCO_F469NI::SetFont(sFONT *fonts)
{
  BSP_LCD_SetFont(fonts);
}

sFONT *LCD_DISCO_F469NI::GetFont(void)
{
  return BSP_LCD_GetFont();
}

uint32_t *LCD_DISCO_F469NI::GetFBStartAdress(void)
{
  return BSP_LCD_GetFBStartAdress();
}

uint32_t LCD_DISCO_F469NI::ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  return BSP_LCD_ReadPixel(Xpos, Ypos);
}

void LCD_DISCO_F469NI::Clear(uint32_t Color)
{
  BSP_LCD_Clear(Color);
}

void LCD_DISCO_F469NI::ClearStringLine(uint32_t Line)
{
  BSP_LCD_ClearStringLine(Line);
}

void LCD_DISCO_F469NI::DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii)
{
  BSP_LCD_DisplayChar(Xpos, Ypos, Ascii);
}

void LCD_DISCO_F469NI::DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode)
{
  BSP_LCD_DisplayStringAt(Xpos, Ypos, Text, Mode);
}

void LCD_DISCO_F469NI::DisplayStringAtLine(uint16_t Line, uint8_t *ptr)
{
  BSP_LCD_DisplayStringAtLine(Line, ptr);
}

void LCD_DISCO_F469NI::DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  BSP_LCD_DrawHLine(Xpos, Ypos, Length);
}

void LCD_DISCO_F469NI::DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  BSP_LCD_DrawVLine(Xpos, Ypos, Length);
}

void LCD_DISCO_F469NI::DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  BSP_LCD_DrawLine(x1, y1, x2, y2);
}

// https://github.com/ArminJo/STMF3-Discovery-Demos/blob/master/lib/graphics/src/thickLine.cpp
/**
 * modified Bresenham with optional overlap (esp. for drawThickLine())
 * Overlap draws additional pixel when changing minor direction - for standard bresenham overlap = LINE_OVERLAP_NONE (0)
 *
 *  Sample line:
 *
 *    00+
 *     -0000+
 *         -0000+
 *             -00
 *
 *  0 pixels are drawn for normal line without any overlap
 *  + pixels are drawn if LINE_OVERLAP_MAJOR
 *  - pixels are drawn if LINE_OVERLAP_MINOR
 */
void LCD_DISCO_F469NI::DrawLineOverlap(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint8_t aOverlap)
{
	BSP_LCD_DrawLineOverlap(aXStart, aYStart, aXEnd, aYEnd, aOverlap);
}

/**
 * Bresenham with thickness
 * no pixel missed and every pixel only drawn once!
 */
void LCD_DISCO_F469NI::DrawThickLine(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint16_t aThickness, uint8_t lineCaps)
{
	// lineCaps = 0 perpendicular line ending
	// lineCaps = 1 round line ending
	BSP_LCD_DrawThickLine(aXStart, aYStart, aXEnd, aYEnd, aThickness, lineCaps);
}

void LCD_DISCO_F469NI::DrawThickCircle2(uint16_t xc, uint16_t yc, uint16_t inner, uint16_t outer)
{
	circle2(xc, yc, inner, outer);
}

void LCD_DISCO_F469NI::drawArc(int x, int y, int r, int startAngle, int endAngle, int thickness)
{
	// Slow and open pixels, like in DrawThickCircle below
	int rDelta = -(thickness/2);
	int px, py, cx, cy;

	startAngle -= 90;
	endAngle   -= 90;

	if (startAngle!=endAngle)
	{
		for (int i=0; i<thickness; i++)
		{
			px = x + cos((startAngle*3.14)/180) * (r+rDelta+i);
			py = y + sin((startAngle*3.14)/180) * (r+rDelta+i);
			for (int d=startAngle+1; d<endAngle+1; d++)
			{
				cx = x + cos((d*3.14)/180) * (r+rDelta+i);
				cy = y + sin((d*3.14)/180) * (r+rDelta+i);
				DrawThickLine(px, py, cx, cy, 2, 0);
				px = cx;
				py = cy;
			}
		}
	}
	else
	{
		px = x + cos((startAngle*3.14)/180) * (r+rDelta);
		py = y + sin((startAngle*3.14)/180) * (r+rDelta);
		cx = x + cos((startAngle*3.14)/180) * (r-rDelta);
		cy = y + sin((startAngle*3.14)/180) * (r-rDelta);
		DrawThickLine(px, py, cx, cy, 2, 0);
	}
}

void LCD_DISCO_F469NI::drawThickArc(int x, int y, int r, int startAngle, int endAngle, int thickness)
{
//	if (startAngle < endAngle) {
		BSP_LCD_DrawThickArc(x, y, r, startAngle, endAngle, thickness);
//	}
//	else {
//		BSP_LCD_DrawThickArc(x, y, r, startAngle, 360, thickness);
//		BSP_LCD_DrawThickArc(x, y, r, 0, endAngle, thickness);
//	}
}

void LCD_DISCO_F469NI::drawThickArcOld(int x, int y, int r, int startAngle, int endAngle, int thickness)
{
	BSP_LCD_DrawThickArcOld(x, y, r, startAngle, endAngle, thickness);
}

void LCD_DISCO_F469NI::CreateAtan2Table(void)
{
	BSP_LCD_CreateAtan2Table();
}

float LCD_DISCO_F469NI::my_cos(float x)
{
	double prec = .01;
	double t , s ;
    int p;
    p = 0;
    s = 1.0;
    t = 1.0;
    while(fabs(t/s) > prec)
    {
        p++;
        t = (-t * x * x) / ((2 * p - 1) * (2 * p));
        s += t;
    }
    return s;
}

float LCD_DISCO_F469NI::my_sin(float x)
{
    int i = 1;
    double cur = x;
    double acc = 1;
    double fact= 1;
    double pow = x;
    while (fabs(acc) > .01 &&   i < 100){
        fact *= ((2*i)*(2*i+1));
        pow *= -1 * x*x;
        acc =  pow / fact;
        cur += acc;
        i++;
    }
    return cur;
}

void LCD_DISCO_F469NI::sincos_fast(float x, float *pS, float *pC)
{
     float cosOff4LUT[] = {
    		 0x1.000000p+00,
			 0x1.6A09E6p-01,
			 0x0.000000p+00,
			 -0x1.6A09E6p-01,
			 -0x1.000000p+00,
			 -0x1.6A09E6p-01,
			 0x0.000000p+00,
			 0x1.6A09E6p-01 };

    int     m, ms, mc;
    float   xI, xR, xR2;
    float   c, s, cy, sy;

    // Cody & Waite's range reduction Algorithm, [-pi/4, pi/4]
    xI  = floorf(x * 0x1.45F306p+00 + 0.5);
    xR  = (x - xI * 0x1.920000p-01) - xI*0x1.FB5444p-13;
    m   = (int) xI;
    xR2 = xR*xR;

    // Find cosine & sine index for angle offsets indices
    mc = (  m  ) & 0x7;     // two's complement permits upper modulus for negative numbers =P
    ms = (m + 6) & 0x7;     // two's complement permits upper modulus for negative numbers =P, note phase correction for sine.

    // Find cosine & sine
    cy = cosOff4LUT[mc];     // Load angle offset neighborhood cosine value
    sy = cosOff4LUT[ms];     // Load angle offset neighborhood sine value

    c = 0xf.ff79fp-4 + xR2 * (-0x7.e58e9p-4);                // TOL = 1.2786e-4
    // c = 0xf.ffffdp-4 + xR2 * (-0x7.ffebep-4 + xR2 * 0xa.956a9p-8);  // TOL = 1.7882e-7

     s = xR * (0xf.ffbf7p-4 + xR2 * (-0x2.a41d0cp-4));   // TOL = 4.835251e-6
    // s = xR * (0xf.fffffp-4 + xR2 * (-0x2.aaa65cp-4 + xR2 * 0x2.1ea25p-8));  // TOL = 1.1841e-8

    *pC = c*cy - s*sy;
    *pS = c*sy + s*cy;
}

float LCD_DISCO_F469NI::sin_fast(float x)
{
     float cosOff4LUT[] = {
    		 0x1.000000p+00,
			 0x1.6A09E6p-01,
			 0x0.000000p+00,
			 -0x1.6A09E6p-01,
			 -0x1.000000p+00,
			 -0x1.6A09E6p-01,
			 0x0.000000p+00,
			 0x1.6A09E6p-01 };

    int     m, ms, mc;
    float   xI, xR, xR2;
    float   c, s, cy, sy;

    // Cody & Waite's range reduction Algorithm, [-pi/4, pi/4]
    xI  = floorf(x * 0x1.45F306p+00 + 0.5);
    xR  = (x - xI * 0x1.920000p-01) - xI*0x1.FB5444p-13;
    m   = (int) xI;
    xR2 = xR*xR;

    // Find cosine & sine index for angle offsets indices
    mc = (  m  ) & 0x7;     // two's complement permits upper modulus for negative numbers =P
    ms = (m + 6) & 0x7;     // two's complement permits upper modulus for negative numbers =P, note phase correction for sine.

    // Find cosine & sine
    cy = cosOff4LUT[mc];     // Load angle offset neighborhood cosine value
    sy = cosOff4LUT[ms];     // Load angle offset neighborhood sine value

    c = 0xf.ff79fp-4 + xR2 * (-0x7.e58e9p-4);                // TOL = 1.2786e-4
    // c = 0xf.ffffdp-4 + xR2 * (-0x7.ffebep-4 + xR2 * 0xa.956a9p-8);  // TOL = 1.7882e-7

     s = xR * (0xf.ffbf7p-4 + xR2 * (-0x2.a41d0cp-4));   // TOL = 4.835251e-6
    // s = xR * (0xf.fffffp-4 + xR2 * (-0x2.aaa65cp-4 + xR2 * 0x2.1ea25p-8));  // TOL = 1.1841e-8

//    *pC = c*cy - s*sy;
    return c*sy + s*cy;
}

float LCD_DISCO_F469NI::cos_fast(float x)
{
     float cosOff4LUT[] = {
    		 0x1.000000p+00,
			 0x1.6A09E6p-01,
			 0x0.000000p+00,
			 -0x1.6A09E6p-01,
			 -0x1.000000p+00,
			 -0x1.6A09E6p-01,
			 0x0.000000p+00,
			 0x1.6A09E6p-01 };

    int     m, ms, mc;
    float   xI, xR, xR2;
    float   c, s, cy, sy;

    // Cody & Waite's range reduction Algorithm, [-pi/4, pi/4]
    xI  = floorf(x * 0x1.45F306p+00 + 0.5);
    xR  = (x - xI * 0x1.920000p-01) - xI*0x1.FB5444p-13;
    m   = (int) xI;
    xR2 = xR*xR;

    // Find cosine & sine index for angle offsets indices
    mc = (  m  ) & 0x7;     // two's complement permits upper modulus for negative numbers =P
    ms = (m + 6) & 0x7;     // two's complement permits upper modulus for negative numbers =P, note phase correction for sine.

    // Find cosine & sine
    cy = cosOff4LUT[mc];     // Load angle offset neighborhood cosine value
    sy = cosOff4LUT[ms];     // Load angle offset neighborhood sine value

    c = 0xf.ff79fp-4 + xR2 * (-0x7.e58e9p-4);                // TOL = 1.2786e-4
    // c = 0xf.ffffdp-4 + xR2 * (-0x7.ffebep-4 + xR2 * 0xa.956a9p-8);  // TOL = 1.7882e-7

     s = xR * (0xf.ffbf7p-4 + xR2 * (-0x2.a41d0cp-4));   // TOL = 4.835251e-6
    // s = xR * (0xf.fffffp-4 + xR2 * (-0x2.aaa65cp-4 + xR2 * 0x2.1ea25p-8));  // TOL = 1.1841e-8

     return c*cy - s*sy;
//    *pS = c*sy + s*cy;
}

float LCD_DISCO_F469NI::sqrt_fast(float x)
{
    union {float f; int i; } X, Y;
    float ScOff;
    uint8_t e;

    X.f = x;
    e = (X.i >> 23);           // f.SFPbits.e;

    if(x <= 0) return(0.0f);

    ScOff = ((e & 1) != 0) ? 1.0f : 0x1.6a09e6p0;  // NOTE: If exp=EVEN, b/c (exp-127) a (EVEN - ODD) := ODD; but a (ODD - ODD) := EVEN!!

    e = ((e + 127) >> 1);                            // NOTE: If exp=ODD,  b/c (exp-127) then flr((exp-127)/2)
    X.i = (X.i & ((1uL << 23) - 1)) | (0x7F << 23);  // Mask mantissa, force exponent to zero.
    Y.i = (((uint32_t) e) << 23);

    // Error grows with square root of the exponent. Unfortunately no work around like inverse square root... :(
    // Y.f *= ScOff * (0x9.5f61ap-4 + X.f*(0x6.a09e68p-4));        // Error = +-1.78e-2 * 2^(flr(log2(x)/2))
    // Y.f *= ScOff * (0x7.2181d8p-4 + X.f*(0xa.05406p-4 + X.f*(-0x1.23a14cp-4)));      // Error = +-7.64e-5 * 2^(flr(log2(x)/2))
    // Y.f *= ScOff * (0x5.f10e7p-4 + X.f*(0xc.8f2p-4 +X.f*(-0x2.e41a4cp-4 + X.f*(0x6.441e6p-8))));     // Error =  8.21e-5 * 2^(flr(log2(x)/2))
    // Y.f *= ScOff * (0x5.32eb88p-4 + X.f*(0xe.abbf5p-4 + X.f*(-0x5.18ee2p-4 + X.f*(0x1.655efp-4 + X.f*(-0x2.b11518p-8)))));   // Error = +-9.92e-6 * 2^(flr(log2(x)/2))
    // Y.f *= ScOff * (0x4.adde5p-4 + X.f*(0x1.08448cp0 + X.f*(-0x7.ae1248p-4 + X.f*(0x3.2cf7a8p-4 + X.f*(-0xc.5c1e2p-8 + X.f*(0x1.4b6dp-8))))));   // Error = +-1.38e-6 * 2^(flr(log2(x)/2))
    // Y.f *= ScOff * (0x4.4a17fp-4 + X.f*(0x1.22d44p0 + X.f*(-0xa.972e8p-4 + X.f*(0x5.dd53fp-4 + X.f*(-0x2.273c08p-4 + X.f*(0x7.466cb8p-8 + X.f*(-0xa.ac00ep-12)))))));    // Error = +-2.9e-7 * 2^(flr(log2(x)/2))
    Y.f *= ScOff * (0x3.fbb3e8p-4 + X.f*(0x1.3b2a3cp0 + X.f*(-0xd.cbb39p-4 + X.f*(0x9.9444ep-4 + X.f*(-0x4.b5ea38p-4 + X.f*(0x1.802f9ep-4 + X.f*(-0x4.6f0adp-8 + X.f*(0x5.c24a28p-12 ))))))));   // Error = +-2.7e-6 * 2^(flr(log2(x)/2))

    return(Y.f);
}

void LCD_DISCO_F469NI::DrawThickCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t lineThickness)
{
	BSP_LCD_DrawThickCircle(x0, y0, radius, lineThickness);
}

void LCD_DISCO_F469NI::DrawPixelBrightness(uint16_t x1, uint16_t y1, float brightness)
{
	BSP_LCD_DrawPixelBrightness(x1, y1, brightness);
}

void LCD_DISCO_F469NI::DrawLineAA(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	BSP_LCD_DrawLineAA(x1, y1, x2, y2);
}

void LCD_DISCO_F469NI::DrawLineThick(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t width)
{
  BSP_LCD_DrawLineThick(x1, y1, x2, y2, width);
}

void LCD_DISCO_F469NI::DrawLineThickPoly(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, float thickness)
{
    Point p[4];
    float angle = atan2(y2-y1,x2-x1);
//    const float M_PI = 3.14159265358979323846;
    p[0].X = (int16_t) x1 + thickness*cos(angle+M_PI/2);
    p[0].Y = (int16_t) y1 + thickness*sin(angle+M_PI/2);
    p[1].X = (int16_t) x1 + thickness*cos(angle-M_PI/2);
    p[1].Y = (int16_t) y1 + thickness*sin(angle-M_PI/2);
    p[2].X = (int16_t) x2 + thickness*cos(angle-M_PI/2);
    p[2].Y = (int16_t) y2 + thickness*sin(angle-M_PI/2);
    p[3].X = (int16_t) x2 + thickness*cos(angle+M_PI/2);
    p[3].Y = (int16_t) y2 + thickness*sin(angle+M_PI/2);

    BSP_LCD_FillPolygon(p, 4);

//  BSP_LCD_DrawLineThickPoly(x1, y1, x2, y2, thickness);
}

void LCD_DISCO_F469NI::DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  BSP_LCD_DrawRect(Xpos, Ypos, Width, Height);
}

void LCD_DISCO_F469NI::DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  BSP_LCD_DrawCircle(Xpos, Ypos, Radius);
}

void LCD_DISCO_F469NI::DrawPolygon(pPoint Points, uint16_t PointCount)
{
  BSP_LCD_DrawPolygon(Points, PointCount);
}

void LCD_DISCO_F469NI::DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  BSP_LCD_DrawEllipse(Xpos, Ypos, XRadius, YRadius);
}

void LCD_DISCO_F469NI::DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp)
{
  BSP_LCD_DrawBitmap(Xpos, Ypos, pbmp);
}

void LCD_DISCO_F469NI::FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  BSP_LCD_FillRect(Xpos, Ypos, Width, Height);
}

void LCD_DISCO_F469NI::FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  BSP_LCD_FillCircle(Xpos, Ypos, Radius);
}

void LCD_DISCO_F469NI::FillPolygon(pPoint Points, uint16_t PointCount)
{
  BSP_LCD_FillPolygon(Points, PointCount);
}

void LCD_DISCO_F469NI::FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  BSP_LCD_FillEllipse(Xpos, Ypos, XRadius, YRadius);
}

void LCD_DISCO_F469NI::DisplayOn(void)
{
  BSP_LCD_DisplayOn();
}

void LCD_DISCO_F469NI::DisplayOff(void)
{
  BSP_LCD_DisplayOff();
}

void LCD_DISCO_F469NI::DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code)
{
  BSP_LCD_DrawPixel(Xpos, Ypos, RGB_Code);
}

void LCD_DISCO_F469NI::PerformDma(uint32_t src, uint32_t dst, uint32_t size)
{
	BSP_PERFORM_DMA(src, dst, size);
}

//=================================================================================================================
// Private methods
//=================================================================================================================
