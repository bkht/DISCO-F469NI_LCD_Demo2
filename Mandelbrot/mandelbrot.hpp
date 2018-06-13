//------------------------------------------------------
//  Class for drawing Mandelbrot set -- Header
//  2015/11/03, Copyright (c) 2015 MIKAMI, Naoki
//-----------------------------------------------------------

#ifndef F746_MANDELBROT_HPP
#define F746_MANDELBROT_HPP

#include "mbed.h"
#include "TS_DISCO_F469NI.h"
#include "LCD_DISCO_F469NI.h"
#include <complex>              // requisite for complex
typedef complex<float> Complex; // define "Complex"

namespace Mikami
{
    // Base class of Mandelbrot set drawer
    class MandelbrotBase
    {
    public:
        // Constructor
        MandelbrotBase(LCD_DISCO_F469NI &lcd,
                       int x0, int y0, int width, int height, int maxCount)
                      : LCD_(&lcd), X0_(x0), Y0_(y0), NX_(width), NY_(height),
                        MAX_COUNT_(maxCount) {}

        void Display(float x1, float x2, float y1, float y2);

        // translate position in the screen to real coordinate value
        float Fx(int x) { return ax_*x + x1_; }
        float Fy(int y) { return ay_*y + y1_; }

    protected:
        // limit maximum value to 255
        int Max255(int n) { return (n > 255) ? 255 : n; }

    private:
        LCD_DISCO_F469NI *const LCD_;
        const int X0_;          // origin of x axis
        const int Y0_;          // origin of y axis
        const int NX_;          // number of pixels for horizon
        const int NY_;          // number of pixels for vertical
        const int MAX_COUNT_;

        float x1_, y1_;
        float ax_, ay_;

        // Get the color corresponding to the number of repititions
        virtual uint32_t GetColor(int x) = 0;

        float Sqr(float x) { return x*x; }

        // discrimination of congergence
        int Converge(Complex c);
    };

    // Derived class of Mandelbrot set drawer to draw black and white pattern
    class MandelbrotBW : public MandelbrotBase
    {
    public:
        MandelbrotBW(LCD_DISCO_F469NI &lcd,
                     int x0, int y0, int width, int height, int maxCount = 100)
                    : MandelbrotBase(lcd, x0, y0, width, height, maxCount) {}

        // converge: black, diverge: white
        virtual uint32_t GetColor(int x)
        { return (x == 0) ? LCD_COLOR_BLACK : LCD_COLOR_WHITE; }
    };

    // Derived class of Mandelbrot set drawer to draw pattern 1
    class MandelbrotColor1 : public MandelbrotBase
    {
    public:
        MandelbrotColor1(LCD_DISCO_F469NI &lcd,
                         int x0, int y0, int width, int height, int maxCount = 100)
                        : MandelbrotBase(lcd, x0, y0, width, height, maxCount) {}

        // Get the color corresponding to the number of repititions
        virtual uint32_t GetColor(int x);
    };

    // Derived class of Mandelbrot set drawer to draw pattern 2
    class MandelbrotColor2 : public MandelbrotBase
    {
    public:
        MandelbrotColor2(LCD_DISCO_F469NI &lcd,
                         int x0, int y0, int width, int height, int maxCount = 100)
                        : MandelbrotBase(lcd, x0, y0, width, height, maxCount) {}

        // Get the color corresponding to the number of repititions
        virtual uint32_t GetColor(int x);
    };
}

#endif  // F746_MANDELBROT_HPP
