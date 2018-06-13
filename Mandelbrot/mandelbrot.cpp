//------------------------------------------------------
//  Class for drawing Mandelbrot set
//  2015/11/03, Copyright (c) 2015 MIKAMI, Naoki
//-----------------------------------------------------------

#include "mandelbrot.hpp"

namespace Mikami
{
    void MandelbrotBase::Display(float x1, float x2, float y1, float y2)
    {
        x1_ = x1;
        y1_ = y1;

        ax_ = (x2 - x1)/(NX_ - 1);
        ay_ = (y2 - y1)/(NY_ - 1);

        for (int nx=0; nx<NX_; nx++)
            for (int ny=0; ny<NY_; ny++)
            {
                uint32_t color = GetColor(Converge(Complex(Fx(nx), Fy(ny))));
                LCD_->DrawPixel(X0_+nx, Y0_+ny, color);
            }
    }

    // discrimination of congergence
    int MandelbrotBase::Converge(Complex c)
    {
        Complex zn = 0;  // initial value

        for (int n=1; n<=MAX_COUNT_; n++)
        {
            zn = zn*zn + c;
            if ((Sqr(zn.real()) + Sqr(zn.imag())) > 4)
                return(n);  // diverge
        }
        return 0;           // converge
    }

    // Get the color corresponding to the number of repititions (Color1)
    uint32_t MandelbrotColor1::GetColor(int x)
    {
        if (x == 0) return LCD_COLOR_BLACK;
        uint32_t clr[] = { LCD_COLOR_BLUE,  LCD_COLOR_CYAN,        LCD_COLOR_LIGHTGREEN,
                           LCD_COLOR_GREEN, LCD_COLOR_YELLOW,      LCD_COLOR_LIGHTYELLOW,
                           LCD_COLOR_RED,   LCD_COLOR_DARKMAGENTA, LCD_COLOR_MAGENTA};
        return  clr[(x-1) % 9];
    }

    // Get the color corresponding to the number of repititions (Color2)
    uint32_t MandelbrotColor2::GetColor(int x)
    {
        int b = Max255(20 + x*50);
        int g = (x >  5) ? Max255((x -  5)*30) : 0;
        int r = (x > 20) ? Max255((x - 20)*10) : 0;
        return 0xFF000000 | (r << 16) | (g << 8) | b;
    }
}
