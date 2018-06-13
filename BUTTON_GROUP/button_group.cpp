//-----------------------------------------------------------
//  Button group class
//
//  2016/02/22, Copyright (c) 2016 MIKAMI, Naoki
//-----------------------------------------------------------

#include "button_group.hpp"

namespace Mikami
{
    // Constructor
    ButtonGroup::ButtonGroup(LCD_DISCO_F469NI &lcd, TS_DISCO_F469NI &ts,
                             uint16_t x0, uint16_t y0,
                             uint16_t width, uint16_t height,
                             uint32_t color, uint32_t backColor,
                             uint16_t number, const string str[],
                             uint16_t spaceX, uint16_t spaceY,
                             uint16_t column,
                             sFONT &fonts, uint32_t textColor)
                            : numberOfButtons_(number), touchedNum_(-1)
    {
        buttons_ = new Button *[number];
        for (int n=0; n<number; n++)
        {
            div_t u1 = div(n, column);
            uint16_t x = x0 + u1.rem*(width + spaceX);
            uint16_t y = y0 + u1.quot*(height + spaceY);
            buttons_[n] = new Button(lcd, ts, x, y, width, height,
                                     color, backColor,
                                     str[n], fonts, textColor);
        }
    }

    // Destructor
    ButtonGroup::~ButtonGroup()
    {
        for (int n=0; n<numberOfButtons_; n++) delete buttons_[n];
        delete[] *buttons_;
    }

    // Draw button
    bool ButtonGroup::Draw(int num, uint32_t color, uint32_t textColor)
    {
        if (!Range(num)) return false;
        buttons_[num]->Draw(color, textColor);
        touchedNum_ = num;
        return true;
    }

    // Redraw button with original color
    bool ButtonGroup::Redraw(int num, uint32_t textColor)
    {
        if (!Range(num)) return false;
        buttons_[num]->Redraw(textColor);
        return true;
    }

    // Erase button with selected color
    bool ButtonGroup::Erase(int num, uint32_t color)
    {
        if (!Range(num)) return false;
        buttons_[num]->Draw(color, color);
        return true;
    }

    // Check touch detected for specified button
    bool ButtonGroup::Touched(int num)
    {
        if (!Range(num)) return false;
        bool touched = buttons_[num]->Touched();
        if (touched) touchedNum_ = num;
        return touched;
    }

    // Check touch detected for specified button and redraw
    bool ButtonGroup::Touched(int num, uint32_t color,
                              uint32_t textColor)
    {
        if (!Range(num)) return false;
        bool touched = buttons_[num]->Touched(color, textColor);
        if (touched)
        {
            if (Range(touchedNum_) && (num != touchedNum_))
                buttons_[touchedNum_]->Redraw();
            touchedNum_ = num;
        }
        return touched;
    }

    // Get touched number
    bool ButtonGroup::GetTouchedNumber(int &num)
    {
        if (buttons_[0]->PanelTouched())
        {
            for (int n=0; n<numberOfButtons_; n++)
                if (buttons_[n]->IsOnButton())
                {
                    num = n;
                    return true;
                }
            return false;
        }
        else
            return false;
    }

    // Get touched number and redraw button if touched
    bool ButtonGroup::GetTouchedNumber(int &num, uint32_t color)
    {
        if (GetTouchedNumber(num))
        {
            buttons_[num]->Draw(color);
            if (Range(touchedNum_) && (num != touchedNum_))
                buttons_[touchedNum_]->Redraw();
            touchedNum_ = num;
            return true;
        }
        else
            return false;
    }
}
