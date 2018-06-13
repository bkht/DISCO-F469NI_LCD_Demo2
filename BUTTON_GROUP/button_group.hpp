//-----------------------------------------------------------
//  Button group class -- Header
//
//  2016/02/22, Copyright (c) 2016 MIKAMI, Naoki
//-----------------------------------------------------------

#ifndef F746_BUTTON_GROUP_HPP
#define F746_BUTTON_GROUP_HPP

#include "button.hpp"
#include <string>

namespace Mikami
{
    class ButtonGroup
    {
    public:
        // Constructor
        ButtonGroup(LCD_DISCO_F469NI &lcd, TS_DISCO_F469NI &ts,
                    uint16_t x0, uint16_t y0,
                    uint16_t width, uint16_t height,
                    uint32_t color, uint32_t backColor,
                    uint16_t number, const string str[],
                    uint16_t spaceX = 0, uint16_t spaceY = 0,
                    uint16_t column = 1,
                    sFONT &fonts = Font12,
                    uint32_t textColor = LCD_COLOR_WHITE);

        // Destructor
        ~ButtonGroup();

        // Draw button
        bool Draw(int num, uint32_t color,
                  uint32_t textColor = LCD_COLOR_WHITE);

        // Draw all buttons
        void DrawAll(uint32_t color,
                     uint32_t textColor = LCD_COLOR_WHITE)
        {
            for (int n=0; n<numberOfButtons_; n++)
                buttons_[n]->Draw(color, textColor);
        }

        // Redraw button with original color
        bool Redraw(int num, uint32_t textColor = LCD_COLOR_WHITE);

        // Erase button with selected color
        bool Erase(int num, uint32_t color);

        // Check touch detected for specified button
        bool Touched(int num);

        // Check touch detected for specified button and redraw
        bool Touched(int num, uint32_t color,
                     uint32_t textColor = LCD_COLOR_WHITE);

        // Get touched number
        bool GetTouchedNumber(int &num);

        // Get touched number and redraw button if touched
        bool GetTouchedNumber(int &num, uint32_t color);

    private:
        Button **buttons_;
        int numberOfButtons_;
        int touchedNum_;

        // Check range of argument
        bool Range(int n)
        { return ((n >= 0) && (n < numberOfButtons_)); }

        // disallow copy constructor and assignment operator
        ButtonGroup(const ButtonGroup&);
        ButtonGroup& operator=(const ButtonGroup&);
    };
}
#endif  // F746_BUTTON_GROUP_HPP
