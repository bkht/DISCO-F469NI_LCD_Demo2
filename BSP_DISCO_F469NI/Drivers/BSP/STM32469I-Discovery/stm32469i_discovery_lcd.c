/**
  ******************************************************************************
  * @file    stm32469i_discovery_lcd.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    27-January-2017
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on STM32469I-Discovery evaluation board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* File Info: ------------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive directly in video mode a LCD TFT using the DSI interface.
     The following IPs are implied : DSI Host IP block working
     in conjunction to the LTDC controller.
   - This driver is linked by construction to LCD KoD mounted on board MB1166.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the LCD using the BSP_LCD_Init() function.
     o Select the LCD layer to be used using the BSP_LCD_SelectLayer() function.
     o Enable the LCD display using the BSP_LCD_DisplayOn() function.

  + Options
     o Configure and enable the color keying functionality using the
       BSP_LCD_SetColorKeying() function.
     o Modify in the fly the transparency and/or the frame buffer address
       using the following functions:
       - BSP_LCD_SetTransparency()
       - BSP_LCD_SetLayerAddress()

  + Display on LCD
     o Clear the whole LCD using BSP_LCD_Clear() function or only one specified string
       line using the BSP_LCD_ClearStringLine() function.
     o Display a character on the specified line and column using the BSP_LCD_DisplayChar()
       function or a complete string line using the BSP_LCD_DisplayStringAtLine() function.
     o Display a string line on the specified position (x,y in pixel) and align mode
       using the BSP_LCD_DisplayStringAtLine() function.
     o Draw and fill a basic shapes (dot, line, rectangle, circle, ellipse, .. bitmap)
       on LCD using the available set of functions.

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32469i_discovery_lcd.h"
#include "../Fonts/fonts.h"
//#include "../../../Utilities/Fonts/font24.c"
//#include "../../../Utilities/Fonts/font20.c"
//#include "../../../Utilities/Fonts/font16.c"
//#include "../../../Utilities/Fonts/font12.c"
//#include "../../../Utilities/Fonts/font8.c"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32469I_Discovery
  * @{
  */

/** @defgroup STM32469I-Discovery_LCD STM32469I Discovery LCD
  * @{
  */

/** @defgroup STM32469I-Discovery_LCD_Private_TypesDefinitions STM32469I Discovery LCD Private TypesDefinitions
  * @{
  */
static DSI_VidCfgTypeDef hdsivideo_handle;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;

/**
  * @}
  */

/** @defgroup STM32469I-Discovery_LCD_Private_Defines STM32469I Discovery LCD Private Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32469I-Discovery_LCD_Private_Macros STM32469I Discovery LCD Private Macros
  * @{
  */
#define ABS(X)                 ((X) > 0 ? (X) : -(X))

#define POLY_X(Z)              ((int32_t)((Points + (Z))->X))
#define POLY_Y(Z)              ((int32_t)((Points + (Z))->Y))
/**
  * @}
  */

/** @defgroup STM32469I-Discovery_LCD_Exported_Variables STM32469I Discovery LCD Exported Variables
  * @{
  */

/**
  * @}
  */


/** @defgroup STM32469I-Discovery_LCD_Private_Variables STM32469I Discovery LCD Private Variables
  * @{
  */

DMA2D_HandleTypeDef hdma2d_eval;
LTDC_HandleTypeDef  hltdc_eval;
DSI_HandleTypeDef hdsi_eval;
uint32_t lcd_x_size = OTM8009A_800X480_WIDTH;
uint32_t lcd_y_size = OTM8009A_800X480_HEIGHT;


//float ATAN2_TABLE_PPY[ATAN2_SIZE+1];
//float ATAN2_TABLE_PPX[ATAN2_SIZE+1];
//float ATAN2_TABLE_PNY[ATAN2_SIZE+1];
//float ATAN2_TABLE_PNX[ATAN2_SIZE+1];
//float ATAN2_TABLE_NPY[ATAN2_SIZE+1];
//float ATAN2_TABLE_NPX[ATAN2_SIZE+1];
//float ATAN2_TABLE_NNY[ATAN2_SIZE+1];
//float ATAN2_TABLE_NNX[ATAN2_SIZE+1];

//        int Size_Ac = 100000;
//        int Size_Ar = Size_Ac + 1;
//        float Pi = (float) M_PI;
//        float Pi_H = (float) (M_PI / 2.0);
//
//        float Atan2[Size_Ar];
//        float Atan2_PM[Size_Ar];
//        float Atan2_MP[Size_Ar];
//        float Atan2_MM[Size_Ar];
//
//        float Atan2_R[Size_Ar];
//        float Atan2_RPM[Size_Ar];
//        float Atan2_RMP[Size_Ar];
//        float Atan2_RMM[Size_Ar];
/**
  * @}
  */


/** @defgroup STM32469I-Discovery_LCD_Private_Variables STM32469I Discovery LCD Private Variables
  * @{
  */

/**
  * @brief  Default Active LTDC Layer in which drawing is made is LTDC Layer Background
  */
static uint32_t  ActiveLayer = LTDC_ACTIVE_LAYER_BACKGROUND;

/**
  * @brief  Current Drawing Layer properties variable
  */
static LCD_DrawPropTypeDef DrawProp[LTDC_MAX_LAYER_NUMBER];
/**
  * @}
  */

/** @defgroup STM32469I-Discovery_LCD_Private_FunctionPrototypes STM32469I Discovery LCD Private FunctionPrototypes
  * @{
  */
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3);
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex);
static void LL_ConvertLineToARGB8888(void * pSrc, void *pDst, uint32_t xSize, uint32_t ColorMode);
/**
  * @}
  */

/** @defgroup STM32469I-Discovery_LCD_Exported_Functions STM32469I Discovery LCD Exported Functions
  * @{
  */

/**
  * @brief  Initializes the DSI LCD.
  * @retval LCD state
  */
uint8_t BSP_LCD_Init(void)
{
  return (BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE));
}

/**
  * @brief  Initializes the DSI LCD. 
  * The ititialization is done as below:
  *     - DSI PLL ititialization
  *     - DSI ititialization
  *     - LTDC ititialization
  *     - OTM8009A LCD Display IC Driver ititialization
  * @retval LCD state
  */
uint8_t BSP_LCD_InitEx(LCD_OrientationTypeDef orientation)
{
  DSI_PLLInitTypeDef dsiPllInit;
  DSI_PHY_TimerTypeDef  PhyTimings;
  static RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  uint32_t LcdClock  = 27429; /*!< LcdClk = 27429 kHz */
  
  uint32_t laneByteClk_kHz = 0;
  uint32_t                   VSA; /*!< Vertical start active time in units of lines */
  uint32_t                   VBP; /*!< Vertical Back Porch time in units of lines */
  uint32_t                   VFP; /*!< Vertical Front Porch time in units of lines */
  uint32_t                   VACT; /*!< Vertical Active time in units of lines = imageSize Y in pixels to display */
  uint32_t                   HSA; /*!< Horizontal start active time in units of lcdClk */
  uint32_t                   HBP; /*!< Horizontal Back Porch time in units of lcdClk */
  uint32_t                   HFP; /*!< Horizontal Front Porch time in units of lcdClk */
  uint32_t                   HACT; /*!< Horizontal Active time in units of lcdClk = imageSize X in pixels to display */
  
  
  /* Toggle Hardware Reset of the DSI LCD using
  * its XRES signal (active low) */
  BSP_LCD_Reset();
  
  /* Call first MSP Initialize only in case of first initialization
  * This will set IP blocks LTDC, DSI and DMA2D
  * - out of reset
  * - clocked
  * - NVIC IRQ related to IP blocks enabled
  */
  BSP_LCD_MspInit();
  
/*************************DSI Initialization***********************************/  
  
  /* Base address of DSI Host/Wrapper registers to be set before calling De-Init */
  hdsi_eval.Instance = DSI;
  
  HAL_DSI_DeInit(&(hdsi_eval));
  
#if !defined(USE_STM32469I_DISCO_REVA)
  dsiPllInit.PLLNDIV  = 125;
  dsiPllInit.PLLIDF   = DSI_PLL_IN_DIV2;
  dsiPllInit.PLLODF   = DSI_PLL_OUT_DIV1;
#else  
  dsiPllInit.PLLNDIV  = 100;
  dsiPllInit.PLLIDF   = DSI_PLL_IN_DIV5;
  dsiPllInit.PLLODF   = DSI_PLL_OUT_DIV1;
#endif
  laneByteClk_kHz = 62500; /* 500 MHz / 8 = 62.5 MHz = 62500 kHz */
  
  /* Set number of Lanes */
  hdsi_eval.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  
  /* TXEscapeCkdiv = f(LaneByteClk)/15.62 = 4 */
  hdsi_eval.Init.TXEscapeCkdiv = laneByteClk_kHz/15620; 
  
  HAL_DSI_Init(&(hdsi_eval), &(dsiPllInit));
  
  /* Timing parameters for all Video modes
  * Set Timing parameters of LTDC depending on its chosen orientation
  */
  if(orientation == LCD_ORIENTATION_PORTRAIT)
  {
    lcd_x_size = OTM8009A_480X800_WIDTH;  /* 480 */
    lcd_y_size = OTM8009A_480X800_HEIGHT; /* 800 */                                
  }
  else
  {
    /* lcd_orientation == LCD_ORIENTATION_LANDSCAPE */
    lcd_x_size = OTM8009A_800X480_WIDTH;  /* 800 */
    lcd_y_size = OTM8009A_800X480_HEIGHT; /* 480 */                                
  }
  
  HACT = lcd_x_size;
  VACT = lcd_y_size;
  
  /* The following values are same for portrait and landscape orientations */
  VSA  = OTM8009A_480X800_VSYNC;
  VBP  = OTM8009A_480X800_VBP;
  VFP  = OTM8009A_480X800_VFP;
  HSA  = OTM8009A_480X800_HSYNC;
  HBP  = OTM8009A_480X800_HBP;
  HFP  = OTM8009A_480X800_HFP;
  
  
  hdsivideo_handle.VirtualChannelID = LCD_OTM8009A_ID;
  hdsivideo_handle.ColorCoding = LCD_DSI_PIXEL_DATA_FMT_RBG888;
  hdsivideo_handle.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  hdsivideo_handle.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  hdsivideo_handle.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;  
  hdsivideo_handle.Mode = DSI_VID_MODE_BURST; /* Mode Video burst ie : one LgP per line */
  hdsivideo_handle.NullPacketSize = 0xFFF;
  hdsivideo_handle.NumberOfChunks = 0;
  hdsivideo_handle.PacketSize                = HACT; /* Value depending on display orientation choice portrait/landscape */ 
  hdsivideo_handle.HorizontalSyncActive      = (HSA * laneByteClk_kHz) / LcdClock;
  hdsivideo_handle.HorizontalBackPorch       = (HBP * laneByteClk_kHz) / LcdClock;
  hdsivideo_handle.HorizontalLine            = ((HACT + HSA + HBP + HFP) * laneByteClk_kHz) / LcdClock; /* Value depending on display orientation choice portrait/landscape */
  hdsivideo_handle.VerticalSyncActive        = VSA;
  hdsivideo_handle.VerticalBackPorch         = VBP;
  hdsivideo_handle.VerticalFrontPorch        = VFP;
  hdsivideo_handle.VerticalActive            = VACT; /* Value depending on display orientation choice portrait/landscape */
  
  /* Enable or disable sending LP command while streaming is active in video mode */
  hdsivideo_handle.LPCommandEnable = DSI_LP_COMMAND_ENABLE; /* Enable sending commands in mode LP (Low Power) */
  
  /* Largest packet size possible to transmit in LP mode in VSA, VBP, VFP regions */
  /* Only useful when sending LP packets is allowed while streaming is active in video mode */
  hdsivideo_handle.LPLargestPacketSize = 16;
  
  /* Largest packet size possible to transmit in LP mode in HFP region during VACT period */
  /* Only useful when sending LP packets is allowed while streaming is active in video mode */
  hdsivideo_handle.LPVACTLargestPacketSize = 0;
  
  
  /* Specify for each region of the video frame, if the transmission of command in LP mode is allowed in this region */
  /* while streaming is active in video mode                                                                         */
  hdsivideo_handle.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;   /* Allow sending LP commands during HFP period */
  hdsivideo_handle.LPHorizontalBackPorchEnable  = DSI_LP_HBP_ENABLE;   /* Allow sending LP commands during HBP period */
  hdsivideo_handle.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;  /* Allow sending LP commands during VACT period */
  hdsivideo_handle.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;   /* Allow sending LP commands during VFP period */
  hdsivideo_handle.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;   /* Allow sending LP commands during VBP period */
  hdsivideo_handle.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE; /* Allow sending LP commands during VSync = VSA period */
  
  /* Configure DSI Video mode timings with settings set above */
  HAL_DSI_ConfigVideoMode(&(hdsi_eval), &(hdsivideo_handle));

  /* Configure DSI PHY HS2LP and LP2HS timings */
  PhyTimings.ClockLaneHS2LPTime = 35;
  PhyTimings.ClockLaneLP2HSTime = 35;
  PhyTimings.DataLaneHS2LPTime = 35;
  PhyTimings.DataLaneLP2HSTime = 35;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 10;
  HAL_DSI_ConfigPhyTimer(&hdsi_eval, &PhyTimings);

/*************************End DSI Initialization*******************************/ 
  
  
/************************LTDC Initialization***********************************/  
  
  /* Timing Configuration */    
  hltdc_eval.Init.HorizontalSync = (HSA - 1);
  hltdc_eval.Init.AccumulatedHBP = (HSA + HBP - 1);
  hltdc_eval.Init.AccumulatedActiveW = (lcd_x_size + HSA + HBP - 1);
  hltdc_eval.Init.TotalWidth = (lcd_x_size + HSA + HBP + HFP - 1);
  
  /* Initialize the LCD pixel width and pixel height */
  hltdc_eval.LayerCfg->ImageWidth  = lcd_x_size;
  hltdc_eval.LayerCfg->ImageHeight = lcd_y_size;   
  
  
  /* LCD clock configuration */
  /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 384 Mhz */
  /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 384 MHz / 7 = 54.857 MHz */
  /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 54.857 MHz / 2 = 27.429 MHz */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 7;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct); 
  
  /* Background value */
  hltdc_eval.Init.Backcolor.Blue = 0;
  hltdc_eval.Init.Backcolor.Green = 0;
  hltdc_eval.Init.Backcolor.Red = 0;
  hltdc_eval.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc_eval.Instance = LTDC;
  
  /* Get LTDC Configuration from DSI Configuration */
  HAL_LTDCEx_StructInitFromVideoConfig(&(hltdc_eval), &(hdsivideo_handle));
  
  /* Initialize the LTDC */  
  HAL_LTDC_Init(&hltdc_eval);

  /* Enable the DSI host and wrapper after the LTDC initialization
     To avoid any synchronization issue, the DSI shall be started after enabling the LTDC */
  HAL_DSI_Start(&(hdsi_eval));
  
#if !defined(DATA_IN_ExtSDRAM)
  /* Initialize the SDRAM */
  BSP_SDRAM_Init();
#endif /* DATA_IN_ExtSDRAM */
  
  /* Initialize the font */
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
  
/************************End LTDC Initialization*******************************/
  
  
/***********************OTM8009A Initialization********************************/  
  
  /* Initialize the OTM8009A LCD Display IC Driver (KoD LCD IC Driver)
  *  depending on configuration set in 'hdsivideo_handle'.
  */
  OTM8009A_Init(OTM8009A_FORMAT_RGB888, orientation);
  
/***********************End OTM8009A Initialization****************************/ 
  
  return LCD_OK;
}

/**
  * @brief  BSP LCD Reset
  *         Hw reset the LCD DSI activating its XRES signal (active low for some time)
  *         and desactivating it later.
  *         This signal is only cabled on Discovery Rev B and beyond.
  */
void BSP_LCD_Reset(void)
{
#if !defined(USE_STM32469I_DISCO_REVA)
/* EVAL Rev B and beyond : reset the LCD by activation of XRES (active low) connected to PH7 */
  GPIO_InitTypeDef  gpio_init_structure;

  __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Configure the GPIO on PH7 */
    gpio_init_structure.Pin   = GPIO_PIN_7;
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;

    HAL_GPIO_Init(GPIOH, &gpio_init_structure);

    /* Activate XRES active low */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);

    //HAL_Delay(20); /* wait 20 ms */
    wait_ms(20);
  
    /* Desactivate XRES */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);
    
    /* Wait for 10ms after releasing XRES before sending commands */
    //HAL_Delay(10);
    wait_ms(10);    
#else
  
#endif /* USE_STM32469I_DISCO_REVA == 0 */
}

/**
  * @brief  Gets the LCD X size.
  * @retval Used LCD X size
  */
uint32_t BSP_LCD_GetXSize(void)
{
  return (lcd_x_size);
}

/**
  * @brief  Gets the LCD Y size.
  * @retval Used LCD Y size
  */
uint32_t BSP_LCD_GetYSize(void)
{
  return (lcd_y_size);
}

/**
  * @brief  Set the LCD X size.
  * @param  imageWidthPixels : uint32_t image width in pixels unit
  */
void BSP_LCD_SetXSize(uint32_t imageWidthPixels)
{
  hltdc_eval.LayerCfg[ActiveLayer].ImageWidth = imageWidthPixels;
}

/**
  * @brief  Set the LCD Y size.
  * @param  imageHeightPixels : uint32_t image height in lines unit
  */
void BSP_LCD_SetYSize(uint32_t imageHeightPixels)
{
  hltdc_eval.LayerCfg[ActiveLayer].ImageHeight = imageHeightPixels;
}


/**
  * @brief  Initializes the LCD layers.
  * @param  LayerIndex: Layer foreground or background
  * @param  FB_Address: Layer frame buffer
  */
void BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address)
{
    LCD_LayerCfgTypeDef  Layercfg;

  /* Layer Init */
  Layercfg.WindowX0 = 0;
  Layercfg.WindowX1 = BSP_LCD_GetXSize();
  Layercfg.WindowY0 = 0;
  Layercfg.WindowY1 = BSP_LCD_GetYSize(); 
  Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  Layercfg.FBStartAdress = FB_Address;
  Layercfg.Alpha = 255;
  Layercfg.Alpha0 = 0;
  Layercfg.Backcolor.Blue = 0;
  Layercfg.Backcolor.Green = 0;
  Layercfg.Backcolor.Red = 0;
  Layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  Layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  Layercfg.ImageWidth = BSP_LCD_GetXSize();
  Layercfg.ImageHeight = BSP_LCD_GetYSize();
  
  HAL_LTDC_ConfigLayer(&hltdc_eval, &Layercfg, LayerIndex); 
  
  DrawProp[LayerIndex].BackColor = LCD_COLOR_WHITE;
  DrawProp[LayerIndex].pFont     = &Font24;
  DrawProp[LayerIndex].TextColor = LCD_COLOR_BLACK;
}


/**
  * @brief  Selects the LCD Layer.
  * @param  LayerIndex: Layer foreground or background
  */
void BSP_LCD_SelectLayer(uint32_t LayerIndex)
{
  ActiveLayer = LayerIndex;
}

/**
  * @brief  Sets an LCD Layer visible
  * @param  LayerIndex: Visible Layer
  * @param  State: New state of the specified layer
  *          This parameter can be one of the following values:
  *            @arg  ENABLE
  *            @arg  DISABLE
  */
void BSP_LCD_SetLayerVisible(uint32_t LayerIndex, FunctionalState State)
{
  if(State == ENABLE)
  {
    __HAL_LTDC_LAYER_ENABLE(&(hltdc_eval), LayerIndex);
  }
  else
  {
    __HAL_LTDC_LAYER_DISABLE(&(hltdc_eval), LayerIndex);
  }
  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&(hltdc_eval));
  
}

/**
  * @brief  Configures the transparency.
  * @param  LayerIndex: Layer foreground or background.
  * @param  Transparency: Transparency
  *           This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF
  */
void BSP_LCD_SetTransparency(uint32_t LayerIndex, uint8_t Transparency)
{
  
  HAL_LTDC_SetAlpha(&(hltdc_eval), Transparency, LayerIndex);
  
}

/**
  * @brief  Sets an LCD layer frame buffer address.
  * @param  LayerIndex: Layer foreground or background
  * @param  Address: New LCD frame buffer value
  */
void BSP_LCD_SetLayerAddress(uint32_t LayerIndex, uint32_t Address)
{
  
  HAL_LTDC_SetAddress(&(hltdc_eval), Address, LayerIndex);
  
}

/**
  * @brief  Sets display window.
  * @param  LayerIndex: Layer index
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height
  */
void BSP_LCD_SetLayerWindow(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Reconfigure the layer size */
  HAL_LTDC_SetWindowSize(&(hltdc_eval), Width, Height, LayerIndex);
  
  /* Reconfigure the layer position */
  HAL_LTDC_SetWindowPosition(&(hltdc_eval), Xpos, Ypos, LayerIndex);
  
}

/**
  * @brief  Configures and sets the color keying.
  * @param  LayerIndex: Layer foreground or background
  * @param  RGBValue: Color reference
  */
void BSP_LCD_SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue)
{
  /* Configure and Enable the color Keying for LCD Layer */
  HAL_LTDC_ConfigColorKeying(&(hltdc_eval), RGBValue, LayerIndex);
  HAL_LTDC_EnableColorKeying(&(hltdc_eval), LayerIndex);
}

/**
  * @brief  Disables the color keying.
  * @param  LayerIndex: Layer foreground or background
  */
void BSP_LCD_ResetColorKeying(uint32_t LayerIndex)
{
  /* Disable the color Keying for LCD Layer */
  HAL_LTDC_DisableColorKeying(&(hltdc_eval), LayerIndex);
}

/**
  * @brief  Sets the LCD text color.
  * @param  Color: Text color code ARGB(8-8-8-8)
  */
void BSP_LCD_SetTextColor(uint32_t Color)
{
  DrawProp[ActiveLayer].TextColor = Color;
}

/**
  * @brief  Gets the LCD text color.
  * @retval Used text color.
  */
uint32_t BSP_LCD_GetTextColor(void)
{
  return DrawProp[ActiveLayer].TextColor;
}

/**
  * @brief  Sets the LCD background color.
  * @param  Color: Layer background color code ARGB(8-8-8-8)
  */
void BSP_LCD_SetBackColor(uint32_t Color)
{
  DrawProp[ActiveLayer].BackColor = Color;
}

/**
  * @brief  Gets the LCD background color.
  * @retval Used background color
  */
uint32_t BSP_LCD_GetBackColor(void)
{
  return DrawProp[ActiveLayer].BackColor;
}

/**
  * @brief  Sets the LCD text font.
  * @param  fonts: Layer font to be used
  */
void BSP_LCD_SetFont(sFONT *fonts)
{
  DrawProp[ActiveLayer].pFont = fonts;
}

/**
  * @brief  Gets the LCD text font.
  * @retval Used layer font
  */
sFONT *BSP_LCD_GetFont(void)
{
  return DrawProp[ActiveLayer].pFont;
}

uint32_t *BSP_LCD_GetFBStartAdress(void)
{
	return (uint32_t*) (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress);
}

/**
  * @brief  Reads an LCD pixel.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @retval RGB pixel color
  */
uint32_t BSP_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  uint32_t ret = 0;

  if(hltdc_eval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint32_t*) (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress + (4*(Ypos*BSP_LCD_GetXSize() + Xpos)));
  }
  else if(hltdc_eval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
  {
    /* Read data value from SDRAM memory */
    ret = (*(__IO uint32_t*) (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress + (4*(Ypos*BSP_LCD_GetXSize() + Xpos))) & 0x00FFFFFF);
  }
  else if((hltdc_eval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565) || \
          (hltdc_eval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
          (hltdc_eval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_AL88))
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint16_t*) (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos)));
  }
  else
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint8_t*) (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos)));
  }

  return ret;
}

/**
  * @brief  Clears the whole currently active layer of LTDC.
  * @param  Color: Color of the background
  */
void BSP_LCD_Clear(uint32_t Color)
{
  /* Clear the LCD */
  LL_FillBuffer(ActiveLayer, (uint32_t *)(hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress), BSP_LCD_GetXSize(), BSP_LCD_GetYSize(), 0, Color);
}

/**
  * @brief  Clears the selected line in currently active layer.
  * @param  Line: Line to be cleared
  */
void BSP_LCD_ClearStringLine(uint32_t Line)
{
  uint32_t color_backup = DrawProp[ActiveLayer].TextColor;
  DrawProp[ActiveLayer].TextColor = DrawProp[ActiveLayer].BackColor;

  /* Draw rectangle with background color */
  BSP_LCD_FillRect(0, (Line * DrawProp[ActiveLayer].pFont->Height), BSP_LCD_GetXSize(), DrawProp[ActiveLayer].pFont->Height);

  DrawProp[ActiveLayer].TextColor = color_backup;
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Displays one character in currently active layer.
  * @param  Xpos: Start column address
  * @param  Ypos: Line where to display the character shape.
  * @param  Ascii: Character ascii code
  *           This parameter must be a number between Min_Data = 0x20 and Max_Data = 0x7E
  */
void BSP_LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii)
{
  DrawChar(Xpos, Ypos, &DrawProp[ActiveLayer].pFont->table[(Ascii-' ') *\
    DrawProp[ActiveLayer].pFont->Height * ((DrawProp[ActiveLayer].pFont->Width + 7) / 8)]);
}

/**
  * @brief  Displays characters in currently active layer.
  * @param  Xpos: X position (in pixel)
  * @param  Ypos: Y position (in pixel)
  * @param  Text: Pointer to string to display on LCD
  * @param  Mode: Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  LEFT_MODE
  */
void BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode)
{
  uint16_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t  *ptr = Text;

  /* Get the text size */
  while (*ptr++) size ++ ;

  /* Characters number per line */
  xsize = (BSP_LCD_GetXSize()/DrawProp[ActiveLayer].pFont->Width);

  switch (Mode)
  {
  case CENTER_MODE:
    {
      refcolumn = Xpos + ((xsize - size)* DrawProp[ActiveLayer].pFont->Width) / 2;
      break;
    }
  case LEFT_MODE:
    {
      refcolumn = Xpos;
      break;
    }
  case RIGHT_MODE:
    {
      refcolumn = - Xpos + ((xsize - size)*DrawProp[ActiveLayer].pFont->Width);
      break;
    }
  default:
    {
      refcolumn = Xpos;
      break;
    }
  }

  /* Check that the Start column is located in the screen */
  if ((refcolumn < 1) || (refcolumn >= 0x8000))
  {
    refcolumn = 1;
  }

  /* Send the string character by character on LCD */
  while ((*Text != 0) & (((BSP_LCD_GetXSize() - (i*DrawProp[ActiveLayer].pFont->Width)) & 0xFFFF) >= DrawProp[ActiveLayer].pFont->Width))
  {
    /* Display one character on LCD */
    BSP_LCD_DisplayChar(refcolumn, Ypos, *Text);
    /* Decrement the column position by 16 */
    refcolumn += DrawProp[ActiveLayer].pFont->Width;

    /* Point on the next character */
    Text++;
    i++;
  }

}

/**
  * @brief  Displays a maximum of 60 characters on the LCD.
  * @param  Line: Line where to display the character shape
  * @param  ptr: Pointer to string to display on LCD
  */
void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr)
{
  BSP_LCD_DisplayStringAt(0, LINE(Line), ptr, LEFT_MODE);
}

/**
  * @brief  Draws an horizontal line in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  */
void BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
	if (Xpos >= BSP_LCD_GetXSize())
		return;
	if (Ypos >= BSP_LCD_GetYSize())
		return;
  uint32_t  Xaddress = 0;
  /* Get the line address */
  Xaddress = (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress) + 4*(BSP_LCD_GetXSize()*Ypos + Xpos);

  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, Length, 1, 0, DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws a vertical line in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  */
void BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
	if (Xpos >= BSP_LCD_GetXSize())
		return;
	if (Ypos >= BSP_LCD_GetYSize())
		return;
  uint32_t  Xaddress = 0;

  /* Get the line address */
  Xaddress = (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress) + 4*(BSP_LCD_GetXSize()*Ypos + Xpos);

  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, 1, Length, (BSP_LCD_GetXSize() - 1), DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws an uni-line (between two points) in currently active layer.
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  */
void BSP_LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */

  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    BSP_LCD_DrawPixel(x, y, DrawProp[ActiveLayer].TextColor);   /* Draw the current pixel */
    num += numadd;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}

void BSP_LCD_DrawLineOverlap(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint8_t aOverlap)
{
	int16_t tDeltaX, tDeltaY, tDeltaXTimes2, tDeltaYTimes2, tError, tStepX, tStepY;

	uint32_t aColor = BSP_LCD_GetTextColor();
	uint16_t XSize = BSP_LCD_GetXSize();
	uint16_t YSize = BSP_LCD_GetYSize();
	/*
	 * Clip to display size
	 */
	if (aXStart >= XSize) {
		aXStart = XSize - 1;
	}
	if (aXStart < 0) {
		aXStart = 0;
	}
	if (aXEnd >= XSize) {
		aXEnd = XSize - 1;
	}
	if (aXEnd < 0) {
		aXEnd = 0;
	}
	if (aYStart >= YSize) {
		aYStart = YSize - 1;
	}
	if (aYStart < 0) {
		aYStart = 0;
	}
	if (aYEnd >= YSize) {
		aYEnd = YSize - 1;
	}
	if (aYEnd < 0) {
		aYEnd = 0;
	}

//	if ((aXStart == aXEnd) || (aYStart == aYEnd)) {
//		//horizontal or vertical line -> fillRect() is faster
//		FillRect(aXStart, aYStart, abs(aXEnd-aXStart), abs(aYEnd-aYStart));
//	} else {
		//calculate direction
		tDeltaX = aXEnd - aXStart;
		tDeltaY = aYEnd - aYStart;
		if (tDeltaX < 0) {
			tDeltaX = -tDeltaX;
			tStepX = -1;
		} else {
			tStepX = +1;
		}
		if (tDeltaY < 0) {
			tDeltaY = -tDeltaY;
			tStepY = -1;
		} else {
			tStepY = +1;
		}
		tDeltaXTimes2 = tDeltaX << 1;
		tDeltaYTimes2 = tDeltaY << 1;
		//draw start pixel
		BSP_LCD_DrawPixel(aXStart, aYStart, aColor);
		if (tDeltaX > tDeltaY) {
			// start value represents a half step in Y direction
			tError = tDeltaYTimes2 - tDeltaX;
			while (aXStart != aXEnd) {
				// step in main direction
				aXStart += tStepX;
				if (tError >= 0) {
					if (aOverlap & LINE_OVERLAP_MAJOR) {
						// draw pixel in main direction before changing
						BSP_LCD_DrawPixel(aXStart, aYStart, aColor);
					}
					// change Y
					aYStart += tStepY;
					if (aOverlap & LINE_OVERLAP_MINOR) {
						// draw pixel in minor direction before changing
						BSP_LCD_DrawPixel(aXStart - tStepX, aYStart, aColor);
					}
					tError -= tDeltaXTimes2;
				}
				tError += tDeltaYTimes2;
				BSP_LCD_DrawPixel(aXStart, aYStart, aColor);
			}
		} else {
			tError = tDeltaXTimes2 - tDeltaY;
			while (aYStart != aYEnd) {
				aYStart += tStepY;
				if (tError >= 0) {
					if (aOverlap & LINE_OVERLAP_MAJOR) {
						// draw pixel in main direction before changing
						BSP_LCD_DrawPixel(aXStart, aYStart, aColor);
					}
					aXStart += tStepX;
					if (aOverlap & LINE_OVERLAP_MINOR) {
						// draw pixel in minor direction before changing
						BSP_LCD_DrawPixel(aXStart, aYStart - tStepY, aColor);
					}
					tError -= tDeltaYTimes2;
				}
				tError += tDeltaXTimes2;
				BSP_LCD_DrawPixel(aXStart, aYStart, aColor);
			}
		}
//	}
}

void BSP_LCD_DrawThickLine(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, uint16_t aThickness, uint8_t lineCaps)
{
	if ((aThickness >= aXStart) ||
		(aThickness >= aYStart) ||
		(aThickness >= aXEnd) ||
		(aThickness >= aYEnd)) {
		return;
	}
	// lineCaps = 0 perpendicular line ending
	// lineCaps = 1 round line ending
	int16_t i, tDeltaX, tDeltaY, tDeltaXTimes2, tDeltaYTimes2, tError, tStepX, tStepY;

	uint8_t aThicknessMode = LINE_THICKNESS_MIDDLE;

	uint16_t XSize = BSP_LCD_GetXSize();
	uint16_t YSize = BSP_LCD_GetYSize();

	// Optional round line caps
	if (lineCaps > 0) {
		BSP_LCD_FillCircle(aXStart, aYStart, aThickness/2);
		BSP_LCD_FillCircle(aXEnd, aYEnd, aThickness/2);
	}

	if (aThickness <= 1) {
		BSP_LCD_DrawLineOverlap(aXStart, aYStart, aXEnd, aYEnd, LINE_OVERLAP_NONE);
	}
	/*
	 * Clip to display size
	 */
	if (aXStart >= XSize) {
		aXStart = XSize - 1;
	}
	if (aXStart < 0) {
		aXStart = 0;
	}
	if (aXEnd >= XSize) {
		aXEnd = XSize - 1;
	}
	if (aXEnd < 0) {
		aXEnd = 0;
	}
	if (aYStart >= YSize) {
		aYStart = YSize - 1;
	}
	if (aYStart < 0) {
		aYStart = 0;
	}
	if (aYEnd >= YSize) {
		aYEnd = YSize - 1;
	}
	if (aYEnd < 0) {
		aYEnd = 0;
	}

	/**
	 * For coordinate system with 0.0 top left
	 * Swap X and Y delta and calculate clockwise (new delta X inverted)
	 * or counterclockwise (new delta Y inverted) rectangular direction.
	 * The right rectangular direction for LINE_OVERLAP_MAJOR toggles with each octant
	 */
	tDeltaY = aXEnd - aXStart;
	tDeltaX = aYEnd - aYStart;
	// mirror 4 quadrants to one and adjust deltas and stepping direction
	int tSwap = 1; // count effective mirroring
	if (tDeltaX < 0) {
		tDeltaX = -tDeltaX;
		tStepX = -1;
		tSwap = !tSwap;
	} else {
		tStepX = +1;
	}
	if (tDeltaY < 0) {
		tDeltaY = -tDeltaY;
		tStepY = -1;
		tSwap = !tSwap;
	} else {
		tStepY = +1;
	}
	tDeltaXTimes2 = tDeltaX << 1;
	tDeltaYTimes2 = tDeltaY << 1;
	int tOverlap;
	// adjust for right direction of thickness from line origin
	int tDrawStartAdjustCount = aThickness / 2;
	if (aThicknessMode == LINE_THICKNESS_DRAW_COUNTERCLOCKWISE) {
		tDrawStartAdjustCount = aThickness - 1;
	} else if (aThicknessMode == LINE_THICKNESS_DRAW_CLOCKWISE) {
		tDrawStartAdjustCount = 0;
	}

	// which octant are we now
	if (tDeltaX >= tDeltaY) {
		if (tSwap) {
			tDrawStartAdjustCount = (aThickness - 1) - tDrawStartAdjustCount;
			tStepY = -tStepY;
		} else {
			tStepX = -tStepX;
		}
		/*
		 * Vector for draw direction of lines is rectangular and counterclockwise to original line
		 * Therefore no pixel will be missed if LINE_OVERLAP_MAJOR is used
		 * on changing in minor rectangular direction
		 */
		// adjust draw start point
		tError = tDeltaYTimes2 - tDeltaX;
		for (i = tDrawStartAdjustCount; i > 0; i--) {
			// change X (main direction here)
			aXStart -= tStepX;
			aXEnd -= tStepX;
			if (tError >= 0) {
				// change Y
				aYStart -= tStepY;
				aYEnd -= tStepY;
				tError -= tDeltaXTimes2;
			}
			tError += tDeltaYTimes2;
		}
		//draw start line
		BSP_LCD_DrawLine(aXStart, aYStart, aXEnd, aYEnd);
		// draw aThickness lines
		tError = tDeltaYTimes2 - tDeltaX;
		for (i = aThickness; i > 1; i--) {
			// change X (main direction here)
			aXStart += tStepX;
			aXEnd += tStepX;
			tOverlap = LINE_OVERLAP_NONE;
			if (tError >= 0) {
				// change Y
				aYStart += tStepY;
				aYEnd += tStepY;
				tError -= tDeltaXTimes2;
				/*
				 * change in minor direction reverse to line (main) direction
				 * because of choosing the right (counter)clockwise draw vector
				 * use LINE_OVERLAP_MAJOR to fill all pixel
				 *
				 * EXAMPLE:
				 * 1,2 = Pixel of first lines
				 * 3 = Pixel of third line in normal line mode
				 * - = Pixel which will additionally be drawn in LINE_OVERLAP_MAJOR mode
				 *           33
				 *       3333-22
				 *   3333-222211
				 * 33-22221111
				 *  221111                     /\
				 *  11                          Main direction of draw vector
				 *  -> Line main direction
				 *  <- Minor direction of counterclockwise draw vector
				 */
				tOverlap = LINE_OVERLAP_MAJOR;
			}
			tError += tDeltaYTimes2;
			BSP_LCD_DrawLineOverlap(aXStart, aYStart, aXEnd, aYEnd, tOverlap);
		}
	} else {
		// the other octant
		if (tSwap) {
			tStepX = -tStepX;
		} else {
			tDrawStartAdjustCount = (aThickness - 1) - tDrawStartAdjustCount;
			tStepY = -tStepY;
		}
		// adjust draw start point
		tError = tDeltaXTimes2 - tDeltaY;
		for (i = tDrawStartAdjustCount; i > 0; i--) {
			aYStart -= tStepY;
			aYEnd -= tStepY;
			if (tError >= 0) {
				aXStart -= tStepX;
				aXEnd -= tStepX;
				tError -= tDeltaYTimes2;
			}
			tError += tDeltaXTimes2;
		}
		//draw start line
		BSP_LCD_DrawLine(aXStart, aYStart, aXEnd, aYEnd);
		tError = tDeltaXTimes2 - tDeltaY;
		for (i = aThickness; i > 1; i--) {
			aYStart += tStepY;
			aYEnd += tStepY;
			tOverlap = LINE_OVERLAP_NONE;
			if (tError >= 0) {
				aXStart += tStepX;
				aXEnd += tStepX;
				tError -= tDeltaYTimes2;
				tOverlap = LINE_OVERLAP_MAJOR;
			}
			tError += tDeltaXTimes2;
			BSP_LCD_DrawLineOverlap(aXStart, aYStart, aXEnd, aYEnd, tOverlap);
		}
	}
}

void xLine(uint16_t x1, uint16_t x2, uint16_t y, uint32_t colour)
{
    while (x1 <= x2) BSP_LCD_DrawPixel(x1++, y, colour);
}

void yLine(uint16_t x, uint16_t y1, uint16_t y2, uint32_t colour)
{
    while (y1 <= y2) BSP_LCD_DrawPixel(x, y1++, colour);
}

void circle2(uint16_t xc, uint16_t yc, uint16_t inner, uint16_t outer)
{
	uint16_t xo = outer;
	uint16_t xi = inner;
	uint16_t y = 0;
	uint16_t erro = 1 - xo;
	uint16_t erri = 1 - xi;

	uint32_t TextColor = BSP_LCD_GetTextColor();

    while(xo >= y) {
        xLine(xc + xi, xc + xo, yc + y,  TextColor);
        yLine(xc + y,  yc + xi, yc + xo, TextColor);
        xLine(xc - xo, xc - xi, yc + y,  TextColor);
        yLine(xc - y,  yc + xi, yc + xo, TextColor);
        xLine(xc - xo, xc - xi, yc - y,  TextColor);
        yLine(xc - y,  yc - xo, yc - xi, TextColor);
        xLine(xc + xi, xc + xo, yc - y,  TextColor);
        yLine(xc + y,  yc - xo, yc - xi, TextColor);

        y++;

        if (erro < 0) {
            erro += 2 * y + 1;
        } else {
            xo--;
            erro += 2 * (y - xo + 1);
        }

        if (y > inner) {
            xi = y;
        } else {
            if (erri < 0) {
                erri += 2 * y + 1;
            } else {
                xi--;
                erri += 2 * (y - xi + 1);
            }
        }
    }
}

void BSP_LCD_DrawThickCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t lineThickness)
{
	if ((Radius <= 1) || (Radius - (lineThickness/2) < 2)) {
		return;
	}
	Radius += (lineThickness/2);
	uint16_t Radius2 = Radius - lineThickness;

	BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);		// Line Draw Color
//	uint32_t TextColor = BSP_LCD_GetTextColor();				// Pixel Draw Color

	uint16_t xo[800];
	uint16_t xi[800];

	int32_t   D;    /* Decision Variable */
	uint32_t  CurX; /* Current X Value */
	uint32_t  CurY; /* Current Y Value */

	D = 3 - (Radius << 1);
	CurX = 0;
	CurY = Radius;

	int iterations = 0;

	// Calculate outer circle
	while (CurX <= CurY)
	{
		xo[CurX] = CurY;
		xo[CurY] = CurX;
		if (iterations < CurX)
			iterations = CurX;
		if (iterations < CurY)
			iterations = CurY;
		xi[CurX*2] = 0;
		xi[CurY*2] = 0;
		xi[CurX*2+1] = 0;
		xi[CurY*2+1] = 0;

		if (D < 0)
		{
			D += (CurX << 2) + 6;
		}
		else
		{
			D += ((CurX - CurY) << 2) + 10;
			CurY--;
		}
		CurX++;
	}

	D = 3 - (Radius2 << 1);
	CurX = 0;
	CurY = Radius2;

	// Calculate inner circle
	while (CurX <= CurY)
	{
		xi[CurX] = CurY;
		xi[CurY] = CurX;

		if (D < 0)
		{
			D += (CurX << 2) + 6;
		}
		else
		{
			D += ((CurX - CurY) << 2) + 10;
			CurY--;
		}
		CurX++;
	}

	// Draw horizontal lines
	for (int y = 0; y <= iterations; y++)
	{
		if (xi[y] != 0) {
			BSP_LCD_DrawHLine(Xpos + xi[y], Ypos - y, xo[y] - xi[y]);
			BSP_LCD_DrawHLine(Xpos - xo[y], Ypos - y, xo[y] - xi[y]);
			BSP_LCD_DrawHLine(Xpos + xi[y], Ypos + y, xo[y] - xi[y]);
			BSP_LCD_DrawHLine(Xpos - xo[y], Ypos + y, xo[y] - xi[y]);
		}
		else
		{
			BSP_LCD_DrawHLine(Xpos - xo[y], Ypos - y, xo[y] + xo[y]);
			BSP_LCD_DrawHLine(Xpos - xo[y], Ypos + y, xo[y] + xo[y]);
		}
//		HAL_Delay (10);
	}

//	// Good, may not be the fastest
//	// Does not draw overlapping pixels
//  	BSP_LCD_DrawHLine(Xpos - Radius + 1, Ypos, lineThickness);	// Left Middle
//  	BSP_LCD_DrawHLine(Xpos + Radius2, Ypos, lineThickness);		// Right Middle
//  	BSP_LCD_DrawVLine(Xpos, Ypos - Radius + 1, lineThickness);	// Top Middle
//  	BSP_LCD_DrawVLine(Xpos, Ypos + Radius2, lineThickness);		// Bottom middle
//  	uint32_t RR = Radius * Radius;
//  	uint32_t R2R2 = Radius2 * Radius2;
//	for(int16_t y = -Radius; y < 0; y++) {
//		for(int16_t x= -Radius; x < 0; x++) {
//			uint32_t r2 = x * x + y * y;
//			if ((r2 <= RR) && (r2 >= R2R2)) {
//				BSP_LCD_DrawPixel(Xpos+x, Ypos+y, TextColor);
//				BSP_LCD_DrawPixel(Xpos+x, Ypos-y, TextColor);
//				BSP_LCD_DrawPixel(Xpos-x, Ypos+y, TextColor);
//				BSP_LCD_DrawPixel(Xpos-x, Ypos-y, TextColor);
//			}
//		}
//	}
//
//	return;

}

void BSP_LCD_DrawThickArc(int Xpos, int Ypos, int Radius, int startAngle, int endAngle, int lineThickness)
{
	if ((Radius <= 1) || (Radius - (lineThickness/2) < 2)) {
		return;
	}
	Radius += (lineThickness/2);
	uint16_t r_out = Radius;
	uint16_t r_in = r_out - lineThickness;

	while (endAngle < startAngle) {
		endAngle += 360;
	}
	while (endAngle > 720) {
		endAngle -= 360;
	}
	while (startAngle < 0) {
		startAngle += 360;
	}
	uint16_t startAngleMem = startAngle;
	uint16_t endAngleMem = endAngle;

    int16_t deg_base;
	int16_t deg;
	BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);		// Line Draw Color
	uint32_t TextColor = BSP_LCD_GetTextColor();				// Pixel Draw Color

	int16_t x_start[4];
	int16_t x_end[4];
    deg = 270;	// BSP_LCD_FastAtan2(-r_out, 0);
    if ((0 >= startAngle) && (0 <= endAngle)) {
    	BSP_LCD_DrawVLine(Xpos, Ypos + r_in,  lineThickness);		// Bottom middle
    }
    if ((90 >= startAngle) && (90 <= endAngle)) {
    	BSP_LCD_DrawHLine(Xpos + r_in, Ypos,  lineThickness);		// Right Middle
    }
    if ((180 >= startAngle) && (180 <= endAngle)) {
    	BSP_LCD_DrawVLine(Xpos, Ypos - r_out + 1,  lineThickness);	// Top Middle
    }
    if ((270 >= startAngle) && (270 <= endAngle)) {
    	BSP_LCD_DrawHLine(Xpos - r_out + 1, Ypos, lineThickness);	// Left Middle
    }

    uint32_t r_out_sqr = r_out * r_out;
    uint32_t r_in_sqr = r_in * r_in;
    int16_t x;
    int16_t y;

	if (endAngle > 360) {
		endAngle = 360;
	}

    for(y = -r_out; y < 0; y++) {
        x_start[0] = LV_COORD_MIN;
        x_start[1] = LV_COORD_MIN;
        x_start[2] = LV_COORD_MIN;
        x_start[3] = LV_COORD_MIN;
        x_end[0] = LV_COORD_MIN;
        x_end[1] = LV_COORD_MIN;
        x_end[2] = LV_COORD_MIN;
        x_end[3] = LV_COORD_MIN;
    	for(x= -r_out; x < 0; x++) {
            uint32_t r_act_sqr = x * x + y * y;
            if(r_act_sqr > r_out_sqr)
            	continue;

            deg_base =  BSP_LCD_FastAtan2(x, y) - 180;

            // Top-Left
			deg = 180 + deg_base; //BSP_LCD_FastAtan2(x, y);
			if ((deg >= startAngle) && (deg <= endAngle)) {
				if(x_start[0] == LV_COORD_MIN)
					x_start[0] = x;
			}
			else if(x_start[0] != LV_COORD_MIN && x_end[0] == LV_COORD_MIN)
				x_end[0] = x - 1;

            // Bottom-Left
			deg = 360 - deg_base; //BSP_LCD_FastAtan2(x, -y);
			if ((deg >= startAngle) && (deg <= endAngle)) {
				if(x_start[1] == LV_COORD_MIN)
					x_start[1] = x;
			}
			else if(x_start[1] != LV_COORD_MIN && x_end[1] == LV_COORD_MIN)
				x_end[1] = x - 1;

            // Top-Right
			deg = 180 - deg_base; //BSP_LCD_FastAtan2(-x, y);
			if ((deg >= startAngle) && (deg <= endAngle)) {
				if(x_start[2] == LV_COORD_MIN)
					x_start[2] = x;
			}
			else if(x_start[2] != LV_COORD_MIN && x_end[2] == LV_COORD_MIN)
				x_end[2] = x - 1;

            // Bottom-Right
			deg = deg_base; //BSP_LCD_FastAtan2(-x, -y);
			if ((deg >= startAngle) && (deg <= endAngle)) {
				if(x_start[3] == LV_COORD_MIN)
					x_start[3] = x;
			}
			else if(x_start[3] != LV_COORD_MIN && x_end[3] == LV_COORD_MIN)
				x_end[3] = x - 1;

            if(r_act_sqr < r_in_sqr)
            	break;	/*No need to continue the iteration in x once we found the inner edge of the arc*/
        }

    	if(x_start[0] != LV_COORD_MIN) {
    		if(x_end[0] == LV_COORD_MIN)
    			x_end[0] = x - 1;
    		BSP_LCD_DrawHLine(Xpos+x_start[0], Ypos+y, x_end[0] - x_start[0] + 1);
        }

    	if(x_start[1] != LV_COORD_MIN) {
    		if(x_end[1] == LV_COORD_MIN)
    			x_end[1] = x - 1;
        	BSP_LCD_DrawHLine(Xpos+x_start[1], Ypos-y, x_end[1] - x_start[1] + 1);
    	}

    	if(x_start[2] != LV_COORD_MIN) {
    		if(x_end[2] == LV_COORD_MIN)
    			x_end[2] = x - 1;
    		BSP_LCD_DrawHLine(Xpos-x_end[2], Ypos+y, LV_MATH_ABS(x_end[2] - x_start[2]) + 1);
    	}

    	if(x_start[3] != LV_COORD_MIN) {
    		if(x_end[3] == LV_COORD_MIN)
    			x_end[3] = x - 1;
        	BSP_LCD_DrawHLine(Xpos-x_end[3], Ypos-y, LV_MATH_ABS(x_end[3] - x_start[3]) + 1);
    	}
    }

	if (endAngleMem <= 360) {
		return;
	}

	if ((360 >= startAngleMem) && (360 <= endAngleMem)) {
		BSP_LCD_DrawVLine(Xpos, Ypos + r_in,  lineThickness);		// Bottom middle
	}
	if ((450 >= startAngleMem) && (450 <= endAngleMem)) {
		BSP_LCD_DrawHLine(Xpos + r_in, Ypos,  lineThickness);		// Right Middle
	}
	if ((540 >= startAngleMem) && (540 <= endAngleMem)) {
		BSP_LCD_DrawVLine(Xpos, Ypos - r_out + 1,  lineThickness);	// Top Middle
	}
	if ((630 >= startAngleMem) && (630 <= endAngleMem)) {
		BSP_LCD_DrawHLine(Xpos - r_out + 1, Ypos, lineThickness);	// Left Middle
	}

	endAngle = endAngleMem - 360;
	startAngle = 0;

	for(y = -r_out; y < 0; y++) {
        x_start[0] = LV_COORD_MIN;
        x_start[1] = LV_COORD_MIN;
        x_start[2] = LV_COORD_MIN;
        x_start[3] = LV_COORD_MIN;
        x_end[0] = LV_COORD_MIN;
        x_end[1] = LV_COORD_MIN;
        x_end[2] = LV_COORD_MIN;
        x_end[3] = LV_COORD_MIN;
    	for(x= -r_out; x < 0; x++) {
            uint32_t r_act_sqr = x * x + y * y;
            if(r_act_sqr > r_out_sqr)
            	continue;

            deg_base =  BSP_LCD_FastAtan2(x, y) - 180;

            // Top-Left
			deg = 180 + deg_base; //BSP_LCD_FastAtan2(x, y);
			if ((deg >= startAngle) && (deg <= endAngle)) {
				if(x_start[0] == LV_COORD_MIN)
					x_start[0] = x;
			}
			else if(x_start[0] != LV_COORD_MIN && x_end[0] == LV_COORD_MIN)
				x_end[0] = x - 1;

            // Bottom-Left
			deg = 360 - deg_base; //BSP_LCD_FastAtan2(x, -y);
			if ((deg >= startAngle) && (deg <= endAngle)) {
				if(x_start[1] == LV_COORD_MIN)
					x_start[1] = x;
			}
			else if(x_start[1] != LV_COORD_MIN && x_end[1] == LV_COORD_MIN)
				x_end[1] = x - 1;

            // Top-Right
			deg = 180 - deg_base; //BSP_LCD_FastAtan2(-x, y);
			if ((deg >= startAngle) && (deg <= endAngle)) {
				if(x_start[2] == LV_COORD_MIN)
					x_start[2] = x;
			}
			else if(x_start[2] != LV_COORD_MIN && x_end[2] == LV_COORD_MIN)
				x_end[2] = x - 1;

            // Bottom-Right
			deg = deg_base; //BSP_LCD_FastAtan2(-x, -y);
			if ((deg >= startAngle) && (deg <= endAngle)) {
				if(x_start[3] == LV_COORD_MIN)
					x_start[3] = x;
			}
			else if(x_start[3] != LV_COORD_MIN && x_end[3] == LV_COORD_MIN)
				x_end[3] = x - 1;

            if(r_act_sqr < r_in_sqr)
            	break;	/*No need to continue the iteration in x once we found the inner edge of the arc*/
        }

    	if(x_start[0] != LV_COORD_MIN) {
    		if(x_end[0] == LV_COORD_MIN)
    			x_end[0] = x - 1;
    		BSP_LCD_DrawHLine(Xpos+x_start[0], Ypos+y, x_end[0] - x_start[0] + 1);
        }

    	if(x_start[1] != LV_COORD_MIN) {
    		if(x_end[1] == LV_COORD_MIN)
    			x_end[1] = x - 1;
        	BSP_LCD_DrawHLine(Xpos+x_start[1], Ypos-y, x_end[1] - x_start[1] + 1);
    	}

    	if(x_start[2] != LV_COORD_MIN) {
    		if(x_end[2] == LV_COORD_MIN)
    			x_end[2] = x - 1;
    		BSP_LCD_DrawHLine(Xpos-x_end[2], Ypos+y, LV_MATH_ABS(x_end[2] - x_start[2]) + 1);
    	}

    	if(x_start[3] != LV_COORD_MIN) {
    		if(x_end[3] == LV_COORD_MIN)
    			x_end[3] = x - 1;
        	BSP_LCD_DrawHLine(Xpos-x_end[3], Ypos-y, LV_MATH_ABS(x_end[3] - x_start[3]) + 1);
    	}
    }
}

void BSP_LCD_DrawThickArcOld(int Xpos, int Ypos, int Radius, int startAngle, int endAngle, int lineThickness)
{
	Radius += (lineThickness/2);
	uint16_t Radius2 = Radius - lineThickness;
	int16_t deg;
	BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);		// Line Draw Color
	uint32_t TextColor = BSP_LCD_GetTextColor();				// Pixel Draw Color

	// Good, may not be the fastest
	// Does not draw overlapping pixels
	deg = BSP_LCD_FastAtan2(-Radius, 0);
	if ((deg >= startAngle) && (deg <= endAngle))
		BSP_LCD_DrawHLine(Xpos - Radius + 1, Ypos, lineThickness);	// Left Middle
	deg = BSP_LCD_FastAtan2(Radius2, 0);
	if ((deg >= startAngle) && (deg <= endAngle))
		BSP_LCD_DrawHLine(Xpos + Radius2, Ypos, lineThickness);		// Right Middle
	deg = BSP_LCD_FastAtan2(0, -Radius);
	if ((deg >= startAngle) && (deg <= endAngle))
		BSP_LCD_DrawVLine(Xpos, Ypos - Radius + 1, lineThickness);	// Top Middle
	deg = BSP_LCD_FastAtan2(0, Radius2);
	if ((deg >= startAngle) && (deg <= endAngle))
		BSP_LCD_DrawVLine(Xpos, Ypos + Radius2, lineThickness);		// Bottom middle
	uint32_t RR = Radius * Radius;
	uint32_t R2R2 = Radius2 * Radius2;
	for(int16_t y = -Radius; y < 0; y++) {
		for(int16_t x= -Radius; x < 0; x++) {
			uint32_t r2 = x * x + y * y;
			if ((r2 <= RR) && (r2 >= R2R2)) {
				deg = BSP_LCD_FastAtan2(x, y);
				if ((deg >= startAngle) && (deg <= endAngle))
					BSP_LCD_DrawPixel(Xpos+x, Ypos+y, TextColor);
				deg = BSP_LCD_FastAtan2(x, -y);
				if ((deg >= startAngle) && (deg <= endAngle))
					BSP_LCD_DrawPixel(Xpos+x, Ypos-y, TextColor);
				deg = BSP_LCD_FastAtan2(-x, y);
				if ((deg >= startAngle) && (deg <= endAngle))
					BSP_LCD_DrawPixel(Xpos-x, Ypos+y, TextColor);
				deg = BSP_LCD_FastAtan2(-x, -y);
				if ((deg >= startAngle) && (deg <= endAngle))
					BSP_LCD_DrawPixel(Xpos-x, Ypos-y, TextColor);
			}
		}
	}
}

uint16_t BSP_LCD_FastAtan2(int x, int y)
{
	// http://www.java-gaming.org/index.php?topic=14647.0

	// Fast XY vector to integer degree algorithm - Jan 2011 www.RomanBlack.com
	// Converts any XY values including 0 to a degree value that should be
	// within +/- 1 degree of the accurate value without needing
	// large slow trig functions like ArcTan() or ArcCos().
	// NOTE! at least one of the X or Y values must be non-zero!
	// This is the full version, for all 4 quadrants and will generate
	// the angle in integer degrees from 0-360.
	// Any values of X and Y are usable including negative values provided
	// they are between -1456 and 1456 so the 16bit multiply does not overflow.

	unsigned char negflag;
	unsigned char tempdegree;
	unsigned char comp;
	unsigned int degree;     // this will hold the result
	//signed int x;            // these hold the XY vector at the start
	//signed int y;            // (and they will be destroyed)
	unsigned int ux;
	unsigned int uy;

	// Save the sign flags then remove signs and get XY as unsigned ints
	negflag = 0;
	if(x < 0)
	{
	   negflag += 0x01;    // x flag bit
	   x = (0 - x);        // is now +
	}
	ux = x;                // copy to unsigned var before multiply
	if(y < 0)
	{
	   negflag += 0x02;    // y flag bit
	   y = (0 - y);        // is now +
	}
	uy = y;                // copy to unsigned var before multiply

	// 1. Calc the scaled "degrees"
	if(ux > uy)
	{
	   degree = (uy * 45) / ux;   // degree result will be 0-45 range
	   negflag += 0x10;    // octant flag bit
	}
	else
	{
	   degree = (ux * 45) / uy;   // degree result will be 0-45 range
	}

	// 2. Compensate for the 4 degree error curve
	comp = 0;
	tempdegree = degree;    // use an unsigned char for speed!
	if(tempdegree > 22)      // if top half of range
	{
	   if(tempdegree <= 44) comp++;
	   if(tempdegree <= 41) comp++;
	   if(tempdegree <= 37) comp++;
	   if(tempdegree <= 32) comp++;  // max is 4 degrees compensated
	}
	else    // else is lower half of range
	{
	   if(tempdegree >= 2) comp++;
	   if(tempdegree >= 6) comp++;
	   if(tempdegree >= 10) comp++;
	   if(tempdegree >= 15) comp++;  // max is 4 degrees compensated
	}
	degree += comp;   // degree is now accurate to +/- 1 degree!

	// Invert degree if it was X>Y octant, makes 0-45 into 90-45
	if(negflag & 0x10) degree = (90 - degree);

	// 3. Degree is now 0-90 range for this quadrant,
	// need to invert it for whichever quadrant it was in
	if(negflag & 0x02)   // if -Y
	{
	   if(negflag & 0x01)   // if -Y -X
			 degree = (180 + degree);
	   else        // else is -Y +X
			 degree = (180 - degree);
	}
	else    // else is +Y
	{
	   if(negflag & 0x01)   // if +Y -X
			 degree = (360 - degree);
	}
	return degree;
}

void BSP_LCD_CreateAtan2Table(void)
{
//    for (int i = 0; i <= Size_Ac; i++) {
//        double d = (double) i / Size_Ac;
//        double x = 1;
//        double y = x * d;
//        float v = (float) atan2(y, x);
//        Atan2[i] = v;
//        Atan2_PM[i] = Pi - v;
//        Atan2_MP[i] = -v;
//        Atan2_MM[i] = -Pi + v;
//
//        Atan2_R[i] = Pi_H - v;
//        Atan2_RPM[i] = Pi_H + v;
//        Atan2_RMP[i] = -Pi_H + v;
//        Atan2_RMM[i] = -Pi_H - v;
//    }
}

/**
 * ATAN2
 */

int BSP_LCD_GetAtan2(int y, int x)
{
//    if (y < 0) {
//        if (x < 0) {
//            //(y < x) because == (-y > -x)
//            if (y < x) {
//                return Atan2_RMM[(int) (x / y * Size_Ac)];
//            } else {
//                return Atan2_MM[(int) (y / x * Size_Ac)];
//            }
//        } else {
//            y = -y;
//            if (y > x) {
//                return Atan2_RMP[(int) (x / y * Size_Ac)];
//            } else {
//                return Atan2_MP[(int) (y / x * Size_Ac)];
//            }
//        }
//    } else {
//        if (x < 0) {
//            x = -x;
//            if (y > x) {
//                return Atan2_RPM[(int) (x / y * Size_Ac)];
//            } else {
//                return Atan2_PM[(int) (y / x * Size_Ac)];
//            }
//        } else {
//            if (y > x) {
//                return Atan2_R[(int) (x / y * Size_Ac)];
//            } else {
//                return Atan2[(int) (y / x * Size_Ac)];
//            }
//        }
//    }
}

// draws a pixel on screen of given brightness
// 0<=brightness<=1. We can use your own library
// to draw on screen
void BSP_LCD_DrawPixelBrightness(uint16_t x1, uint16_t y1, float brightness)
{
	uint32_t TextColor = BSP_LCD_GetTextColor();
	uint32_t BackColor = BSP_LCD_GetBackColor();
//	printf("TextColor 0x%08X", TextColor);
//	printf("BackColor 0x%08X", BackColor);

	uint8_t TextColorR = (TextColor & 0x00ff0000) >> 16;
	uint8_t TextColorG = (TextColor & 0x0000ff00) >> 8;
	uint8_t TextColorB = (TextColor & 0x000000ff);
	uint8_t BackColorR = (BackColor & 0x00ff0000) >> 16;
	uint8_t BackColorG = (BackColor & 0x0000ff00) >> 8;
	uint8_t BackColorB = (BackColor & 0x000000ff);
	float R = (float)TextColorR*brightness + (float)BackColorR*(1.0-brightness);
	float G = (float)TextColorG*brightness + (float)BackColorG*(1.0-brightness);
	float B = (float)TextColorB*brightness + (float)BackColorB*(1.0-brightness);
	uint8_t pR = (uint8_t)R;
	uint8_t pG = (uint8_t)G;
	uint8_t pB = (uint8_t)B;
	uint32_t pixelColor = 0xff000000 + (pR << 16) + (pG << 8) + pB;
	BSP_LCD_DrawPixel(x1, y1, pixelColor);   /* Draw the current pixel */
}

// swaps two numbers
void swap(int *a , int *b)
{
    int temp = *a;
    *a = *b;
    *b = temp;
}

//returns integer part of a floating point number
int iPartOfNumber(float x)
{
    return (int)x;
}

//returns fractional part of a number
float fPartOfNumber(float x)
{
    if (x>0) return x - (int)x;
    else return x - ((int)x+1);

}

//returns 1 - fractional part of number
float rfPartOfNumber(float x)
{
    return 1 - fPartOfNumber(x);
}

void BSP_LCD_DrawLineAA(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    int steep = abs(y2 - y1) > abs(x2 - x1);

    // swap the co-ordinates if slope > 1 or we
    // draw backwards
    if (steep)
    {
        swap(&x1 , &y1);
        swap(&x2 , &y2);
    }
    if (x1 > x2)
    {
        swap(&x1 ,&x2);
        swap(&y1 ,&y2);
    }

    //compute the slope
    float dx = x2-x1;
    float dy = y2-y1;
    float gradient = dy/dx;
    if (dx == 0.0)
        gradient = 1;

    int xpxl1 = x1;
    int xpxl2 = x2;
    float intersectY = y1;

    // main loop
    if (steep)
    {
        for (uint16_t x = xpxl1 ; x <= xpxl2 ; x++)
        {
            // pixel coverage is determined by fractional
            // part of y co-ordinate
        	BSP_LCD_DrawPixelBrightness(iPartOfNumber(intersectY), x,
                        1-rfPartOfNumber(intersectY));
        	BSP_LCD_DrawPixelBrightness(iPartOfNumber(intersectY)-1, x,
                        1-fPartOfNumber(intersectY));
            intersectY += gradient;
        }
    }
    else
    {
        for (uint16_t x = xpxl1 ; x <=xpxl2 ; x++)
        {
            // pixel coverage is determined by fractional
            // part of y co-ordinate
        	BSP_LCD_DrawPixelBrightness(x, iPartOfNumber(intersectY),
                        1-rfPartOfNumber(intersectY));
        	BSP_LCD_DrawPixelBrightness(x, iPartOfNumber(intersectY)-1,
                          1-fPartOfNumber(intersectY));
            intersectY += gradient;
        }
    }

//	// Fails
//	uint16_t dx = abs(x2-x1), sx = x1<x2 ? 1 : -1;
//	uint16_t dy = abs(y2-y1), sy = y1<y2 ? 1 : -1;
//	uint16_t err = dx-dy, e2, x3;                       /* error value e_xy */
//	uint16_t ed = dx+dy == 0 ? 1 : sqrt((float)dx*dx+(float)dy*dy);
//
//   uint32_t R = (DrawProp[ActiveLayer].TextColor & 0x00ff0000) >> 16;
//   uint32_t G = (DrawProp[ActiveLayer].TextColor & 0x0000ff00) >> 8;
//   uint32_t B = (DrawProp[ActiveLayer].TextColor & 0x000000ff);
//   for ( ; ; ){                                         /* pixel loop */
//	   uint32_t pR = R*abs(err-dx+dy)/ed;
//	   uint32_t pG = G*abs(err-dx+dy)/ed;
//	   uint32_t pB = B*abs(err-dx+dy)/ed;
//	   uint32_t pixelColor = 0xff000000 + (pR << 16) + (pG << 8) + pB;
//      BSP_LCD_DrawPixel(x1,y1, pixelColor);   /* Draw the current pixel */
////      setPixelAA(x0,y0, 255*abs(err-dx+dy)/ed);
//      e2 = err;
//      x3 = x1;
//      if (2*e2 >= -dx) {                                    /* x step */
//         if (x1 == x2)
//        	 break;
//         if (e2+dy < ed) {
//			pR = R*(e2+dy)/ed;
//			pG = G*(e2+dy)/ed;
//			pB = B*(e2+dy)/ed;
//			pixelColor = 0xff000000 + (pR << 16) + (pG << 8) + pB;
//             BSP_LCD_DrawPixel(x1,y1+sy, pixelColor);   /* Draw the current pixel */
////        	 setPixelAA(x0,y0+sy, 255*(e2+dy)/ed);
//         }
//         err -= dy;
//         x1 += sx;
//      }
//      if (2*e2 <= dy) {                                     /* y step */
//         if (y1 == y2)
//        	 break;
//         if (dx-e2 < ed) {
// 			pR = R*(dx-e2)/ed;
// 			pG = G*(dx-e2)/ed;
// 			pB = B*(dx-e2)/ed;
// 			uint32_t pixelColor = 0xff000000 + (pR << 16) + (pG << 8) + pB;
//             BSP_LCD_DrawPixel(x3+sx,y1, pixelColor);   /* Draw the current pixel */
////        	 setPixelAA(x2+sx,y0, 255*(dx-e2)/ed);
//         }
//         err += dx;
//         y1 += sy;
//    }
//  }
}

/**
  * @brief  Draws an uni-line (between two points) in currently active layer.
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @param  width: Width of line
  */
void BSP_LCD_DrawLineThick(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t width)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */

  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
	BSP_LCD_FillCircle(x, y, width);   	      /* Draw the current pixel */
    num += numadd;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}


/**
  * @brief  Draws an uni-line (between two points) in currently active layer.
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @param  width: Width of line
  */
void BSP_LCD_DrawLineThickPoly(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, float thickness)
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
}

/**
  * @brief  Draws a rectangle in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width
  * @param  Height: Rectangle height
  */
void BSP_LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Draw horizontal lines */
  BSP_LCD_DrawHLine(Xpos, Ypos, Width);
  BSP_LCD_DrawHLine(Xpos, (Ypos+ Height), Width);

  /* Draw vertical lines */
  BSP_LCD_DrawVLine(Xpos, Ypos, Height);
  BSP_LCD_DrawVLine((Xpos + Width), Ypos, Height);
}

/**
  * @brief  Draws a circle in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  */
void BSP_LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t   D;    /* Decision Variable */
  uint32_t  CurX; /* Current X Value */
  uint32_t  CurY; /* Current Y Value */

  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;

  while (CurX <= CurY)
  {
    BSP_LCD_DrawPixel((Xpos + CurX), (Ypos - CurY), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos - CurX), (Ypos - CurY), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos + CurY), (Ypos - CurX), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos - CurY), (Ypos - CurX), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos + CurX), (Ypos + CurY), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos - CurX), (Ypos + CurY), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos + CurY), (Ypos + CurX), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos - CurY), (Ypos + CurX), DrawProp[ActiveLayer].TextColor);

    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}

/**
  * @brief  Draws an poly-line (between many points) in currently active layer.
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  */
void BSP_LCD_DrawPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0;

  if(PointCount < 2)
  {
    return;
  }

  BSP_LCD_DrawLine(Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y);

  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    BSP_LCD_DrawLine(X, Y, Points->X, Points->Y);
  }
}

/**
  * @brief  Draws an ellipse on LCD in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius
  */
void BSP_LCD_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  K = (float)(rad2/rad1);

  do {
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/K)), (Ypos+y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/K)), (Ypos+y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/K)), (Ypos-y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/K)), (Ypos-y), DrawProp[ActiveLayer].TextColor);

    e2 = err;
    if (e2 <= x) {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}

/**
  * @brief  Draws a bitmap picture loaded in the internal Flash (32 bpp) in currently active layer.
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pbmp: Pointer to Bmp picture address in the internal Flash
  */
void BSP_LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, width = 0, height = 0, bit_pixel = 0;
  uint32_t Address;
  uint32_t InputColorMode = 0;

  /* Get bitmap data address offset */
  index = *(__IO uint16_t *) (pbmp + 10);
  index |= (*(__IO uint16_t *) (pbmp + 12)) << 16;

  /* Read bitmap width */
  width = *(uint16_t *) (pbmp + 18);
  width |= (*(uint16_t *) (pbmp + 20)) << 16;

  /* Read bitmap height */
  height = *(uint16_t *) (pbmp + 22);
  height |= (*(uint16_t *) (pbmp + 24)) << 16;

  /* Read bit/pixel */
  bit_pixel = *(uint16_t *) (pbmp + 28);

  /* Set the address */
  Address = hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress + (((BSP_LCD_GetXSize()*Ypos) + Xpos)*(4));

  /* Get the layer pixel format */
  if ((bit_pixel/8) == 4)
  {
    InputColorMode = CM_ARGB8888;
  }
  else if ((bit_pixel/8) == 2)
  {
    InputColorMode = CM_RGB565;
  }
  else
  {
    InputColorMode = CM_RGB888;
  }

  /* Bypass the bitmap header */
  pbmp += (index + (width * (height - 1) * (bit_pixel/8)));

  /* Convert picture to ARGB8888 pixel format */
  for(index=0; index < height; index++)
  {
    /* Pixel format conversion */
    LL_ConvertLineToARGB8888((uint32_t *)pbmp, (uint32_t *)Address, width, InputColorMode);

    /* Increment the source and destination buffers */
    Address+=  (BSP_LCD_GetXSize()*4);
    pbmp -= width*(bit_pixel/8);
  }
}

/**
  * @brief  Draws a full rectangle in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width
  * @param  Height: Rectangle height
  */
void BSP_LCD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint32_t  Xaddress = 0;

  /* Set the text color */
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);

  /* Get the rectangle start address */
  Xaddress = (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress) + 4*(BSP_LCD_GetXSize()*Ypos + Xpos);

  /* Fill the rectangle */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, Width, Height, (BSP_LCD_GetXSize() - Width), DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws a full circle in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  */
void BSP_LCD_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;     /* Decision Variable */
  uint32_t  CurX; /* Current X Value */
  uint32_t  CurY; /* Current Y Value */

  D = 3 - (Radius << 1);

  CurX = 0;
  CurY = Radius;

  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);

  while (CurX <= CurY)
  {
    if(CurY > 0)
    {
      BSP_LCD_DrawHLine(Xpos - CurY, Ypos + CurX, 2*CurY);
      BSP_LCD_DrawHLine(Xpos - CurY, Ypos - CurX, 2*CurY);
    }

    if(CurX > 0)
    {
      BSP_LCD_DrawHLine(Xpos - CurX, Ypos - CurY, 2*CurX);
      BSP_LCD_DrawHLine(Xpos - CurX, Ypos + CurY, 2*CurX);
    }
    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
  BSP_LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Draws a full poly-line (between many points) in currently active layer.
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  */
void BSP_LCD_FillPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0, X2 = 0, Y2 = 0, X_center = 0, Y_center = 0, X_first = 0, Y_first = 0, pixelX = 0, pixelY = 0, counter = 0;
  uint16_t  IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;

  IMAGE_LEFT = IMAGE_RIGHT = Points->X;
  IMAGE_TOP= IMAGE_BOTTOM = Points->Y;

  for(counter = 1; counter < PointCount; counter++)
  {
    pixelX = POLY_X(counter);
    if(pixelX < IMAGE_LEFT)
    {
      IMAGE_LEFT = pixelX;
    }
    if(pixelX > IMAGE_RIGHT)
    {
      IMAGE_RIGHT = pixelX;
    }

    pixelY = POLY_Y(counter);
    if(pixelY < IMAGE_TOP)
    {
      IMAGE_TOP = pixelY;
    }
    if(pixelY > IMAGE_BOTTOM)
    {
      IMAGE_BOTTOM = pixelY;
    }
  }

  if(PointCount < 2)
  {
    return;
  }

  X_center = (IMAGE_LEFT + IMAGE_RIGHT)/2;
  Y_center = (IMAGE_BOTTOM + IMAGE_TOP)/2;

  X_first = Points->X;
  Y_first = Points->Y;

  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    X2 = Points->X;
    Y2 = Points->Y;

    FillTriangle(X, X2, X_center, Y, Y2, Y_center);
    FillTriangle(X, X_center, X2, Y, Y_center, Y2);
    FillTriangle(X_center, X2, X, Y_center, Y2, Y);
  }

  FillTriangle(X_first, X2, X_center, Y_first, Y2, Y_center);
  FillTriangle(X_first, X_center, X2, Y_first, Y_center, Y2);
  FillTriangle(X_center, X2, X_first, Y_center, Y2, Y_first);
}

/**
  * @brief  Draws a full ellipse in currently active layer.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius
  */
void BSP_LCD_FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  K = (float)(rad2/rad1);

  do
  {
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/K)), (Ypos+y), (2*(uint16_t)(x/K) + 1));
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/K)), (Ypos-y), (2*(uint16_t)(x/K) + 1));

    e2 = err;
    if (e2 <= x)
    {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}

/**
  * @brief  Switch back on the display if was switched off by previous call of BSP_LCD_DisplayOff().
  *         Exit DSI ULPM mode if was allowed and configured in Dsi Configuration.
  */
void BSP_LCD_DisplayOn(void)
{
  /* Send Display on DCS command to display */
  HAL_DSI_ShortWrite(&(hdsi_eval),
                     hdsivideo_handle.VirtualChannelID,
                     DSI_DCS_SHORT_PKT_WRITE_P1,
                     OTM8009A_CMD_DISPON,
                     0x00);
  
}

/**
  * @brief  Switch Off the display.
  *         Enter DSI ULPM mode if was allowed and configured in Dsi Configuration.
  */
void BSP_LCD_DisplayOff(void)
{
  /* Send Display off DCS Command to display */
  HAL_DSI_ShortWrite(&(hdsi_eval),
                     hdsivideo_handle.VirtualChannelID,
                     DSI_DCS_SHORT_PKT_WRITE_P1,
                     OTM8009A_CMD_DISPOFF,
                     0x00);
  
}

/**
  * @brief  DCS or Generic short/long write command
  * @param  NbrParams: Number of parameters. It indicates the write command mode:
  *                 If inferior to 2, a long write command is performed else short.
  * @param  pParams: Pointer to parameter values table.
  * @retval HAL status
  */
void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t *pParams)
{
  if(NbrParams <= 1)
  {
   HAL_DSI_ShortWrite(&hdsi_eval, LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1, pParams[0], pParams[1]); 
  }
  else
  {
   HAL_DSI_LongWrite(&hdsi_eval,  LCD_OTM8009A_ID, DSI_DCS_LONG_PKT_WRITE, NbrParams, pParams[NbrParams], pParams); 
  }
}

/*******************************************************************************
                       LTDC, DMA2D and DSI BSP Routines
*******************************************************************************/
/**
  * @brief  Handles DMA2D interrupt request.
  * @note : Can be surcharged by application code implementation of the function.
  */
__weak void BSP_LCD_DMA2D_IRQHandler(void)
{
  HAL_DMA2D_IRQHandler(&hdma2d_eval);
}

/**
  * @brief  Handles DSI interrupt request.
  * @note : Can be surcharged by application code implementation of the function.
  */
__weak void BSP_LCD_DSI_IRQHandler(void)
{
  HAL_DSI_IRQHandler(&(hdsi_eval));
}


/**
  * @brief  Handles LTDC interrupt request.
  * @note : Can be surcharged by application code implementation of the function.
  */
__weak void BSP_LCD_LTDC_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&(hltdc_eval));
}

/**
  * @brief  De-Initializes the BSP LCD Msp
  * Application can surcharge if needed this function implementation.
  */
__weak void BSP_LCD_MspDeInit(void)
{
  /** @brief Disable IRQ of LTDC IP */
  HAL_NVIC_DisableIRQ(LTDC_IRQn);

  /** @brief Disable IRQ of DMA2D IP */
  HAL_NVIC_DisableIRQ(DMA2D_IRQn);

  /** @brief Disable IRQ of DSI IP */
  HAL_NVIC_DisableIRQ(DSI_IRQn);

  /** @brief Force and let in reset state LTDC, DMA2D and DSI Host + Wrapper IPs */
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DSI_FORCE_RESET();

  /** @brief Disable the LTDC, DMA2D and DSI Host and Wrapper clocks */
  __HAL_RCC_LTDC_CLK_DISABLE();
  __HAL_RCC_DMA2D_CLK_DISABLE();
  __HAL_RCC_DSI_CLK_DISABLE();
}

/**
  * @brief  Initialize the BSP LCD Msp.
  * Application can surcharge if needed this function implementation
  */
__weak void BSP_LCD_MspInit(void)
{
  /** @brief Enable the LTDC clock */
  __HAL_RCC_LTDC_CLK_ENABLE();

  /** @brief Toggle Sw reset of LTDC IP */
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_LTDC_RELEASE_RESET();

  /** @brief Enable the DMA2D clock */
  __HAL_RCC_DMA2D_CLK_ENABLE();

  /** @brief Toggle Sw reset of DMA2D IP */
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DMA2D_RELEASE_RESET();

  /** @brief Enable DSI Host and wrapper clocks */
  __HAL_RCC_DSI_CLK_ENABLE();

  /** @brief Soft Reset the DSI Host and wrapper */
  __HAL_RCC_DSI_FORCE_RESET();
  __HAL_RCC_DSI_RELEASE_RESET();

  /** @brief NVIC configuration for LTDC interrupt that is now enabled */
  HAL_NVIC_SetPriority(LTDC_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(LTDC_IRQn);

  /** @brief NVIC configuration for DMA2D interrupt that is now enabled */
  HAL_NVIC_SetPriority(DMA2D_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2D_IRQn);

  /** @brief NVIC configuration for DSI interrupt that is now enabled */
  HAL_NVIC_SetPriority(DSI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DSI_IRQn);
}

/**
  * @brief  This function handles LTDC Error interrupt Handler.
  * @note : Can be surcharged by application code implementation of the function.
  */

__weak void BSP_LCD_LTDC_ER_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&(hltdc_eval));
}


/**
  * @brief  Draws a pixel on LCD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  RGB_Code: Pixel color in ARGB mode (8-8-8-8)
  */
void BSP_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code)
{
  /* Write data value to all SDRAM memory */
  *(__IO uint32_t*) (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress + (4*(Ypos*BSP_LCD_GetXSize() + Xpos))) = RGB_Code;
}


/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: Line where to display the character shape
  * @param  Ypos: Start column address
  * @param  c: Pointer to the character data
  */
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c)
{
  uint32_t i = 0, j = 0;
  uint16_t height, width;
  uint8_t  offset;
  uint8_t  *pchar;
  uint32_t line;

  height = DrawProp[ActiveLayer].pFont->Height;
  width  = DrawProp[ActiveLayer].pFont->Width;

  offset =  8 *((width + 7)/8) -  width ;

  for(i = 0; i < height; i++)
  {
    pchar = ((uint8_t *)c + (width + 7)/8 * i);

    switch(((width + 7)/8))
    {

    case 1:
      line =  pchar[0];
      break;

    case 2:
      line =  (pchar[0]<< 8) | pchar[1];
      break;

    case 3:
    default:
      line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];
      break;
    }

    for (j = 0; j < width; j++)
    {
      if(line & (1 << (width- j + offset- 1)))
      {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp[ActiveLayer].TextColor);
      }
      else
      {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp[ActiveLayer].BackColor);
      }
    }
    Ypos++;
  }
}

/**
  * @brief  Fills a triangle (between 3 points).
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @param  x3: Point 3 X position
  * @param  y3: Point 3 Y position
  */
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */

  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    BSP_LCD_DrawLine(x, y, x3, y3);

    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  }
}

/**
  * @brief  Fills a buffer.
  * @param  LayerIndex: Layer index
  * @param  pDst: Pointer to destination buffer
  * @param  xSize: Buffer width
  * @param  ySize: Buffer height
  * @param  OffLine: Offset
  * @param  ColorIndex: Color index
  */
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex)
{
  /* Register to memory mode with ARGB8888 as color Mode */
  hdma2d_eval.Init.Mode         = DMA2D_R2M;
  hdma2d_eval.Init.ColorMode    = DMA2D_ARGB8888;
  hdma2d_eval.Init.OutputOffset = OffLine;

  hdma2d_eval.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d_eval) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d_eval, LayerIndex) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d_eval, ColorIndex, (uint32_t)pDst, xSize, ySize) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d_eval, 1000);
      }
    }
  }
}

/**
  * @brief  Converts a line to an ARGB8888 pixel format.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Output color
  * @param  xSize: Buffer width
  * @param  ColorMode: Input color mode
  */
static void LL_ConvertLineToARGB8888(void *pSrc, void *pDst, uint32_t xSize, uint32_t ColorMode)
{
  /* Configure the DMA2D Mode, Color Mode and output offset */
  hdma2d_eval.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d_eval.Init.ColorMode    = DMA2D_ARGB8888;
  hdma2d_eval.Init.OutputOffset = 0;

  /* Foreground Configuration */
  hdma2d_eval.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d_eval.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d_eval.LayerCfg[1].InputColorMode = ColorMode;
  hdma2d_eval.LayerCfg[1].InputOffset = 0;

  hdma2d_eval.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d_eval) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d_eval, 1) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d_eval, (uint32_t)pSrc, (uint32_t)pDst, xSize, 1) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d_eval, 10);
      }
    }
  }
}


/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
void BSP_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

//    NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
//    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn); 	// DMA IRQ Disable

	/* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
	hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
	hdma_memtomem_dma2_stream1.Init.Channel = DMA_CHANNEL_0;
	hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
	hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
	hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
	hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; /* Peripheral data alignment : 32bit */
	hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD; /* Peripheral data alignment : 32bit */
	hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
	hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;

//#ifndef POLLING_DMA
  /* Set the transfer complete interrupt callback function,
	 not sure it needs call to HAL_DMA_Init afterwards*/
//  hdma_memtomem_dma2_stream1.XferCpltCallback = XferCpltCallback;
//  hdma_memtomem_dma2_stream1.XferErrorCallback = XferErrorCallback;
//#endif
	if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
	{
		while(1);	// Halt on error
	}

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
//  NVIC_SetVector(DMA2_Stream1_IRQn, (uint32_t )my_irq_handler);// void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
//  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);	// IRQn_Type, uint32_t, uint32_t
//  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn); 	// DMA IRQ Enable
}


void BSP_PERFORM_DMA(uint32_t src, uint32_t dst, uint32_t size)
{
//	  uint32_t  Xaddress = 0;
//
//	  /* Get the line address */
//	  Xaddress = (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress) + 4*(BSP_LCD_GetXSize()*Ypos + Xpos);
//
//	  /* Write line */
//	  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, Length, 1, 0, DrawProp[ActiveLayer].TextColor);

	// Start the DMA transfer using polling mode
	if (HAL_DMA_Start(&hdma_memtomem_dma2_stream1,
			(uint32_t)src,
			(uint32_t)dst,
			size) != HAL_OK) {
		while(1);	// Halt on error
	}
	// Polling for transfer complete, if not using the interrupt
	HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream1,
							HAL_DMA_FULL_TRANSFER, 1000);
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
