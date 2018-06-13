#include "mbed.h"
#include "main.h"
#include "TS_DISCO_F469NI.h"
#include "LCD_DISCO_F469NI.h"
#include "bitmaps.h"
#include "lvgl/lvgl.h"
#include "PRNG.h"

#define DMA_INTERRUPT

LCD_DISCO_F469NI lcd;
TS_DISCO_F469NI ts;

Serial pc(USBTX, USBRX);

DigitalOut ledG(LED1, 1);
DigitalOut ledO(LED2, 1);
DigitalOut ledR(LED3, 1);
DigitalOut ledB(LED4, 1);
Ticker ticker1;

PRNG prng;

// DMA Stuff
// mbed-os/targets/TARGET_STM/TARGET_STM32L1/device/stm32l1xx_hal_dma.c
// mbed-os/targets/TARGET_STM/TARGET_STM32L1/device/stm32l1xx_hal_dma.h
// https://os.mbed.com/users/todotani/code/DMA_M2M/file/692bf16d1455/DMA_M2M.c/
// https://os.mbed.com/teams/ST/code/BSP_DISCO_F746NG/rev/458ab1edf6b2/
// https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f411RE/src/ch20/nucleo_hal_bsp.c
// /home/jacksoft/mbed/HAL/stm32f407vgt_disco/hal1/dma/dma/Src
// /home/jacksoft/mbed/DMA/main.c
// https://github.com/littlevgl/lv_boards/blob/master/stm32f429_discovery_no_os/hal_stm_lvgl/tft/tft.c#L191
// https://github.com/littlevgl/lv_boards/blob/master/stm32f429_discovery_no_os/hal_stm_lvgl/tft/tft.c#L952

/************************** PRIVATE DEFINTIONS*************************/
#define DMA_SIZE        32          /** DMA transfer size */
#define DMA_CYCLES      1000

#define GAUGE_MAX 500
#define GAUGE_CRITCAL GAUGE_MAX * 0.8

#define SDRAM_BANK_ADDR     ((uint32_t)0xc0000000)

//DMAScr_Buffer will be burn into flash when compile
static uint32_t DMASrc_Buffer[DMA_SIZE]=
{
    0x01020304,0x05060708,0x090A0B0C,0x0D0E0F10,
    0x11121314,0x15161718,0x191A1B1C,0x1D1E1F20,
    0x21222324,0x25262728,0x292A2B2C,0x2D2E2F30,
    0x31323334,0x35363738,0x393A3B3C,0x3D3E3F40,
    0x01020304,0x05060708,0x090A0B0C,0x0D0E0F10,
    0x11121314,0x15161718,0x191A1B1C,0x1D1E1F20,
    0x21222324,0x25262728,0x292A2B2C,0x2D2E2F30,
    0x31323334,0x35363738,0x393A3B3C,0x3D3E3F40
};

uint32_t DMADest_Buffer[DMA_SIZE];
//uint32_t *DMADest_Buffer = (uint32_t *)0x2007C000;   //LPC_AHBRAM0_BASE;

volatile uint32_t TC_count = 0;
uint32_t loop = 0;
//GPDMA_Channel_CFG_Type GPDMACfg;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;

static int32_t x1_flush;
static int32_t y1_flush;
static int32_t x2_flush;
static int32_t y2_fill;
static int32_t y_fill_act;
static const lv_color_t * buf_to_flush;
static __IO uint32_t * my_fb = (__IO uint32_t*) (SDRAM_BANK_ADDR);	// (__IO uint16_t*) (SDRAM_BANK_ADDR);

static TS_StateTypeDef state_;

int slider_value = 0;

//float slider_value_dest = 100;

lv_obj_t * gauge1 = NULL;
lv_obj_t * lmeter;
lv_obj_t * label1;
lv_chart_series_t * ser1;
lv_chart_series_t * ser2;
lv_obj_t * chart;
lv_obj_t *sw1;
lv_obj_t *sw2;

// Check touch detected
bool Touched()
{
    ts.GetState(&state_);
    if (!state_.touchDetected) return false;
    return true;
}

void Bitmap(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int bpp, unsigned char *bitmap)
{
    unsigned int i;
    unsigned int  j;
    int padd;
    unsigned short *bitmap_ptr = (unsigned short *)bitmap;

	uint32_t index = 0;
	if (bpp == 24) {
		// BBBBBBBB GGGGGGGG RRRRRRRR
		for (j = 0; j < h; j++) {         //Lines
			for (i = 0; i < w; i++) {     // one line
				index = (j * w + i) * 3;
				uint32_t color = 0xff000000;
				uint8_t b = bitmap[index];
				uint8_t g = bitmap[index + 1];
				uint8_t r = bitmap[index + 2];
//				index += 3;
				color += (r << 16);
				color += (g << 8);
				color += b;
				lcd.DrawPixel(x+i, y+h-j, color);
			}
//			// Padding for 4 byte alignment
//			if (w % 1) {
//				index++;
//			}
		}
	}
	if (bpp == 16) {
		// Color definitions for 64k color mode
		// Bits 0..4 -> Blue 0..4
		// Bits 5..10 -> Green 0..5
		// Bits 11..15 -> Red 0..4
		// index+1  index
		// 43210543 21043210
		// RRRRRGGG GGGBBBBB
		for (j = 0; j < h; j++) {         //Lines
			for (i = 0; i < w; i++) {     // one line
				index = (j * w + i) * 2;
				uint32_t color = 0xff000000;
				uint8_t b = ((bitmap[index] & 0x1f) << 3) |
						    ((bitmap[index] & 0x1f) >> 2); // 3 LSB's filled with MSB's
				uint8_t g = ((bitmap[index] & 0xe0) >> 3) |
						    ((bitmap[index + 1] & 0x07) << 5) |
							(bitmap[index + 1] & 0x07); // 2 LSB's filled with MSB's
				uint8_t r = (bitmap[index + 1] & 0xf8) |
						    (bitmap[index + 1] >> 5); // 3 LSB's filled with MSB's
//				index += 2;
				color += (r << 16);
				color += (g << 8);
				color += b;
				lcd.DrawPixel(x+i, y+h-j, color);
			}
//			// Padding for 4 byte alignment
//			if (w % 1) {
//				index++;
//			}
		}
	}
}

uint16_t GetBitmapXSize(sBITMAP *bitmap)
{
	return bitmap->Width;
}

uint16_t GetBitmapYSize(sBITMAP *bitmap)
{
	return bitmap->Height;
}

void SplashScreenBitmap(sBITMAP *bitmap)
{
    uint16_t SizeX = bitmap->Width;
    uint16_t SizeY = bitmap->Height;
    uint16_t BPP = bitmap->BitsPerPixel;
    uint16_t posX = (lcd.GetXSize() - SizeX) / 2;
    uint16_t posY = (lcd.GetYSize() - SizeY) / 2;
   	Bitmap(posX, posY-1, SizeX, SizeY, BPP, (unsigned char *) bitmap->table); // x, y, bitmap
}

void DrawBitmap(uint16_t posX, uint16_t posY, sBITMAP *bitmap)
{
    uint16_t SizeX = bitmap->Width;
    uint16_t SizeY = bitmap->Height;
    uint16_t BPP = bitmap->BitsPerPixel;
    Bitmap(posX, posY-1, SizeX, SizeY, BPP, (unsigned char *) bitmap->table); // x, y, bitmap
}

void doDMA(uint32_t src, uint32_t dst, uint32_t size)
{
	// Start the DMA transfer using polling mode
	if (HAL_DMA_Start(&hdma_memtomem_dma2_stream0,
			(uint32_t)src,
			(uint32_t)dst,
			size) != HAL_OK) {
		while(1);	// Halt on error
	}
	// Polling for transfer complete, if not using the interrupt
	HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0,
							HAL_DMA_FULL_TRANSFER, 1000);
}

/**
  * @brief  Updates just a region of the screen containing an object
  * @note   It configures DMA for memory to memory transfers
  * @retval None
  */
static void tft_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p)
{
	// Updates just a region of the screen containing an object
	// such as a button, meter, or chart, that needs to be updated.
	// In the flush function you can use DMA to do the flushing in the background.
	// Each transfer is a contiguous part of source and destination,
	// basically a horizontal line, so that an DMA transfer can handle it.
	// For every horizontal line in the rectangular region, a DMA transfer is used,
	// as we need to increment the address of the frame buffer by the
	// horizontal resolution of the LCD, which makes it a non-contiguous space.
	// For every new vertical position we first need to calculate the address
	// for the destination frame buffer.
	// Because color_p just only holds the data of the rectangular region,
	// it just needs to increment that pointer by the DMA size for every line.
	// Once the flushing is ready we have to call lv_flush_ready();

	/*Return if the area is out the screen*/
	if(x2 < 0) return;
	if(y2 < 0) return;
	if(x1 > TFT_HOR_RES - 1) return;
	if(y1 > TFT_VER_RES - 1) return;

	/*Truncate the area to the screen*/
	int32_t act_x1 = x1 < 0 ? 0 : x1;
	int32_t act_y1 = y1 < 0 ? 0 : y1;
	int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
	int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

	x1_flush = act_x1;
	y1_flush = act_y1;
	x2_flush = act_x2;
	y2_fill = act_y2;
	y_fill_act = act_y1;
	buf_to_flush = color_p;

#ifdef DMA_INTERRUPT

	/*##-7- Start the DMA transfer using the interrupt mode #*/
	/* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	/* Enable All the DMA interrupts */
	HAL_StatusTypeDef err;
	err = HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0,
			(uint32_t)buf_to_flush,
			(uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
			  (x2_flush - x1_flush + 1));
	if(err != HAL_OK)
	{
		while(1);	/*Halt on error*/
	}

#else

	// A faster than the pixel-by-pixel approach is to put all pixels to the
	// screen using DMA
//    uint32_t tftXSize = lcd.GetXSize();
//	static const uint32_t *ptrFBStart = lcd.GetFBStartAdress();
	uint32_t dmaSize = ((x2_flush-x1_flush) + 1);
//	uint32_t FBIndex = (uint32_t)&ptrFBStart[y1_flush * tftXSize + x1_flush];
    while(y1_flush <= y2_fill) {
    	uint32_t FBIndex = (uint32_t)&my_fb[y1_flush * TFT_HOR_RES + x1_flush];
//    	doDMA((uint32_t)color_p, (uint32_t)FBIndex, dmaSize);
		// Start the DMA transfer using polling mode
		if (HAL_DMA_Start(&hdma_memtomem_dma2_stream0,
//				(uint32_t)&color_p[(y1_flush - y1) * dmaSize],
				(uint32_t)color_p,
				(uint32_t)FBIndex,
				dmaSize) != HAL_OK)
		{
			while(1);	// Halt on error
		}
		// Polling for transfer complete, if not using the interrupt
		HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0,
								HAL_DMA_FULL_TRANSFER, 1000);
    	color_p += dmaSize;
    	y1_flush++;
//    	FBIndex += TFT_HOR_RES;
    }
    // Inform the graphics library that you are ready with the flushing,
    // by calling 'lv_fluh_ready()' when ready
    lv_flush_ready();

#endif


	// The most simple case (but also the slowest) to update the screen
    // pixel-by-pixel
//    for(int32_t y = y1; y <= y2; y++) {
//        for(int32_t x = x1; x <= x2; x++) {
//            // Put a pixel to the display.
//            // For example: put_px(x, y, *color_p)
//            lcd.DrawPixel(x, y, color_p->full);
//            color_p++;
//        }
//    }
//    // Inform the graphics library that you are ready with the flushing,
//    // by calling 'lv_fluh_ready()' when ready
//    lv_flush_ready();
}

/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void XferCpltCallback(DMA_HandleTypeDef *DmaHandle)
{
	if (DmaHandle->Instance == DMA2_Stream0)
	{
		y_fill_act++;

		if(y_fill_act > y2_fill) {
			lv_flush_ready();
		} else {
			buf_to_flush += x2_flush - x1_flush + 1;
			/*##-7- Start the DMA transfer using the interrupt mode ####################*/
			/* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
			/* Enable All the DMA interrupts */

			if(HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0,
			                    (uint32_t)buf_to_flush,
			                    (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
			                    (x2_flush - x1_flush + 1)) != HAL_OK)
			{
				while(1);	/*Halt on error*/
			}
		}
	}
}

/* Read the touchpad and store it in 'data'
 * REaturn false if no more data read; true for ready again */
static bool ex_tp_read(lv_indev_data_t *data)
{
    /* Read your touchpad */
    /* data->state = LV_INDEV_STATE_REL or LV_INDEV_STATE_PR */
    /* data->point.x = tp_x; */
    /* data->point.y = tp_y; */
    TS_StateTypeDef TS_State;
    uint16_t x, y;
    ts.GetState(&TS_State);
    if (TS_State.touchDetected)
    	data->state = LV_INDEV_STATE_PR;  // LV_INDEV_EVENT_PR
    else
    	data->state = LV_INDEV_STATE_REL; // LV_INDEV_EVENT_REL

    for (int idx = 0; idx < TS_State.touchDetected; idx++)
    {
    	data->point.x = TS_State.touchX[idx];
    	data->point.y = (uint16_t)((float)TS_State.touchY[idx] * 1.14); // Calibrated Y
    }

    /*No buffering so no more data read*/
    return false;   /*false: no more data to read because we are no buffering*/
}

static lv_res_t slider_action(lv_obj_t *slider)
{
	// https://littlevgl.com/basics
//	    printf("New slider value: %d\n", lv_slider_get_value(slider));
	int value = lv_slider_get_value(slider);

	slider_value = value;
    if (lv_sw_get_state(sw1))
    {
//		lv_gauge_set_value(gauge1, 0, slider_value);
		lv_lmeter_set_value(lmeter, slider_value);
		char buf[20];
		sprintf(buf, "%d%c", slider_value, '%');
		lv_label_set_text(label1, buf);
    }
    if (lv_sw_get_state(sw2))
    {
		ser2->points[3] = slider_value;
		lv_chart_refresh(chart); /*Required after direct set*/
    }
//    if ((lv_sw_get_state(sw1)) && (lv_sw_get_state(sw1)))
//    {
//		lv_lmeter_set_value(lmeter, slider_value);
//		char buf[20];
//		sprintf(buf, "%d%c", slider_value, '%');
//		lv_label_set_text(label1, buf);
//    }
    if (value >= 50)
		ledO = 0;
	else
		ledO = 1;
    return LV_RES_OK;
}


static lv_res_t my_click_action1(lv_obj_t *btn)
{
	// https://github.com/littlevgl/lvgl/issues/93
	// lv_list_create(lv_scr_act(), NULL)
//    lv_obj_t * label = lv_obj_get_child(btn, NULL); /*The label is the only child*/
//    lv_label_set_text(label, "Clicked");
	ledO = !ledO;
    return LV_RES_OK;   /*The button is not deleted*/
}

static lv_res_t my_click_action2(lv_obj_t *btn)
{
	// https://github.com/littlevgl/lvgl/issues/93
	// lv_list_create(lv_scr_act(), NULL)
//    lv_obj_t * label = lv_obj_get_child(btn, NULL); /*The label is the only child*/
//    lv_label_set_text(label, "Clicked");
	ledR = !ledR;
    return LV_RES_OK;   /*The button is not deleted*/
}

static lv_res_t sw1_action(lv_obj_t *sw1)
{
	ledO = !lv_sw_get_state(sw1);
    return LV_RES_OK;   /*The button is not deleted*/
}

static lv_res_t sw2_action(lv_obj_t *sw2)
{
	ledR = !lv_sw_get_state(sw2);
    return LV_RES_OK;   /*The button is not deleted*/
}


static void gif_task(void * img_obj)
{
	ledB = !ledB;
}



// DMA stuff
// mbed-os/targets/TARGET_STM/TARGET_STM32L1/device/stm32l1xx_hal_dma.c
// mbed-os/targets/TARGET_STM/TARGET_STM32L1/device/stm32l1xx_hal_dma.h
// https://os.mbed.com/users/todotani/code/DMA_M2M/file/692bf16d1455/DMA_M2M.c/
// https://os.mbed.com/teams/ST/code/BSP_DISCO_F746NG/rev/458ab1edf6b2/
// https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f411RE/src/ch20/nucleo_hal_bsp.c
// /home/jacksoft/mbed/HAL/stm32f407vgt_disco/hal1/dma/dma/Src
// /home/jacksoft/mbed/DMA/main.c
// https://github.com/littlevgl/lv_boards/blob/master/stm32f429_discovery_no_os/hal_stm_lvgl/tft/tft.c#L191
// https://github.com/littlevgl/lv_boards/blob/master/stm32f429_discovery_no_os/hal_stm_lvgl/tft/tft.c#L952

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Compare two buffers.
  * @param  pBuff1: buffer to be compared
  *         pBuff2: buffer to be compared
  *         len   : buffer length
  * @retval 0: pBuff1 identical to pBuff2
  *         1: pBuff1 differs from pBuff2
  */
uint8_t BufferCmp(uint32_t* pBuff1, uint32_t* pBuff2, uint16_t len)
{
	return memcmp(pBuff1, pBuff2, len);
//	while (len--)
//	{
//		if(*pBuff1 != *pBuff2)
//		{
//			return 1;
//		}
//		pBuff1++;
//		pBuff2++;
//	}
//	return 0;
}

/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void XferCpltCallback2(DMA_HandleTypeDef *DmaHandle)
{
//	if (DmaHandle->Instance == DMA2_Stream0)
//	{
//	y_fill_act ++;
//
//	if(y_fill_act > y2_fill) {
//		lv_flush_ready();
//	} else {
//		buf_to_flush += x2_flush - x1_flush + 1;
//		/*##-7- Start the DMA transfer using the interrupt mode ####################*/
//		/* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
//		/* Enable All the DMA interrupts */
//		if(HAL_DMA_Start_IT(hdma_memtomem_dma2_stream0,
//		                    (uint32_t)buf_to_flush,
//		                    (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
//		                    (x2_flush - x1_flush + 1)) != HAL_OK)
//		{
//			while(1);	/*Halt on error*/
//		}
//	}

//	}
//    lv_flush_ready();               /*Call 'lv_fluh_ready()' when ready*/

//	/* Compare the source and destination buffers */
//	if(memcmp(DMASrc_Buffer, DMADest_Buffer, DMA_SIZE))
//	{
//		/* Turn red LED on */
//		ledR = 0;
//		ledG = 1;
////		HAL_GPIO_WritePin(LED_O_GPIO_Port, LED_O_Pin, GPIO_PIN_SET);
//	}
//	else
//	{
//	/* Turn green LED on */
//		ledR = 1;
//		ledG = 0;
////		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
//	}
}

static void XferErrorCallback(DMA_HandleTypeDef *DmaHandle)
{
	if (DmaHandle->Instance == DMA2_Stream0 )
	{
		;
	}
	/* Transfer Error: Blink LED3 */
	while(1){
		ledO = !ledO;
//		HAL_GPIO_TogglePin (LED_O_GPIO_Port, LED_O_Pin);
		HAL_Delay (100);
	}
}

static uint32_t my_irq_handler(void){
//    printf("DMA2 HISR =%4x \n\r", DMA2->HISR);// Stream4 HIFR = 0 ?? DMA status is none?
//    lv_flush_ready();
//    lv_vdb_flush();
    HAL_DMA_IRQHandler(&hdma_memtomem_dma2_stream0);
    return 0;
}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn); 	// DMA IRQ Disable

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; /* Peripheral data alignment : 32bit */
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD; /* Peripheral data alignment : 32bit */
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;

//#ifndef POLLING_DMA
  /* Set the transfer complete interrupt callback function,
	 not sure it needs call to HAL_DMA_Init afterwards*/
  hdma_memtomem_dma2_stream0.XferCpltCallback = XferCpltCallback;
  hdma_memtomem_dma2_stream0.XferErrorCallback = XferErrorCallback;
//#endif
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetVector(DMA2_Stream0_IRQn, (uint32_t )my_irq_handler);// void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);	// IRQn_Type, uint32_t, uint32_t
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn); 	// DMA IRQ Enable
}

/**
* Fill a rectangular area with a color
* @param x1 left coordinate of the rectangle
* @param x2 right coordinate of the rectangle
* @param y1 top coordinate of the rectangle
* @param y2 bottom coordinate of the rectangle
* @param color fill color
*/
static void tft_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color)
{
    /*Return if the area is out the screen*/
    if(x2 < 0) return;
    if(y2 < 0) return;
    if(x1 > TFT_HOR_RES - 1) return;
    if(y1 > TFT_VER_RES - 1) return;

    /*Truncate the area to the screen*/
    int32_t act_x1 = x1 < 0 ? 0 : x1;
    int32_t act_y1 = y1 < 0 ? 0 : y1;
    int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
    int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

    uint32_t x;
    uint32_t y;

    /*Fill the remaining area*/
    lcd.SetTextColor((uint32_t) color.full);
    lcd.DrawRect((uint16_t) act_x1, (uint16_t) act_y1,
    		(uint16_t) act_x2 -  act_x1, (uint16_t) act_y2 - act_y1);

//    for(x = act_x1; x <= act_x2; x++) {
//        for(y = act_y1; y <= act_y2; y++) {
//            my_fb[y * TFT_HOR_RES + x] = color.full;
//        }
//    }
}

void fill_wrapper(const lv_area_t * coords, const lv_area_t * mask, lv_color_t color, lv_opa_t opa)
{
	/*`opa` and `mask` are don't care*/

    lcd.SetTextColor((uint32_t) color.full);
    lcd.DrawRect((uint16_t) coords->x1, (uint16_t) coords->y1,
    		(uint16_t) lv_area_get_width(coords), (uint16_t) lv_area_get_height(coords));
}

/**
 * Put a color map to a rectangular area
 * @param x1 left coordinate of the rectangle
 * @param x2 right coordinate of the rectangle
 * @param y1 top coordinate of the rectangle
 * @param y2 bottom coordinate of the rectangle
 * @param color_p pointer to an array of colors
 */
static void tft_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p)
{
	/*Return if the area is out the screen*/
	if(x2 < 0) return;
	if(y2 < 0) return;
	if(x1 > TFT_HOR_RES - 1) return;
	if(y1 > TFT_VER_RES - 1) return;

	/*Truncate the area to the screen*/
	int32_t act_x1 = x1 < 0 ? 0 : x1;
	int32_t act_y1 = y1 < 0 ? 0 : y1;
	int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
	int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

#if LV_VDB_DOUBLE == 0
	uint32_t y;
	for(y = act_y1; y <= act_y2; y++) {
		memcpy((void*)&my_fb[y * TFT_HOR_RES + act_x1],
				color_p,
				(act_x2 - act_x1 + 1) * sizeof(my_fb[0]));
		color_p += x2 - x1 + 1;    /*Skip the parts out of the screen*/
	}
#else

	x1_flush = act_x1;
	y1_flush = act_y1;
	x2_flush = act_x2;
	y2_fill = act_y2;
	y_fill_act = act_y1;
	buf_to_flush = color_p;


	  /*##-7- Start the DMA transfer using the interrupt mode #*/
	  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	  /* Enable All the DMA interrupts */
	  if(HAL_DMA_Start_IT(&DmaHandle,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
						  (x2_flush - x1_flush + 1)) != HAL_OK)
	  {
	    while(1)
	    {
	    }
	  }

#endif
}

void tick_1ms(void)
{
//	lv_tick_inc(1);
}

int main()
{
	char txt[60];
    pc.baud(BAUD_RATE);
    printf("\n");
    printf("-----------------------------------------\n");
    printf("Starting 201800502_DISCO-F469NI_LCD_Demo1\n");
    printf("" __DATE__ " " __TIME__ "\n");
#if defined(MBED_MAJOR_VERSION)
    printf("Using Mbed OS %d.%d.%d\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
#else
    printf("Using Mbed OS from master.\n");
#endif
    printf("CPU SystemCoreClock: %d MHz\n", SystemCoreClock/1000000);
    printf("-----------------------------------------\n");

//    wait(2);
	/*Start up indication*/
//	BSP_LED_Init(LED3);
	for (uint8_t i = 0; i < 50; i++) {
//		BSP_LED_Toggle(LED3);
		ledB = !ledB;
		HAL_Delay(20);
	}

    // DMA stuff
    // mbed-os/targets/TARGET_STM/TARGET_STM32L1/device/stm32l1xx_hal_dma.c
    // mbed-os/targets/TARGET_STM/TARGET_STM32L1/device/stm32l1xx_hal_dma.h
    // https://os.mbed.com/users/todotani/code/DMA_M2M/file/692bf16d1455/DMA_M2M.c/
    // https://os.mbed.com/teams/ST/code/BSP_DISCO_F746NG/rev/458ab1edf6b2/
    // https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f411RE/src/ch20/nucleo_hal_bsp.c
    // /home/jacksoft/mbed/HAL/stm32f407vgt_disco/hal1/dma/dma/Src
    // /home/jacksoft/mbed/DMA/main.c
    // https://github.com/littlevgl/lv_boards/blob/master/stm32f429_discovery_no_os/hal_stm_lvgl/tft/tft.c#L191
	// https://github.com/littlevgl/lv_boards/blob/master/stm32f429_discovery_no_os/hal_stm_lvgl/tft/tft.c#L952

    /* Disable GPDMA interrupt */
//    NVIC_DisableIRQ(DMA_IRQn);
    /* preemption = 1, sub-priority = 1 */
//    NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));

/*
(#) Enable and configure the peripheral to be connected to the DMA Stream
    (except for internal SRAM/FLASH memories: no initialization is
    necessary) please refer to Reference manual for connection between peripherals
    and DMA requests.

(#) For a given Stream, program the required configuration through the following parameters:
    Transfer Direction, Source and Destination data formats,
    Circular, Normal or peripheral flow control mode, Stream Priority level,
    Source and Destination Increment mode, FIFO mode and its Threshold (if needed),
    Burst mode for Source and/or Destination (if needed) using HAL_DMA_Init() function.

-@- Prior to HAL_DMA_Init() the clock must be enabled for DMA through the following macros:
    __HAL_RCC_DMA1_CLK_ENABLE() or __HAL_RCC_DMA2_CLK_ENABLE().

 *** Polling mode IO operation ***
 =================================
[..]
	  (+) Use HAL_DMA_Start() to start DMA transfer after the configuration of Source
		  address and destination address and the Length of data to be transferred.
	  (+) Use HAL_DMA_PollForTransfer() to poll for the end of current transfer, in this
		  case a fixed Timeout can be configured by User depending from his application.
	  (+) Use HAL_DMA_Abort() function to abort the current transfer.

 *** Interrupt mode IO operation ***
 ===================================
(+) Configure the DMA interrupt priority using HAL_NVIC_SetPriority()
(+) Enable the DMA IRQ handler using HAL_NVIC_EnableIRQ()
(+) Use HAL_DMA_Start_IT() to start DMA transfer after the configuration of
    Source address and destination address and the Length of data to be transferred.
    In this case the DMA interrupt is configured
(+) Use HAL_DMA_IRQHandler() called under DMA_IRQHandler() Interrupt subroutine
(+) At the end of data transfer HAL_DMA_IRQHandler() function is executed and user
    can add his own function by customization of function pointer XferCpltCallback
    and XferErrorCallback (i.e a member of DMA handle structure).
*/

    printf("MX_DMA_Init()\n");
    MX_DMA_Init();
    printf("Done.\n");

    // BSP_SDRAM_Init();
    // BSP_LCD_SetLayerAddress()
    // SetLayerAddress
    // HAL_LTDC_SetAddress()
    // void BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address)
    // Layercfg.FBStartAdress = FB_Address;
    // stm32469i_discovery_lcd.c, 661
    // uint32_t BSP_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos)
    // Read data value from SDRAM memory
    // ret = *(__IO uint32_t*) (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress + (4*(Ypos*BSP_LCD_GetXSize() + Xpos)));
//    uint32_t * BSP_LCD_GetFBStartAdress(void)
//    {
//    	uint32_t * ret = 0;
//        ret = (__IO uint32_t*) (hltdc_eval.LayerCfg[ActiveLayer].FBStartAdress);
//        return ret;
//    }
//    uint32_t *LCD_DISCO_F469NI::GetFBStartAdress(void)
//    {
//      return BSP_LCD_GetFBStartAdress();
//    }
//    lv_vdb_t *ptrSrc = lv_vdb_get();
//    uint32_t *ptrDest = lcd.GetFBStartAdress();



    printf("HAL_DMA_Start_IT(...)\n");
	/* Start the DMA transfer using polling mode */
	if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0, (uint32_t)&DMASrc_Buffer,
			(uint32_t)&DMADest_Buffer, DMA_SIZE) != HAL_OK)
	{
	    printf("Error.\n");
		/* Transfer Error: Blink Red LED for ever */
		while(1)
		{
//			HAL_GPIO_TogglePin (LED_R_GPIO_Port, LED_R_Pin);
		    printf("Error.\n");
			HAL_Delay (1000);
		}
	}
    printf("Done.\n");



//    printf("HAL_DMA_PollForTransfer(...)\n");
//	/* Polling for transfer complete, if not using XferCpltCallback interrupt */
//	HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0,
//	                        HAL_DMA_FULL_TRANSFER, 1000);
//    printf("Done.\n");
//
//    printf("BufferCmp(...)\n");
////	DMADest_Buffer[0] = 0xBEEFBEEF;
//	/* Compare the source and destination buffers */
//	// dstBuffer[4] = 0x12345678;	// Create difference
//	if(BufferCmp((uint32_t*)DMASrc_Buffer, (uint32_t*)DMADest_Buffer, DMA_SIZE))
//	{
//		/* Turn red LED on */
//		ledR = 1;
//	    printf("Red.\n");
//	}
//	else
//	{
//		/* Turn green LED on */
//		ledB = 1;
//	    printf("Green.\n");
//	}
//    printf("Done.\n");



//	HAL_GPIO_WritePin(LED_O_GPIO_Port, LED_O_Pin, GPIO_PIN_RESET);



	Timer t1;
	t1.reset();
	t1.start();


    SplashScreenBitmap(&VV_740x178x16); // bitmap

	printf("SplashScreenBitmap: %f\n", t1.read());

	t1.reset();

	wait(0.5);
	while (!Touched());

    // Testing
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(0xFFFFFFFF);

    lcd.CreateAtan2Table();

//	lcd.DrawThickLine(760, 40, 40, 440, 30, 0);
//	lcd.DrawThickLine(40, 40, 760, 440, 30, 1);
//	lcd.DrawThickCircle2(400, 240, 100, 130);

//	lcd.drawThickArc(400, 240, 100, 0, 360, 30);
//	lcd.drawThickArcOld(400, 240, 100, 45, 315, 30);
	lcd.DrawThickCircle(600, 240, 100, 30);
//	lcd.drawThickArc(400, 240, 100, 90, 270, 30);
	printf("Circle: %f\n", t1.read());

	t1.reset();
//	lcd.DrawThickCircle(600, 240, 100, 30);

//    lcd.SetTextColor(0xFFDC3912);	// Red
//	lcd.drawThickArc(400, 240, 100, 45, 105, 30);
//    lcd.SetTextColor(0xFFFF7B24);	// Orange
//	lcd.drawThickArc(400, 240, 100, 105, 165, 30);
//    lcd.SetTextColor(0xFFFBD109);	// Yellow
//	lcd.drawThickArc(400, 240, 100, 165, 225, 30);
//    lcd.SetTextColor(0xFF109618);	// Green
//	lcd.drawThickArc(400, 240, 100, 225, 315, 30);

	lcd.drawThickArc(200, 240, 100, 0, 360, 30);
	printf("Arc: %f\n", t1.read());

	wait(0.5);
	// Draw random Arcs
	uint32_t color;
	while (1)
	{
		uint16_t arc_radius = rand() % 100 + 60;
		uint16_t arc_x = rand() % 800;
		uint16_t arc_y = rand() % 480;
		uint16_t arc_s = rand() % 360;
		uint16_t arc_e = rand() % 360;
		uint16_t line_thickness = rand() % 25 + 5;
		color = 0xFF000000 + rand() % 0xFFFFFF;
	    lcd.SetTextColor(color);
		lcd.drawThickArc(arc_x, arc_y, arc_radius, arc_s, arc_e, line_thickness);

		if (Touched()) break;
	}

	printf("Arcs done.\n");
	wait(0.5);
	while (!Touched());

	wait(0.5);
	// Draw random Circles
	while (1)
	{
		uint16_t line_thickness = rand() % 25 + 5;
		uint16_t arc_radius = rand() % 100 + 20;
		uint16_t arc_x = rand() % 800;
		uint16_t arc_y = rand() % 480;
		color = 0xFF000000 + rand() % 0xFFFFFF;
	    lcd.SetTextColor(color);
		lcd.DrawThickCircle(arc_x, arc_y, arc_radius, line_thickness);

		if (Touched()) break;
	}

	printf("Circles done.\n");
	wait(0.5);
	while (!Touched());

	wait(0.5);
	// Draw random Lines
	while (1)
	{
		uint16_t aThickness = rand() % 25 + 5;
		uint16_t aXStart = rand() % 800;
		uint16_t aYStart = rand() % 480;
		uint16_t aXEnd = rand() % 800;
		uint16_t aYEnd = rand() % 480;
		color = 0xFF000000 + rand() % 0xFFFFFF;
	    lcd.SetTextColor(color);
	    uint8_t lineCaps = 0;
		lcd.DrawThickLine(aXStart, aYStart, aXEnd, aYEnd, aThickness, lineCaps);
//		printf("%d %d %d %d %d\n", aXStart, aYStart, aXEnd, aYEnd, aThickness);

		if (Touched()) break;
	}

	printf("Lines done.\n");
	wait(0.5);
	while (!Touched());


	ticker1.attach_us(&tick_1ms, 1000);	// 1mS






    TS_StateTypeDef TS_State;
    uint16_t x, y;
    uint8_t text[30];
    uint8_t status;
    uint8_t idx;
    uint8_t cleared = 0;
    uint8_t prev_nb_touches = 0;
  
//    my_fb = (__IO uint32_t*) lcd.GetFBStartAdress();
    printf("Framebuffer: 0x%08x\n", my_fb);
	// https://github.com/littlevgl/lv_examples/blob/master/lv_tutorial/0_porting/lv_tutorial_porting.c
	//lvgl_init();
	lv_init();




	lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
	lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/
	/*Set up the functions to access to your display*/
	disp_drv.disp_fill = tft_fill;
	disp_drv.disp_map = tft_map;
	disp_drv.disp_flush = tft_flush;            /*Used in buffered mode (LV_VDB_SIZE != 0  in lv_conf.h)*/
	/*Finally register the driver*/
	lv_disp_drv_register(&disp_drv);                /*Register the driver in LittlevGL*/

    /*************************
     * Input device interface
     *************************/
    /*Add a touchpad in the example*/
    /*touchpad_init();*/                            /*Initialize your touchpad*/
    lv_indev_drv_t indev_drv;                       /*Descriptor of an input device driver*/
    lv_indev_drv_init(&indev_drv);                  /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;         /*The touchpad is pointer type device*/
    indev_drv.read = ex_tp_read;                 /*Library ready your touchpad via this function*/
    lv_indev_drv_register(&indev_drv);              /*Finally register the driver*/

	// https://littlevgl.com/
    // https://littlevgl.com/porting
	/*Add a button*/
	lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);             /*Add to the active screen*/
	lv_obj_set_pos(btn1, 10, 10);                                    /*Adjust the position*/
	/*Add text*/
	lv_obj_t * label_btn1 = lv_label_create(btn1, NULL);                  /*Put on 'btn1'*/
	lv_label_set_text(label_btn1, "Orange");
	// https://littlevgl.com/object-types/button-lv_btn
	lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, my_click_action1);   /*Assign a callback for clicking*/

	/*Add a button*/
	lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), NULL);             /*Add to the active screen*/
	lv_obj_set_pos(btn2, 10, 80);                                    /*Adjust the position*/
	/*Add text*/
	lv_obj_t * label_btn2 = lv_label_create(btn2, NULL);                  /*Put on 'btn1'*/
	lv_label_set_text(label_btn2, "Red");
	// https://littlevgl.com/object-types/button-lv_btn
	lv_btn_set_action(btn2, LV_BTN_ACTION_CLICK, my_click_action2);   /*Assign a callback for clicking*/
//	lv_obj_align(btn1, btn2, LV_ALIGN_OUT_LEFT_MID, 0, +50);

	//	lv_btn_set_toggle(btn1, true);
//	lv_btn_set_state(btn1, LV_BTN_STATE_TGL_PR);
//	lv_btn_set_state(btn1, LV_BTN_STATE_TGL_REL);
//	lv_btn_set_state(btn1, LV_BTN_STATE_INA);
// lv_obj_set_hidden(my_list_1, true/false)




//	lv_vdb_t *ptrSrc = lv_vdb_get();	// lv_vdb_t
//	uint32_t *ptrDest = lcd.GetFBStartAdress();
//	uint32_t buffer_size = 800*480;
//	int r = 0;
//	int g = 0;
//	int b = 0;
//	for (int i = 0; i < buffer_size; i++)
//	{
//		ptrDest[i] = 0xff000000 + ((r & 0xff) << 16) +  ((g & 0xff) << 8) +  (b & 0xff);
//		r+=4;
//		if (r >= 256)
//		{
//		    r = 0;
//		    g+=4;
//		    if (g >= 256)
//		    {
//		    	g = 0;
//		    	b+=4;
//		    }
//		}
////		ptrDest++;
//	}
//    wait(5);
//    printf("HAL_DMA_Start_IT(...)\n");
//	/* Start the DMA transfer using polling mode */
//	if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0, (uint32_t)ptrSrc,
//			(uint32_t)ptrDest, buffer_size) != HAL_OK)
//	{
//	    printf("Error.\n");
//		/* Transfer Error: Blink Red LED for ever */
//		while(1)
//		{
////			HAL_GPIO_TogglePin (LED_R_GPIO_Port, LED_R_Pin);
//		    printf("Error.\n");
//			HAL_Delay (1000);
//		}
//	}
//    printf("Done.\n");
//
//    wait(5);




	/*Create a default slider*/
	lv_obj_t * slider1 = lv_slider_create(lv_scr_act(), NULL);
	lv_obj_set_size(slider1, 512, 35);
	lv_obj_align(slider1, NULL, LV_ALIGN_IN_TOP_RIGHT, -30, 30);
	lv_slider_set_action(slider1, slider_action);
	lv_slider_set_range(slider1, 0, 100);
	lv_bar_set_value(slider1, 40);


	/*Create a label right to the slider*/
	lv_obj_t * slider1_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(slider1_label, "Default");
	lv_obj_align(slider1_label, slider1, LV_ALIGN_OUT_LEFT_MID, -20, 0);

	/*Create a bar, an indicator and a knob style*/
	static lv_style_t style_bg;
	static lv_style_t style_indic;
	static lv_style_t style_knob;

	lv_style_copy(&style_bg, &lv_style_pretty);
	style_bg.body.main_color =  LV_COLOR_BLACK;
	style_bg.body.grad_color =  LV_COLOR_GRAY;
	style_bg.body.radius = LV_RADIUS_CIRCLE;
	style_bg.body.border.color = LV_COLOR_WHITE;

	lv_style_copy(&style_indic, &lv_style_pretty);
	style_indic.body.grad_color =  LV_COLOR_GREEN;
	style_indic.body.main_color =  LV_COLOR_LIME;
	style_indic.body.radius = LV_RADIUS_CIRCLE;
	style_indic.body.shadow.width = 10;
	style_indic.body.shadow.color = LV_COLOR_LIME;
	style_indic.body.padding.hor = 3;
	style_indic.body.padding.ver = 3;

	lv_style_copy(&style_knob, &lv_style_pretty);
	style_knob.body.radius = LV_RADIUS_CIRCLE;
	style_knob.body.opa = LV_OPA_70;
	style_knob.body.padding.ver = 10 ;


    /*Create an image object*/
    lv_obj_t * gif_img = lv_img_create(lv_scr_act(), NULL);
    lv_obj_set_pos(gif_img, 20, 180);

    /* Create a task which will change the images on 'gif_img'
     * The last parameter will be passed to the function */
    lv_task_create(gif_task, 500, LV_TASK_PRIO_MID, gif_img);



//    /*Create an array for the points of the line*/
//    static lv_point_t line_points[] = {{5, 5}, {70, 70}, {120, 10}, {180, 60}, {240, 10}};
//
//    /*Create line with default style*/
//    lv_obj_t * line1;
//    line1 = lv_line_create(lv_scr_act(), NULL);
//    lv_line_set_points(line1, line_points, 5);     /*Set the points*/
//    lv_obj_align(line1, NULL, LV_ALIGN_IN_TOP_MID, 0, 80);
//
//    /*Create new style (thin light blue)*/
//    static lv_style_t style_line2;
//    lv_style_copy(&style_line2, &lv_style_plain);
//    style_line2.line.color = LV_COLOR_MAKE(0x2e, 0x96, 0xff);
//    style_line2.line.color = LV_COLOR_HEX(0x2e96ff);
//    style_line2.line.width = 3;
//
//    /*Copy the previous line and apply the new style*/
//    lv_obj_t * line2 = lv_line_create(lv_scr_act(), line1);
//    lv_line_set_style(line2, &style_line2);
//    lv_obj_align(line2, line1, LV_ALIGN_OUT_BOTTOM_MID, 0, -35);
//
//    /*Create new style (thick dark blue)*/
//    static lv_style_t style_line3;
//    lv_style_copy(&style_line3, &lv_style_plain);
//    style_line3.line.color = LV_COLOR_MAKE(0x00, 0x3b, 0x75);
//    style_line3.line.width = 5;
//
//    /*Copy the previous line and apply the new style*/
//    lv_obj_t * line3 = lv_line_create(lv_scr_act(), line1);
//    lv_line_set_style(line3, &style_line3);
//    lv_obj_align(line3, line2, LV_ALIGN_OUT_BOTTOM_MID, 0, -35);





    /*******************************
     * Create 2 boxes
     *******************************/

    lv_obj_t * box1;
    box1 = lv_cont_create(lv_scr_act(), NULL);
    lv_obj_set_style(box1, &lv_style_pretty);
    lv_cont_set_fit(box1, true, true);

    /*Add a text to the container*/
    lv_obj_t * txt1 = lv_label_create(box1, NULL);

#ifdef DAMEN
    lv_label_set_text(txt1, "Good morning\n"
                           "Ron en Markus,\n"
                           "Hartelijk dank voor\n"
                           "de uitnodiging!\n"
                           "Met vriendelijke groet,\n"
                           "Jack");
#else
    lv_label_set_text(txt1, "Lorem ipsum dolor\n"
                           "sit amet, consectetur\n"
                           "adipiscing elit, sed do\n"
                           "eiusmod tempor incididunt\n"
                           "ut labore et dolore dolor\n"
                           "magna ipsum amet aliqua.");
#endif

    lv_obj_align(box1, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 160);      /*Align the container*/

    /*Create a style*/
    static lv_style_t style;
    lv_style_copy(&style, &lv_style_pretty_color);
    style.body.shadow.width = 8;
    style.body.padding.hor = 5;                         /*Set a great horizontal padding*/
    style.body.padding.ver = 5;                         /*Set a great vertical padding*/

    /*Create an other container*/
    lv_obj_t * box2;
    box2 = lv_cont_create(lv_scr_act(), NULL);
    lv_obj_set_style(box2, &style);     /*Set the new style*/
    lv_cont_set_fit(box2, true, false); /*Do not enable the vertical fit */
    lv_obj_set_height(box2, 75);        /*Set a fix height*/
//    lv_obj_set_width(box2, 210);        /*Set a fix width*/

    /*Add a text to the new container*/
    lv_obj_t * txt2 = lv_label_create(box2, NULL);
    lv_label_set_text(txt2, "2018-05-09 12:01 ...\n"
                            "2018-05-10 12:02 ...\n"
                            "2018-05-11 12:04 ...\n"
                            "2018-05-12 12:07 ...");

    /*Align the container to the bottom of the previous*/
    lv_obj_align(box2, box1, LV_ALIGN_OUT_BOTTOM_MID, 0, -37);




//    /*Create a Tab view object*/
//    lv_obj_t *tabview;
//    tabview = lv_tabview_create(lv_scr_act(), NULL);
//
//    lv_tabview_set_anim_time(tabview, 500);
//
//    /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
//    lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Tab 1");
//    lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "Tab 2");
//    lv_obj_t *tab3 = lv_tabview_add_tab(tabview, "Tab 3");
//
//
//    /*Add content to the tabs*/
//    lv_obj_t * label2 = lv_label_create(tab1, NULL);
//    lv_label_set_text(label2, "This the first tab\n\n"
//                             "If the content\n"
//                             "become too long\n"
//                             "the tab become\n"
//                             "scrollable\n\n");
//
//    label2 = lv_label_create(tab2, NULL);
//    lv_label_set_text(label2, "Second tab");
//
//    label2 = lv_label_create(tab3, NULL);
//    lv_label_set_text(label2, "Third tab");





    /*******************************
     * Create 3 similar line meter
     *******************************/

    /*Create a simple style with ticker line width*/
    static lv_style_t style_lmeter1;
    lv_style_copy(&style_lmeter1, &lv_style_pretty_color);
    style_lmeter1.line.width = 2;
    style_lmeter1.line.color = LV_COLOR_SILVER;
    style_lmeter1.body.main_color = LV_COLOR_HEX(0x91bfed);         /*Light blue*/
    style_lmeter1.body.grad_color = LV_COLOR_HEX(0x04386c);         /*Dark blue*/

    /*Create the first line meter */
//    lv_obj_t * lmeter;
    lmeter = lv_lmeter_create(lv_scr_act(), NULL);
    lv_lmeter_set_range(lmeter, 0, 100);                   /*Set the range*/
    lv_lmeter_set_value(lmeter, 30);                       /*Set the current value*/
    lv_lmeter_set_style(lmeter, &style_lmeter1);           /*Apply the new style*/
    lv_obj_set_size(lmeter, 80, 80);
    lv_obj_align(lmeter, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 20, -20);
    lv_lmeter_set_scale(lmeter, 270, 31);

    /*Add a label to show the current value*/
//    lv_obj_t * label1;
    label1 = lv_label_create(lmeter, NULL);
    lv_label_set_text(label1, "30%");
    lv_label_set_style(label1, &lv_style_pretty);
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);

    /*Create the second line meter and label*/
    lmeter = lv_lmeter_create(lv_scr_act(), lmeter);
    lv_lmeter_set_value(lmeter, 60);
    lv_obj_align(lmeter, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 120, -20);
    lv_lmeter_set_scale(lmeter, 270, 31);

    label1 = lv_label_create(lmeter, label1);
    lv_label_set_text(label1, "60%");
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);

    /*Create the third line meter and label*/
    lmeter = lv_lmeter_create(lv_scr_act(), lmeter);
    lv_lmeter_set_value(lmeter, 90);
    lv_obj_align(lmeter, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 220, -20);
    lv_lmeter_set_scale(lmeter, 270, 31);

    label1 = lv_label_create(lmeter, label1);
    lv_label_set_text(label1, "90%");
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);

    /*********************************
     * Create a greater line meter
     *********************************/

    /*Create a new style*/
    static lv_style_t style_lmeter2;
    lv_style_copy(&style_lmeter2, &lv_style_pretty_color);
    style_lmeter2.line.width = 2;
    style_lmeter2.line.color = LV_COLOR_SILVER;
    style_lmeter2.body.padding.hor = 16;            /*Line length*/
    style_lmeter2.body.main_color = LV_COLOR_LIME;
    style_lmeter2.body.grad_color = LV_COLOR_ORANGE;

    /*Create the line meter*/
    lmeter = lv_lmeter_create(lv_scr_act(), lmeter);
    lv_obj_set_style(lmeter, &style_lmeter2);
    lv_obj_set_size(lmeter, 120, 120);
    lv_obj_align(lmeter, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -20);
    lv_lmeter_set_scale(lmeter, 270, 31);
    lv_lmeter_set_value(lmeter, 40);

    /*Create a label style with greater font*/
    static lv_style_t style_label;
    lv_style_copy(&style_label, &lv_style_pretty);
    style_label.text.font = &lv_font_dejavu_30;
    style_label.text.color = LV_COLOR_RED;

    /*Add a label to show the current value*/
    label1 = lv_label_create(lmeter, label1);
    lv_label_set_text(label1, "40%");
    lv_obj_set_style(label1, &style_label);
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);



    /*********************************
     * Create a gauge meter
     *********************************/

    /*Create a style*/
    static lv_style_t style_gauge;
    lv_style_copy(&style_gauge, &lv_style_pretty_color);
    style_gauge.body.main_color = LV_COLOR_HEX3(0x666);     /*Line color at the beginning*/
    style_gauge.body.grad_color =  LV_COLOR_HEX3(0x666);    /*Line color at the end*/
    style_gauge.body.padding.hor = 10;                      /*Scale line length*/
    style_gauge.body.padding.inner = 8 ;                    /*Scale label padding*/
    style_gauge.body.border.color = LV_COLOR_HEX3(0x333);   /*Needle middle circle color*/
    style_gauge.line.width = 3;
    style_gauge.text.color = LV_COLOR_HEX3(0x333);
    style_gauge.line.color = LV_COLOR_RED;                  /*Line color after the critical value*/


    /*Describe the color for the needles*/
    static lv_color_t needle_colors[] = {LV_COLOR_BLUE, LV_COLOR_ORANGE, LV_COLOR_PURPLE};

    /*Create a gauge*/
//    lv_obj_t * gauge1 = lv_gauge_create(lv_scr_act(), NULL);
    gauge1 = lv_gauge_create(lv_scr_act(), NULL);
    lv_gauge_set_style(gauge1, &style_gauge);
    lv_gauge_set_needle_count(gauge1, 1, needle_colors);
    lv_obj_align(gauge1, NULL, LV_ALIGN_CENTER, 0, -20);
    lv_gauge_set_range(gauge1, 0, GAUGE_MAX);
    lv_gauge_set_critical_value(gauge1, GAUGE_CRITCAL);
    lv_gauge_set_scale (gauge1, 270, 21, 6);	// gauge, angle, line_cnt, label_cnt

    /*Set the values*/
    lv_gauge_set_value(gauge1, 0, 10);
//    lv_gauge_set_value(gauge1, 1, 20);
//    lv_gauge_set_value(gauge1, 2, 30);
    lv_gauge_set_value(gauge1, 0, 40);




    // Graph / Chart
    /*Create a style for the chart*/
    static lv_style_t style2;
    lv_style_copy(&style2, &lv_style_pretty);
    style2.body.shadow.width = 6;
    style2.body.shadow.color = LV_COLOR_GRAY;
    style2.line.color = LV_COLOR_GRAY;

    /*Create a chart*/
//    lv_obj_t * chart;
    chart = lv_chart_create(lv_scr_act(), NULL);
    lv_obj_set_size(chart, 220, 150);
    lv_obj_set_style(chart, &style2);
    lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 270, -50);
//    lv_chart_type_t type = static_cast<lv_chart_type_t>(LV_CHART_TYPE_POINT | LV_CHART_TYPE_LINE);
//    lv_chart_set_type(chart, type);   /*Show lines and points too*/
    lv_chart_set_type(chart, static_cast<lv_chart_type_t>(LV_CHART_TYPE_POINT | LV_CHART_TYPE_LINE));   /*Show lines and points too*/
//    static const lv_chart_type_t type = LV_CHART_TYPE_POINT | LV_CHART_TYPE_LINE;
//    lv_chart_set_type(chart, type);   /*Show lines and points too*/

//    lv_chart_set_type(chart, (lv_chart_type_t) LV_CHART_TYPE_POINT | LV_CHART_TYPE_LINE);   /*Show lines and points too*/
//    int LV_CHART_TYPE_LINE_AND_POINT = LV_CHART_TYPE_POINT | LV_CHART_TYPE_LINE;
//    lv_chart_set_type(chart, LV_CHART_TYPE_LINE_AND_POINT);   /*Show lines and points too*/

    //    lv_chart_set_range(chart2, -20, 120);
//    lv_chart_set_div_line_count(chart, 4, 0);

    lv_chart_set_series_opa(chart, LV_OPA_70);                            /*Opacity of the data series*/
    lv_chart_set_series_width(chart, 4);                                  /*Line width and point radious*/

    lv_chart_set_range(chart, 0, 100);

    /*Add two data series*/
//    lv_chart_series_t * ser1 = lv_chart_add_series(chart, LV_COLOR_RED);
//    lv_chart_series_t * ser2 = lv_chart_add_series(chart, LV_COLOR_GREEN);
    ser1 = lv_chart_add_series(chart, LV_COLOR_RED);
    ser2 = lv_chart_add_series(chart, LV_COLOR_GREEN);

    /*Set the next points on 'dl1'*/
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 50);
    lv_chart_set_next(chart, ser1, 70);
    lv_chart_set_next(chart, ser1, 90);

    /*Directly set points on 'dl2'*/
    ser2->points[0] = 90;
    ser2->points[1] = 65;
    ser2->points[2] = 72;
    ser2->points[3] = 40;
    ser2->points[4] = 55;
    ser2->points[5] = 60;

    lv_chart_refresh(chart); /*Required after direct set*/




    // Switches

    /*Create styles for the switches*/
    static lv_style_t bg_style;
    static lv_style_t indic_style;
    static lv_style_t knob_on_style;
    static lv_style_t knob_off_style;
    lv_style_copy(&bg_style, &lv_style_pretty);
    bg_style.body.radius = LV_RADIUS_CIRCLE;

    lv_style_copy(&indic_style, &lv_style_pretty_color);
    indic_style.body.radius = LV_RADIUS_CIRCLE;
    indic_style.body.main_color = LV_COLOR_HEX(0x9fc8ef);
    indic_style.body.grad_color = LV_COLOR_HEX(0x658099);
    indic_style.body.padding.hor = 0;
    indic_style.body.padding.ver = 0;

    lv_style_copy(&knob_off_style, &lv_style_pretty);
    knob_off_style.body.radius = LV_RADIUS_CIRCLE;
    knob_off_style.body.shadow.width = 4;
    knob_off_style.body.shadow.type = LV_SHADOW_BOTTOM;

    lv_style_copy(&knob_on_style, &lv_style_pretty_color);
    knob_on_style.body.radius = LV_RADIUS_CIRCLE;
    knob_on_style.body.shadow.width = 4;
    knob_on_style.body.shadow.type = LV_SHADOW_BOTTOM;

    /*Create a switch and apply the styles*/
    sw1 = lv_sw_create(lv_scr_act(), NULL);
    lv_sw_set_style(sw1, LV_SW_STYLE_BG, &bg_style);
    lv_sw_set_style(sw1, LV_SW_STYLE_INDIC, &indic_style);
    lv_sw_set_style(sw1, LV_SW_STYLE_KNOB_ON, &knob_on_style);
    lv_sw_set_style(sw1, LV_SW_STYLE_KNOB_OFF, &knob_off_style);
//    lv_obj_align(sw1, NULL, LV_ALIGN_CENTER, 0, -50);
	lv_obj_set_size(sw1, 80, 40);
    lv_obj_align(sw1, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -200, -30);

    /*Copy the first switch and turn it ON*/
    sw2 = lv_sw_create(lv_scr_act(), sw1);
	lv_obj_set_size(sw2, 80, 40);
    lv_obj_align(sw2, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -200, -100);

    lv_sw_set_action (sw1, sw1_action);
    lv_sw_set_action (sw2, sw2_action);
    lv_sw_on(sw2);



    // Rectangle or Circle or what?
    static lv_area_t cords;
    cords.x1 = 110;
    cords.y1 = 10;
    cords.x2 = 500;
    cords.y2 = 240;
    static lv_area_t mask;
    mask.x1 = 10;
    mask.y1 = 10;
    mask.x2 = 400;
    mask.y2 = 240;
	static lv_style_t style_rect;
    lv_style_copy(&style_rect, &lv_style_pretty);
	style_rect.body.main_color =  LV_COLOR_LIME;
	style_rect.body.grad_color =  LV_COLOR_GREEN;
	style_rect.body.radius = LV_RADIUS_CIRCLE;
	style_rect.body.opa = LV_OPA_70;
	style_rect.body.border.color = LV_COLOR_WHITE;
	style_rect.body.border.width = 3;
	style_rect.body.border.part = LV_BORDER_RIGHT;
	style_rect.body.border.opa = LV_OPA_70;
	style_rect.body.shadow.color = LV_COLOR_LIME;
	style_rect.body.shadow.type = LV_SHADOW_BOTTOM;
	style_rect.body.shadow.width = 10;
	style_rect.body.padding.inner = 8;
	style_rect.body.padding.hor = 5;
	style_rect.body.padding.ver = 5;
    style_rect.text.color = LV_COLOR_HEX3(0x333);
    style_rect.text.font = &lv_font_dejavu_30;
    style_rect.text.letter_space = 1;
    style_rect.text.line_space = 1;
    style_rect.text.opa = LV_OPA_50;
    style_rect.image.color = LV_COLOR_HEX(0x202020);
    style_rect.image.intense = LV_OPA_50;
    style_rect.image.opa = LV_OPA_50;
    style_rect.line.color = LV_COLOR_RED;
    style_rect.line.width = 3;
    style_rect.line.opa = LV_OPA_50;
    lv_draw_rect(&cords, &mask, &style_rect);

    void (*fill_fp)(const lv_area_t * coords, const lv_area_t * mask, lv_color_t color, lv_opa_t opa) = fill_wrapper;


    int direction = 1;
    float slider_value_float = 0;
    float slider_value_dest = GAUGE_MAX;
    float stepDiv = 1200000 / GAUGE_MAX;
    float min_step = GAUGE_MAX / 40000;
    while(1)
    {
    	int slider_value = (int) slider_value_float;
		lv_gauge_set_value(gauge1, 0, slider_value);
		float step = (slider_value_dest - slider_value_float) / stepDiv;
		if (step < 0) step *= -1;
		if (step < min_step) step = min_step;
		if (slider_value_dest > slider_value_float)
		{
			if (slider_value_float < GAUGE_MAX)
				slider_value_float += step;
			if (slider_value_float >= GAUGE_MAX-1)
			{
				slider_value_dest = 0;
			}
		}
		else
		{
			if (slider_value_float > 0)
				slider_value_float -= step;
			if (slider_value_float <= 1)
			{
				slider_value_dest = GAUGE_MAX;
			}
		}

		lv_tick_inc(1);

		lv_task_handler();
		wait(0.00005);
    }
}
