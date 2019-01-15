/**
 * @file disp.c
 * 
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_conf.h"
#include "lvgl/lv_core/lv_vdb.h"
#include "lvgl/lv_hal/lv_hal.h"
#include <string.h>

#include "tft.h"
#include "stm32f7xx_hal.h"

#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_sdram.h"

/*********************
 *      DEFINES
 *********************/


#define VSYNC               1
#define VBP                 1
#define VFP                 1
#define VACT                480
#define HSYNC               1
#define HBP                 1
#define HFP                 1
#define HACT                800

#define LAYER0_ADDRESS               (LCD_FB_START_ADDRESS)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/*These 3 functions are needed by LittlevGL*/
static void tft_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color);
static void tft_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p);
static void tft_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p);


/*LCD*/
static void LCD_Config(void);
static void LTDC_Init(void);

/*SD RAM*/
static void CopyBuffer(const uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize);

/**********************
 *  STATIC VARIABLES
 **********************/

extern LTDC_HandleTypeDef hltdc_discovery;
static DMA2D_HandleTypeDef           hdma2d;
extern DSI_HandleTypeDef hdsi_discovery;
DSI_VidCfgTypeDef hdsivideo_handle;
DSI_CmdCfgTypeDef CmdCfg;
DSI_LPCmdTypeDef LPCmd;
DSI_PLLInitTypeDef dsiPllInit;
static RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;


static uint32_t * my_fb = (uint32_t *)LAYER0_ADDRESS;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize your display here
 */
void tft_init(void)
{
	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);

	BSP_SDRAM_Init();
	LCD_Config();
	BSP_LCD_LayerDefaultInit(0, LAYER0_ADDRESS);
	BSP_LCD_SelectLayer(0);

	/* Send Display On DCS Command to display */
	HAL_DSI_ShortWrite(&(hdsi_discovery),
			0,
			DSI_DCS_SHORT_PKT_WRITE_P1,
			OTM8009A_CMD_DISPON,
			0x00);

	/*Refresh the LCD display*/
	HAL_DSI_Refresh(&hdsi_discovery);

	disp_drv.disp_flush = tft_flush;

	lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Flush a color buffer
 * @param x1 left coordinate of the rectangle
 * @param x2 right coordinate of the rectangle
 * @param y1 top coordinate of the rectangle
 * @param y2 bottom coordinate of the rectangle
 * @param color_p pointer to an array of colors
 */
static void tft_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p)
{
	/*Return if the area is out the screen*/
	if(x2 < 0) return;
	if(y2 < 0) return;
	if(x1 > TFT_HOR_RES - 1) return;
	if(y1 > TFT_VER_RES - 1) return;

	CopyBuffer((const uint32_t *)color_p, my_fb, x1, y1, x2 - x1 + 1, y2 - y1 + 1);

	HAL_DSI_Refresh(&hdsi_discovery);

	lv_flush_ready();
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
	for(x = act_x1; x <= act_x2; x++) {
		for(y = act_y1; y <= act_y2; y++) {
			my_fb[y * TFT_HOR_RES + x] = color.full;
		}
	}
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

static void LCD_Config(void)
{
	DSI_PHY_TimerTypeDef  PhyTimings;

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

	/* LCD clock configuration */
	/* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
	/* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 417 Mhz */
	/* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 417 MHz / 5 = 83.4 MHz */
	/* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 83.4 / 2 = 41.7 MHz */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 417;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	/* Base address of DSI Host/Wrapper registers to be set before calling De-Init */
	hdsi_discovery.Instance = DSI;

	HAL_DSI_DeInit(&(hdsi_discovery));

	dsiPllInit.PLLNDIV  = 100;
	dsiPllInit.PLLIDF   = DSI_PLL_IN_DIV5;
	dsiPllInit.PLLODF   = DSI_PLL_OUT_DIV1;

	hdsi_discovery.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
	hdsi_discovery.Init.TXEscapeCkdiv = 0x4;
	HAL_DSI_Init(&(hdsi_discovery), &(dsiPllInit));

	/* Configure the DSI for Command mode */
	CmdCfg.VirtualChannelID      = 0;
	CmdCfg.HSPolarity            = DSI_HSYNC_ACTIVE_HIGH;
	CmdCfg.VSPolarity            = DSI_VSYNC_ACTIVE_HIGH;
	CmdCfg.DEPolarity            = DSI_DATA_ENABLE_ACTIVE_HIGH;
	CmdCfg.ColorCoding           = DSI_RGB888;
	CmdCfg.CommandSize           = HACT;
	CmdCfg.TearingEffectSource   = DSI_TE_DSILINK;
	CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
	CmdCfg.VSyncPol              = DSI_VSYNC_FALLING;
	CmdCfg.AutomaticRefresh      = DSI_AR_DISABLE;
	CmdCfg.TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE;
	HAL_DSI_ConfigAdaptedCommandMode(&hdsi_discovery, &CmdCfg);

	LPCmd.LPGenShortWriteNoP    = DSI_LP_GSW0P_ENABLE;
	LPCmd.LPGenShortWriteOneP   = DSI_LP_GSW1P_ENABLE;
	LPCmd.LPGenShortWriteTwoP   = DSI_LP_GSW2P_ENABLE;
	LPCmd.LPGenShortReadNoP     = DSI_LP_GSR0P_ENABLE;
	LPCmd.LPGenShortReadOneP    = DSI_LP_GSR1P_ENABLE;
	LPCmd.LPGenShortReadTwoP    = DSI_LP_GSR2P_ENABLE;
	LPCmd.LPGenLongWrite        = DSI_LP_GLW_ENABLE;
	LPCmd.LPDcsShortWriteNoP    = DSI_LP_DSW0P_ENABLE;
	LPCmd.LPDcsShortWriteOneP   = DSI_LP_DSW1P_ENABLE;
	LPCmd.LPDcsShortReadNoP     = DSI_LP_DSR0P_ENABLE;
	LPCmd.LPDcsLongWrite        = DSI_LP_DLW_ENABLE;
	HAL_DSI_ConfigCommand(&hdsi_discovery, &LPCmd);

	/* Initialize LTDC */
	LTDC_Init();

	/* Start DSI */
	HAL_DSI_Start(&(hdsi_discovery));

	/* Configure DSI PHY HS2LP and LP2HS timings */
	PhyTimings.ClockLaneHS2LPTime = 35;
	PhyTimings.ClockLaneLP2HSTime = 35;
	PhyTimings.DataLaneHS2LPTime = 35;
	PhyTimings.DataLaneLP2HSTime = 35;
	PhyTimings.DataLaneMaxReadTime = 0;
	PhyTimings.StopWaitTime = 10;
	HAL_DSI_ConfigPhyTimer(&hdsi_discovery, &PhyTimings);

	/* Initialize the OTM8009A LCD Display IC Driver (KoD LCD IC Driver)
	 *  depending on configuration set in 'hdsivideo_handle'.
	 */
	OTM8009A_Init(OTM8009A_COLMOD_RGB888, LCD_ORIENTATION_LANDSCAPE);

	LPCmd.LPGenShortWriteNoP    = DSI_LP_GSW0P_DISABLE;
	LPCmd.LPGenShortWriteOneP   = DSI_LP_GSW1P_DISABLE;
	LPCmd.LPGenShortWriteTwoP   = DSI_LP_GSW2P_DISABLE;
	LPCmd.LPGenShortReadNoP     = DSI_LP_GSR0P_DISABLE;
	LPCmd.LPGenShortReadOneP    = DSI_LP_GSR1P_DISABLE;
	LPCmd.LPGenShortReadTwoP    = DSI_LP_GSR2P_DISABLE;
	LPCmd.LPGenLongWrite        = DSI_LP_GLW_DISABLE;
	LPCmd.LPDcsShortWriteNoP    = DSI_LP_DSW0P_DISABLE;
	LPCmd.LPDcsShortWriteOneP   = DSI_LP_DSW1P_DISABLE;
	LPCmd.LPDcsShortReadNoP     = DSI_LP_DSR0P_DISABLE;
	LPCmd.LPDcsLongWrite        = DSI_LP_DLW_DISABLE;
	HAL_DSI_ConfigCommand(&hdsi_discovery, &LPCmd);

	HAL_DSI_ConfigFlowControl(&hdsi_discovery, DSI_FLOW_CONTROL_BTA);

	/* Send Display Off DCS Command to display */
	HAL_DSI_ShortWrite(&(hdsi_discovery),
			0,
			DSI_DCS_SHORT_PKT_WRITE_P1,
			OTM8009A_CMD_DISPOFF,
			0x00);

	/* Refresh the display */
	HAL_DSI_Refresh(&hdsi_discovery);
}

/**
 * @brief
 * @param  None
 * @retval None
 */
static void LTDC_Init(void)
{
	/* DeInit */
	HAL_LTDC_DeInit(&hltdc_discovery);

	/* LTDC Config */
	/* Timing and polarity */
	hltdc_discovery.Init.HorizontalSync = HSYNC;
	hltdc_discovery.Init.VerticalSync = VSYNC;
	hltdc_discovery.Init.AccumulatedHBP = HSYNC+HBP;
	hltdc_discovery.Init.AccumulatedVBP = VSYNC+VBP;
	hltdc_discovery.Init.AccumulatedActiveH = VSYNC+VBP+VACT;
	hltdc_discovery.Init.AccumulatedActiveW = HSYNC+HBP+HACT;
	hltdc_discovery.Init.TotalHeigh = VSYNC+VBP+VACT+VFP;
	hltdc_discovery.Init.TotalWidth = HSYNC+HBP+HACT+HFP;

	/* background value */
	hltdc_discovery.Init.Backcolor.Blue = 0;
	hltdc_discovery.Init.Backcolor.Green = 0;
	hltdc_discovery.Init.Backcolor.Red = 0;

	/* Polarity */
	hltdc_discovery.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc_discovery.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc_discovery.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc_discovery.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc_discovery.Instance = LTDC;

	HAL_LTDC_Init(&hltdc_discovery);
}


static void CopyBuffer(const uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize)
{
	uint32_t row;
	for(row = y; row < y + ysize; row++ ) {
		memcpy(&pDst[row * 800 + x], pSrc, xsize * 4);
		pSrc += xsize;
	}

	/**********************
	 * Using DMA2D should be better and faster but makes artifacts on the screen
	 **********************/

//	uint32_t destination = (uint32_t)pDst + (y * 800 + x) * 4;
//	uint32_t source      = (uint32_t)pSrc;
//
//	/*##-1- Configure the DMA2D Mode, Color Mode and output offset #############*/
//	hdma2d.Init.Mode         = DMA2D_M2M;
//	hdma2d.Init.ColorMode    = DMA2D_OUTPUT_ARGB8888;
//	hdma2d.Init.OutputOffset = 800 - xsize;
//	hdma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/
//	hdma2d.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */
//
//	/*##-2- DMA2D Callbacks Configuration ######################################*/
//	hdma2d.XferCpltCallback  = NULL;
//
//	/*##-3- Foreground Configuration ###########################################*/
//	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
//	hdma2d.LayerCfg[1].InputAlpha = 0xFF;
//	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
//	hdma2d.LayerCfg[1].InputOffset = 0;
//	hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR; /* No ForeGround Red/Blue swap */
//	hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */
//
//	hdma2d.Instance          = DMA2D;
//
//	/* DMA2D Initialization */
//	if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
//	{
//		if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
//		{
//			if (HAL_DMA2D_Start(&hdma2d, source, destination, xsize, ysize) == HAL_OK)
//			{
//				/* Polling For DMA transfer */
//				HAL_DMA2D_PollForTransfer(&hdma2d, 100);
//			}
//		}
//	}
}
