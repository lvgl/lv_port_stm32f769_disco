/**
 * @file disp.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_conf.h"
#include "lvgl/lvgl.h"
#include <string.h>

#include "tft.h"
#include "stm32f7xx_hal.h"

#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_sdram.h"

/*********************
 *      DEFINES
 *********************/

/* DMA Stream parameters definitions. You can modify these parameters to select
   a different DMA Stream and/or channel.
   But note that only DMA2 Streams are capable of Memory to Memory transfers. */
#define DMA_STREAM               DMA2_Stream0
#define DMA_CHANNEL              DMA_CHANNEL_0
#define DMA_STREAM_IRQ           DMA2_Stream0_IRQn
#define DMA_STREAM_IRQHANDLER    DMA2_Stream0_IRQHandler

#if TFT_NO_TEARING
#define ZONES               4       /*Divide the screen into zones to handle tearing effect*/
#else
#define ZONES               1
#endif

#define VSYNC               OTM8009A_800X480_VSYNC
#define VBP                 OTM8009A_800X480_VBP
#define VFP                 OTM8009A_800X480_VFP
#define VACT                OTM8009A_800X480_HEIGHT
#define HSYNC               OTM8009A_800X480_HSYNC
#define HBP                 OTM8009A_800X480_HBP
#define HFP                 OTM8009A_800X480_HFP
#define HACT                (OTM8009A_800X480_WIDTH / ZONES)

#define LAYER0_ADDRESS               (LCD_FB_START_ADDRESS)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/*For LittlevGL*/
static void tft_flush_cb(lv_disp_t * disp, const lv_area_t * area, uint8_t * pxmap);

/*LCD*/
static void LCD_Config(void);
static void LTDC_Init(void);

/*DMA to flush to frame buffer*/
static void DMA_Config(void);
static void DMA_TransferComplete(DMA_HandleTypeDef *han);
static void DMA_TransferError(DMA_HandleTypeDef *han);

/**********************
 *  STATIC VARIABLES
 **********************/

extern LTDC_HandleTypeDef hltdc_discovery;
extern DSI_HandleTypeDef hdsi_discovery;
DSI_VidCfgTypeDef hdsivideo_handle;
DSI_CmdCfgTypeDef CmdCfg;
DSI_LPCmdTypeDef LPCmd;
DSI_PLLInitTypeDef dsiPllInit;
static RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

#if LV_COLOR_DEPTH == 16
static uint16_t * my_fb = (uint16_t *)LAYER0_ADDRESS;
#else
static uint32_t * my_fb = (uint32_t *)LAYER0_ADDRESS;
#endif

static lv_display_t * disp;

static DMA_HandleTypeDef     DmaHandle;
static volatile int32_t x1_flush;
static volatile int32_t y1_flush;
static volatile int32_t x2_flush;
static volatile int32_t y2_flush;
static volatile int32_t y_flush_act;
static volatile const uint8_t * buf_to_flush;

static volatile bool refr_qry;
static volatile uint32_t t_last = 0;

#if TFT_NO_TEARING
uint8_t pPage[]       = {0x00, 0x00, 0x01, 0xDF}; /*   0 -> 479 */


uint8_t pCols[ZONES][4] =
{
#if (ZONES == 4 )
  {0x00, 0x00, 0x00, 0xC7}, /*   0 -> 199 */
  {0x00, 0xC8, 0x01, 0x8F}, /* 200 -> 399 */
  {0x01, 0x90, 0x02, 0x57}, /* 400 -> 599 */
  {0x02, 0x58, 0x03, 0x1F}, /* 600 -> 799 */
#elif (ZONES == 2 )
  {0x00, 0x00, 0x01, 0x8F}, /*   0 -> 399 */
  {0x01, 0x90, 0x03, 0x1F}
#elif (ZONES == 1 )
  {0x00, 0x00, 0x03, 0x1F}, /*   0 -> 799 */
#endif
};
#endif

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
	BSP_SDRAM_Init();
	/* Deactivate speculative/cache access to first FMC Bank to save FMC bandwidth */
	FMC_Bank1->BTCR[0] = 0x000030D2;
	LCD_Config();

	/* Send Display On DCS Command to display */
	HAL_DSI_ShortWrite(&(hdsi_discovery),
			0,
			DSI_DCS_SHORT_PKT_WRITE_P1,
			OTM8009A_CMD_DISPON,
			0x00);

	DMA_Config();

	static uint8_t buf1[TFT_HOR_RES * 48 * 2];
	static uint8_t buf2[TFT_HOR_RES * 48 * 2];
	disp = lv_display_create(800, 480);
	lv_display_set_buffers(disp, buf1, buf2, TFT_HOR_RES * 48 * 2, LV_DISP_RENDER_MODE_PARTIAL);
	lv_display_set_flush_cb(disp, tft_flush_cb);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void tft_flush_cb(lv_disp_t * disp, const lv_area_t * area, uint8_t * pxmap)
{

	SCB_CleanInvalidateDCache();

	/*Truncate the area to the screen*/
	int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
	int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
	int32_t act_x2 = area->x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : area->x2;
	int32_t act_y2 = area->y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : area->y2;

	x1_flush = act_x1;
	y1_flush = act_y1;
	x2_flush = act_x2;
	y2_flush = act_y2;
	y_flush_act = act_y1;
	buf_to_flush = pxmap;

	/*Use DMA instead of DMA2D to leave it free for GPU*/
	HAL_StatusTypeDef err;
	err = HAL_DMA_Start_IT(&DmaHandle,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_flush_act * TFT_HOR_RES + x1_flush],
			  (x2_flush - x1_flush + 1));
	if(err != HAL_OK)
	{
		while(1);	/*Halt on error*/
	}
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
#if LV_COLOR_DEPTH == 16
	CmdCfg.ColorCoding           = DSI_RGB565;
#else
	CmdCfg.ColorCoding           = DSI_RGB888;
#endif
	CmdCfg.CommandSize           = HACT;
	CmdCfg.TearingEffectSource   = DSI_TE_EXTERNAL;
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
#if LV_COLOR_DEPTH == 16
	OTM8009A_Init(OTM8009A_FORMAT_RBG565, LCD_ORIENTATION_LANDSCAPE);
#else
	OTM8009A_Init(OTM8009A_FORMAT_RGB888, LCD_ORIENTATION_LANDSCAPE);
#endif
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


#if TFT_NO_TEARING
	  HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pCols[0]);
	  HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_PASET, pPage);

	/* Enable GPIOJ clock */
	__HAL_RCC_GPIOJ_CLK_ENABLE();

	/* Configure DSI_TE pin from MB1166 : Tearing effect on separated GPIO from KoD LCD */
	/* that is mapped on GPIOJ2 as alternate DSI function (DSI_TE)                      */
	/* This pin is used only when the LCD and DSI link is configured in command mode    */
	/* Not used in DSI Video mode.
	 */
	GPIO_InitTypeDef  GPIO_Init_Structure;
	GPIO_Init_Structure.Pin       = GPIO_PIN_2;
	GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
	GPIO_Init_Structure.Pull      = GPIO_NOPULL;
	GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
	GPIO_Init_Structure.Alternate = GPIO_AF13_DSI;
	HAL_GPIO_Init(GPIOJ, &GPIO_Init_Structure);

	static uint8_t ScanLineParams[2];
#if ZONES == 2
	uint16_t scanline = 200;
#elif ZONES == 4
	uint16_t scanline = 283;
#endif
	ScanLineParams[0] = scanline >> 8;
	ScanLineParams[1] = scanline & 0x00FF;

	HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 2, 0x44, ScanLineParams);
	/* set_tear_on */
	HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_TEEON, 0x00);
#endif

}

#if TFT_NO_TEARING
/**
* LCD_SetUpdateRegion
*/
void LCD_SetUpdateRegion(int idx)
{
  HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pCols[idx]);
}
#endif
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


    LCD_LayerCfgTypeDef  Layercfg;

   /* Layer Init */
   Layercfg.WindowX0 = 0;
   Layercfg.WindowX1 = HACT;
   Layercfg.WindowY0 = 0;
   Layercfg.WindowY1 = BSP_LCD_GetYSize();
#if LV_COLOR_DEPTH == 16
   Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
#else
   Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
#endif
   Layercfg.FBStartAdress = LAYER0_ADDRESS;
   Layercfg.Alpha = 255;
   Layercfg.Alpha0 = 0;
   Layercfg.Backcolor.Blue = 0;
   Layercfg.Backcolor.Green = 0;
   Layercfg.Backcolor.Red = 0;
   Layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
   Layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
   Layercfg.ImageWidth = BSP_LCD_GetXSize();;
   Layercfg.ImageHeight = BSP_LCD_GetYSize();

   HAL_LTDC_ConfigLayer(&hltdc_discovery, &Layercfg, 0);

}

#if TFT_NO_TEARING
static volatile uint32_t LCD_ActiveRegion;

/**
  * @brief  Tearing Effect DSI callback.
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_TearingEffectCallback(DSI_HandleTypeDef *hdsi)
{
    if(refr_qry) {
        LCD_ActiveRegion = 1;
        HAL_DSI_Refresh(hdsi);
        refr_qry = false;
    }
}

void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi)
{

    if(LCD_ActiveRegion < ZONES )
    {
        /* Disable DSI Wrapper */
        __HAL_DSI_WRAPPER_DISABLE(hdsi);
        /* Update LTDC configuaration */
        LTDC_LAYER(&hltdc_discovery, 0)->CFBAR  = LAYER0_ADDRESS + LCD_ActiveRegion  * HACT * 2;
        __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
        __HAL_DSI_WRAPPER_ENABLE(hdsi);

        LCD_SetUpdateRegion(LCD_ActiveRegion++);
        /* Refresh the right part of the display */
        HAL_DSI_Refresh(hdsi);

    }
    else
    {
        __HAL_DSI_WRAPPER_DISABLE(&hdsi_discovery);
        LTDC_LAYER(&hltdc_discovery, 0)->CFBAR  = LAYER0_ADDRESS;

        __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
        __HAL_DSI_WRAPPER_ENABLE(&hdsi_discovery);

        LCD_SetUpdateRegion(0);
        lv_disp_flush_ready(disp);
    }
}
#endif

/**
  * @brief  Configure the DMA controller according to the Stream parameters
  *         defined in main.h file
  * @note  This function is used to :
  *        -1- Enable DMA2 clock
  *        -2- Select the DMA functional Parameters
  *        -3- Select the DMA instance to be used for the transfer
  *        -4- Select Callbacks functions called after Transfer complete and
               Transfer error interrupt detection
  *        -5- Initialize the DMA stream
  *        -6- Configure NVIC for DMA transfer complete/error interrupts
  * @param  None
  * @retval None
  */
static void DMA_Config(void)
{
  /*## -1- Enable DMA2 clock #################################################*/
  __HAL_RCC_DMA2_CLK_ENABLE();

  /*##-2- Select the DMA functional Parameters ###############################*/
  DmaHandle.Init.Channel = DMA_CHANNEL;                     /* DMA_CHANNEL_0                    */
  DmaHandle.Init.Direction = DMA_MEMORY_TO_MEMORY;          /* M2M transfer mode                */
  DmaHandle.Init.PeriphInc = DMA_PINC_ENABLE;               /* Peripheral increment mode Enable */
  DmaHandle.Init.MemInc = DMA_MINC_ENABLE;                  /* Memory increment mode Enable     */
#if LV_COLOR_DEPTH == 16
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* Peripheral data alignment : 16bit */
  DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    /* memory data alignment : 16bit     */
#else
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; /* Peripheral data alignment : 16bit */
  DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;    /* memory data alignment : 16bit     */
#endif
  DmaHandle.Init.Mode = DMA_NORMAL;                         /* Normal DMA mode                  */
  DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;              /* priority level : high            */
  DmaHandle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;            /* FIFO mode enabled                */
  DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL; /* FIFO threshold: 1/4 full   */
  DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;              /* Memory burst                     */
  DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;           /* Peripheral burst                 */

  /*##-3- Select the DMA instance to be used for the transfer : DMA2_Stream0 #*/
  DmaHandle.Instance = DMA_STREAM;

  /*##-4- Initialize the DMA stream ##########################################*/
  if(HAL_DMA_Init(&DmaHandle) != HAL_OK)
  {
    while(1);
  }

  /*##-5- Select Callbacks functions called after Transfer complete and Transfer error */
  HAL_DMA_RegisterCallback(&DmaHandle, HAL_DMA_XFER_CPLT_CB_ID, DMA_TransferComplete);
  HAL_DMA_RegisterCallback(&DmaHandle, HAL_DMA_XFER_ERROR_CB_ID, DMA_TransferError);

  /*##-6- Configure NVIC for DMA transfer complete/error interrupts ##########*/
  HAL_NVIC_SetPriority(DMA_STREAM_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(DMA_STREAM_IRQ);
}

/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void DMA_TransferComplete(DMA_HandleTypeDef *han)
{
	y_flush_act ++;

	if(y_flush_act > y2_flush) {
#if TFT_NO_TEARING
		if(lv_disp_flush_is_last(disp)) refr_qry = true;
		else lv_disp_flush_ready(disp);
#else
		if(lv_disp_flush_is_last(disp)) HAL_DSI_Refresh(&hdsi_discovery);

		lv_disp_flush_ready(disp);
#endif
	} else {
	  buf_to_flush += (x2_flush - x1_flush + 1) * 2;
	  /*##-7- Start the DMA transfer using the interrupt mode ####################*/
	  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	  /* Enable All the DMA interrupts */
	  if(HAL_DMA_Start_IT(han,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_flush_act * TFT_HOR_RES + x1_flush],
						  (x2_flush - x1_flush + 1)) != HAL_OK)
	  {
	    while(1);	/*Halt on error*/
	  }
	}
}

/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void DMA_TransferError(DMA_HandleTypeDef *han)
{
    while(1);
}


/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void DMA_STREAM_IRQHANDLER(void)
{
    /* Check the interrupt and clear flag */
    HAL_DMA_IRQHandler(&DmaHandle);
}

/**
  * @brief  Initialize the BSP LCD Msp.
  * Do not DMA2D is initialized by LVGL
  */
void BSP_LCD_MspInit(void)
{
  /** @brief Enable the LTDC clock */
  __HAL_RCC_LTDC_CLK_ENABLE();

  /** @brief Toggle Sw reset of LTDC IP */
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_LTDC_RELEASE_RESET();

  /** @brief Enable DSI Host and wrapper clocks */
  __HAL_RCC_DSI_CLK_ENABLE();

  /** @brief Soft Reset the DSI Host and wrapper */
  __HAL_RCC_DSI_FORCE_RESET();
  __HAL_RCC_DSI_RELEASE_RESET();

  /** @brief NVIC configuration for LTDC interrupt that is now enabled */
  HAL_NVIC_SetPriority(LTDC_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(LTDC_IRQn);

  /** @brief NVIC configuration for DSI interrupt that is now enabled */
  HAL_NVIC_SetPriority(DSI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DSI_IRQn);
}
