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

#define VSYNC               OTM8009A_800X480_VSYNC
#define VBP                 OTM8009A_800X480_VBP
#define VFP                 OTM8009A_800X480_VFP
#define VACT                OTM8009A_800X480_HEIGHT
#define HSYNC               OTM8009A_800X480_HSYNC
#define HBP                 OTM8009A_800X480_HBP
#define HFP                 OTM8009A_800X480_HFP
#define HACT                OTM8009A_800X480_WIDTH


#define LAYER0_ADDRESS               (LCD_FB_START_ADDRESS)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/*For LittlevGL*/
static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p);
static void gpu_blend_cb(lv_disp_drv_t *disp_drv, lv_color_t *dest, const lv_color_t *src, uint32_t length, lv_opa_t opa);
static void gpu_fill_cb(lv_disp_drv_t *drv, lv_color_t *dest_buf, const lv_coord_t dest_width, const lv_area_t *fill_area, lv_color_t color);

/*LCD*/
static void LCD_Config(void);
static void LTDC_Init(void);
static void MPU_SDRAM_Config(void);
static void DMA2D_TransferComplete(DMA2D_HandleTypeDef *han);

/**********************
 *  STATIC VARIABLES
 **********************/

extern LTDC_HandleTypeDef hltdc_discovery;
static DMA2D_HandleTypeDef hdma2d;
extern DSI_HandleTypeDef hdsi_discovery;
DSI_VidCfgTypeDef hdsivideo_handle;
DSI_CmdCfgTypeDef CmdCfg;
DSI_LPCmdTypeDef LPCmd;
DSI_PLLInitTypeDef dsiPllInit;
static RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

static uint32_t * my_fb = (uint32_t *)LAYER0_ADDRESS;

static lv_disp_drv_t disp_drv;
static volatile bool gpu_busy;
static volatile bool copy_buf;
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
static volatile uint32_t t_saved = 0;
void monitor_cb(lv_disp_drv_t * d, uint32_t t, uint32_t p)
{
	t_saved = t;
}

/**
 * Initialize your display here
 */
void tft_init(void)
{

	BSP_SDRAM_Init();
	MPU_SDRAM_Config();
	/* Deactivate speculative/cache access to first FMC Bank to save FMC bandwidth */
	FMC_Bank1->BTCR[0] = 0x000030D2;
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

	static lv_disp_buf_t disp_buf;
	static lv_color_t buf[TFT_HOR_RES * 48];
	static lv_color_t buf2[TFT_HOR_RES * 48];
	lv_disp_buf_init(&disp_buf, buf, buf2, TFT_HOR_RES * 48);

	lv_disp_drv_init(&disp_drv);
	disp_drv.flush_cb = tft_flush_cb;
	disp_drv.monitor_cb = monitor_cb;
	disp_drv.buffer = &disp_buf;
	disp_drv.gpu_blend_cb = gpu_blend_cb;
	disp_drv.gpu_fill_cb = gpu_fill_cb;

	lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
	SCB_InvalidateICache();

	SCB_CleanInvalidateDCache();

	while(gpu_busy);

	uint8_t * pDst = (uint8_t*)my_fb;
	uint8_t * pSrc = (uint8_t*)color_p;
	uint32_t xsize = lv_area_get_width(area);
	uint32_t ysize = lv_area_get_height(area);
	uint32_t destination = (uint32_t)pDst + (area->y1 * 800 + area->x1) * 4;
	uint32_t source      = (uint32_t)pSrc;

	/*##-1- Configure the DMA2D Mode, Color Mode and output offset #############*/
	hdma2d.Init.Mode         = DMA2D_M2M;
	hdma2d.Init.ColorMode    = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Init.OutputOffset = 800 - xsize;
	hdma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/
	hdma2d.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */

	/*##-2- DMA2D Callbacks Configuration ######################################*/
	hdma2d.XferCpltCallback  = NULL;

	/*##-3- Foreground Configuration ###########################################*/
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0xFF;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR; /* No ForeGround Red/Blue swap */
	hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */

	hdma2d.XferCpltCallback = DMA2D_TransferComplete;
	hdma2d.Instance          = DMA2D;
	HAL_NVIC_SetPriority(DMA2D_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2D_IRQn);

	/* DMA2D Initialization */
	if(HAL_DMA2D_Init(&hdma2d) == HAL_OK) {
		if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK) {
			gpu_busy = true;
			copy_buf = true;
			HAL_DMA2D_Start_IT(&hdma2d, source, destination, xsize, ysize);
		}
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

static void gpu_fill_cb(lv_disp_drv_t *drv, lv_color_t *dest_buf, const lv_coord_t dest_width, const lv_area_t *fill_area, lv_color_t color)
{
	while(gpu_busy);
	SCB_CleanInvalidateDCache();

	lv_color_t * destination = dest_buf + (dest_width * fill_area->y1 + fill_area->x1);

	uint32_t w = fill_area->x2 - fill_area->x1 + 1;
	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_R2M;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Init.OutputOffset = dest_width - w;
	hdma2d.LayerCfg[1].InputAlpha = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.XferCpltCallback = DMA2D_TransferComplete;
	HAL_NVIC_SetPriority(DMA2D_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2D_IRQn);

	/* DMA2D Initialization */
	if (HAL_DMA2D_Init(&hdma2d) == HAL_OK) {
		if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK) {
			lv_coord_t h = lv_area_get_height(fill_area);
			gpu_busy = true;
			copy_buf = false;
			HAL_DMA2D_BlendingStart_IT(&hdma2d, lv_color_to32(color), (uint32_t)destination, (uint32_t)destination, w, h);
		}
	}
	while(gpu_busy);
}

static void gpu_blend_cb(lv_disp_drv_t *disp_drv, lv_color_t *dest, const lv_color_t *src, uint32_t length, lv_opa_t opa)
{
	while(gpu_busy);
	SCB_CleanInvalidateDCache();

	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M_BLEND;
	hdma2d.Init.OutputOffset = 0;

	/* Foreground layer */
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = opa;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;

	/* Background layer */
	hdma2d.LayerCfg[0].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[0].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[0].InputOffset = 0;
	hdma2d.XferCpltCallback = DMA2D_TransferComplete;
		HAL_NVIC_SetPriority(DMA2D_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA2D_IRQn);

	/* DMA2D Initialization */
	if (HAL_DMA2D_Init(&hdma2d) == HAL_OK) {
		if (HAL_DMA2D_ConfigLayer(&hdma2d, 0) == HAL_OK && HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK) {
			gpu_busy = true;
			copy_buf = false;
			HAL_DMA2D_BlendingStart_IT(&hdma2d, (uint32_t) src, (uint32_t) dest, (uint32_t) dest, length, 1);
		}
	}

	while(gpu_busy);
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

static void MPU_SDRAM_Config(void)
{

	MPU_Region_InitTypeDef MPU_InitStruct;

	HAL_MPU_Disable();
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

	// Disable caching of the frame buffer region
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = LAYER0_ADDRESS;
    MPU_InitStruct.Size = MPU_REGION_SIZE_2MB;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
}


void DMA2D_IRQHandler(void)
{
    /* Check the interrupt and clear flag */
    HAL_DMA2D_IRQHandler(&hdma2d);
}


static void DMA2D_TransferComplete(DMA2D_HandleTypeDef *han)
{

	gpu_busy = false;

	if(copy_buf) {
		copy_buf = false;
		HAL_DSI_Refresh(&hdsi_discovery);
		lv_disp_flush_ready(&disp_drv);
	}

}

