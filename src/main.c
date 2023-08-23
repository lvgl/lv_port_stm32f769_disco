/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */


#include "stm32f7xx.h"
#include "stm32f769i_discovery.h"
#include "hal_stm_lvgl/tft/tft.h"
#include "hal_stm_lvgl/touchpad/touchpad.h"
#include "lvgl/lvgl.h"
#include "lvgl/examples/lv_examples.h"
#include "lvgl/demos/lv_demos.h"

static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);

int main(void) {

	/* Enable the CPU Cache */
	CPU_CACHE_Enable();

	/* STM32F7xx HAL library initialization:
	       - Configure the Flash prefetch
	       - Systick timer is configured by default as source of time base, but user
	         can eventually implement his proper time base source (a general purpose
	         timer for example or other time source), keeping in mind that Time base
	         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
	         handled in milliseconds basis.
	       - Set NVIC Group Priority to 4
	       - Low Level Initialization
	 */
	HAL_Init();

	SystemClock_Config();

	lv_init();

	tft_init();
	touchpad_init();

//	lv_demo_benchmark(LV_DEMO_BENCHMARK_MODE_RENDER_AND_DRIVER);
	lv_demo_widgets();

//	  lv_obj_t * main_cont = lv_obj_create(lv_scr_act());
//	  lv_obj_center(main_cont);
//	  lv_obj_set_size(main_cont, 740, 400);
//	  lv_obj_set_flex_flow(main_cont, LV_FLEX_FLOW_ROW_WRAP);
//
//	  uint32_t i;
//	  for(i = 0; i < 20; i++) {
//	      lv_obj_t * cont = lv_obj_create(main_cont);
//	      lv_obj_set_size(cont, 130, 100);
//	      lv_obj_set_style_pad_all(cont, 5, 0);
//	      lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW_WRAP);
//
//	      uint32_t j;
//	      for(j = 0; j < 5; j++) {
//	          lv_obj_t * btn = lv_btn_create(cont);
//	          lv_obj_t * label = lv_label_create(btn);
//	          lv_label_set_text(label, "A");
//	      }
//	  }


	uint32_t i;
	for(i = 0; i < 1000; i++) {
		lv_obj_invalidate(lv_scr_act());
		lv_refr_now(NULL);
	}



	while(1) {
	    lv_task_handler();
//		HAL_Delay(1);
	    lv_obj_invalidate(lv_scr_act());
	}


	while(1) {
	    lv_task_handler();
		HAL_Delay(1);
	}
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}


/**
* @brief  CPU L1-Cache enable.
* @param  None
* @retval None
*/
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}
