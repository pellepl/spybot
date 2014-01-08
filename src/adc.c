#include "system.h"
#include "adc.h"
#include "miniutils.h"

#ifdef CONFIG_ADC

#define ADC_IDLE    0
#define ADC_FIRST   1
#define ADC_SECOND  2

static volatile u8_t state;
static u16_t spl1, spl2;
static adc_cb adc_finished_cb;

void ADC_init() {
  state = ADC_IDLE;

  ADC_InitTypeDef  ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC2, &ADC_InitStructure);

  ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);

  ADC_Cmd(ADC2, ENABLE);

  // reset calibration
  ADC_ResetCalibration(ADC2);
  while(ADC_GetResetCalibrationStatus(ADC2));

  // recalibrate
  ADC_StartCalibration(ADC2);
  while(ADC_GetCalibrationStatus(ADC2));

  state = ADC_IDLE;
}

s32_t ADC_sample(adc_cb cb) {
  if (state != ADC_IDLE) {
    return ADC_ERR_BUSY;
  }
  adc_finished_cb = cb;
  state = ADC_FIRST;
  ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
  ADC_SoftwareStartConvCmd(ADC2, ENABLE);

  return ADC_OK;
}


void ADC_irq(void) {
  if (ADC_GetITStatus(ADC2, ADC_IT_EOC) != RESET) {
    ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
    if (state == ADC_FIRST) {
      state = ADC_SECOND;
      spl1 = ADC_GetConversionValue(ADC2);
      ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5);
      ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    } else {
      spl2 = ADC_GetConversionValue(ADC2);
      state = ADC_IDLE;
      if (adc_finished_cb) {
        adc_finished_cb(spl1, spl2);
      } else {
        DBG(D_APP, D_INFO, "ADC: %04x %04x\n", spl1, spl2);
      }
    }
  }
}

#endif
