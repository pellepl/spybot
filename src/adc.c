#include "system.h"
#include "adc.h"

#ifdef CONFIG_ADC

void ADC_init() {
  ADC_InitTypeDef  ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
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
}

u32_t ADC_sample(int ch) {
  if (ch == 0) {
    ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
  } else {
    ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5);
  }
  ADC_SoftwareStartConvCmd(ADC2, ENABLE);
  while (ADC_GetSoftwareStartConvStatus(ADC2));
  return ADC_GetConversionValue(ADC2);
}


void ADC_irq(void) {
  if (ADC_GetITStatus(ADC2, ADC_IT_EOC) != RESET) {
    ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
    // TODO
  }
}

#endif
