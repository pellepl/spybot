#include "system.h"
#include "adc.h"
#include "miniutils.h"

#ifdef CONFIG_ADC

#define ADC_IDLE          0
#define ADC_JOYST_FIRST   1
#define ADC_JOYST_SECOND  2
#define ADC_AUDIO         3

static struct {
  volatile u8_t state;
  u8_t *buf;
  u32_t len;
  u32_t ix;
  bool trig_upper;
  bool trig_lower;
  adc_cb adc_finished_cb;
} adc1;
static struct {
  volatile u8_t state;
  u16_t spl1, spl2;
  adc_cb adc_finished_cb;
} adc2;

void ADC_init() {
  adc2.state = ADC_IDLE;
  adc1.state = ADC_IDLE;

  ADC_InitTypeDef  ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);

  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);

  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);

  // reset calibration
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  // recalibrate
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));

  // reset calibration
  ADC_ResetCalibration(ADC2);
  while(ADC_GetResetCalibrationStatus(ADC2));
  // recalibrate
  ADC_StartCalibration(ADC2);
  while(ADC_GetCalibrationStatus(ADC2));
}

s32_t ADC_sample_joystick(adc_cb cb) {
  if (adc2.state != ADC_IDLE) {
    return ADC_ERR_BUSY;
  }
  adc2.adc_finished_cb = cb;
  adc2.state = ADC_JOYST_FIRST;
  ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_71Cycles5);
  ADC_SoftwareStartConvCmd(ADC2, ENABLE);

  return ADC_OK;
}

s32_t ADC_sample_sound(adc_cb cb, u8_t *buf, u32_t len) {
  if (adc1.state != ADC_IDLE) {
    return ADC_ERR_BUSY;
  }
  adc1.adc_finished_cb = cb;
  adc1.buf = buf;
  adc1.len = len;
  adc1.ix = 0;
  adc1.state = ADC_AUDIO;
  adc1.trig_upper = FALSE;
  adc1.trig_lower = FALSE;
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  return ADC_OK;
}


void ADC_irq(void) {
  // audio
  if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    u16_t val = ADC_GetConversionValue(ADC1);
    if (!adc1.trig_upper) {
      if (val > 2048+256) {
        adc1.trig_upper = TRUE;
        if (adc1.ix >= adc1.len / 4) {
          memcpy(&adc1.buf[0], &adc1.buf[adc1.ix - adc1.len / 4],  (adc1.len / 4)-1);
          adc1.ix = adc1.len / 4;
        }
      }
    } else {
      if (!adc1.trig_lower) {
        if (val < 2048-256) {
          adc1.trig_lower = TRUE;
          if (adc1.ix >= adc1.len / 2) {
            memcpy(&adc1.buf[0], &adc1.buf[adc1.ix - adc1.len / 2],  (adc1.len / 2)-1);
            adc1.ix = adc1.len / 2;
          }
        }
      }
    }
    adc1.buf[adc1.ix++] = (val >> (4+3));
    if (adc1.ix < adc1.len) {
      ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    } else {
      adc1.state = ADC_IDLE;
      if (adc1.adc_finished_cb) {
        adc1.adc_finished_cb(0,0);
      }
    }
  }

  // joystick
  if (ADC_GetITStatus(ADC2, ADC_IT_EOC) != RESET) {
    ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
    if (adc2.state == ADC_JOYST_FIRST) {
      adc2.state = ADC_JOYST_SECOND;
      adc2.spl1 = ADC_GetConversionValue(ADC2);
      ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5);
      ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    } else {
      adc2.spl2 = ADC_GetConversionValue(ADC2);
      adc2.state = ADC_IDLE;
      if (adc2.adc_finished_cb) {
        adc2.adc_finished_cb(adc2.spl1, adc2.spl2);
      } else {
        DBG(D_APP, D_INFO, "ADC: %04x %04x\n", adc2.spl1, adc2.spl2);
      }
    }
  }
}

#endif
