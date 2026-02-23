#include "config.h"

#if defined(VOLTAGE_SENSING)
static ADC_HandleTypeDef hadc2 = {0};
static DMA_HandleTypeDef hdma_adc2 = {0};
static volatile uint32_t vbus_adc2_dma_raw = 0;
#endif

#if defined(ESTOP_ENABLE)
static uint8_t estop_state = 0U;
static uint8_t estop_sample_prev = 0U;
static uint32_t estop_change_tick = 0U;
#endif

float target_current_to_amps(int16_t target) {
  constexpr float TARGET_INPUT_MAX = 16000.0f;
  float normalized = static_cast<float>(target) / TARGET_INPUT_MAX;
  normalized = CLAMP(normalized, -1.0f, 1.0f);
  return normalized * maxCurrent;
}

#if defined(VOLTAGE_SENSING)
bool init_vbus_adc2_dma(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();

#if defined(STM32F4)
  __HAL_RCC_ADC2_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
#elif defined(STM32G4)
  __HAL_RCC_ADC12_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
#ifdef __HAL_RCC_DMAMUX1_CLK_ENABLE
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
#endif
#else
  return false;
#endif

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0;
#if defined(STM32F4)
  gpio.Pin = GPIO_PIN_1;
#endif
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  hadc2.Instance = ADC2;
#if defined(STM32F4)
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.NbrOfDiscConversion = 0;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
#elif defined(STM32G4)
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
#endif

#if defined(STM32F4)
  hdma_adc2.Instance = DMA2_Stream2;
  hdma_adc2.Init.Channel = DMA_CHANNEL_1;
  hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc2.Init.MemInc = DMA_MINC_DISABLE;
  hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc2.Init.Mode = DMA_CIRCULAR;
  hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#elif defined(STM32G4)
  hdma_adc2.Instance = DMA1_Channel1;
  hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
  hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc2.Init.MemInc = DMA_MINC_DISABLE;
  hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc2.Init.Mode = DMA_CIRCULAR;
  hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
#endif

  if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) {
    return false;
  }
  __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);

  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    return false;
  }

  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_1;
#if defined(STM32G4)
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
#elif defined(STM32F4)
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
#endif

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    return false;
  }

#if defined(STM32G4)
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    return false;
  }
#endif

  if (HAL_ADC_Start_DMA(&hadc2, const_cast<uint32_t*>(&vbus_adc2_dma_raw), 1) != HAL_OK) {
    return false;
  }

  return true;
}

float vbus_from_dma_counts(void) {
  const float pin_voltage = static_cast<float>(vbus_adc2_dma_raw & 0x0FFFu) * VBUS_ADC_SCALE;
  return pin_voltage * v_bus_scale;
}

volatile uint32_t* vbus_adc2_dma_address(void) {
  return &vbus_adc2_dma_raw;
}
#else
bool init_vbus_adc2_dma(void) { return false; }
float vbus_from_dma_counts(void) { return static_cast<float>(supply_voltage_V); }
volatile uint32_t* vbus_adc2_dma_address(void) { return nullptr; }
#endif

void estop_init(void) {
#if defined(ESTOP_ENABLE)
  ESTOP_GPIO_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = ESTOP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ESTOP_PORT, &GPIO_InitStruct);

  estop_flag = 0U;
  estop_latch_flag = 0U;
  estop_change_tick = HAL_GetTick();
  estop_sample_prev = (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == ESTOP_ACTIVE_STATE);
  estop_state = estop_sample_prev;
#endif
}

void estop_update(void) {
#if defined(ESTOP_ENABLE)
  const uint32_t now = HAL_GetTick();
  const uint8_t sample = (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == ESTOP_ACTIVE_STATE);

  if (sample != estop_sample_prev) {
    estop_sample_prev = sample;
    estop_change_tick = now;
  }

  if ((now - estop_change_tick) >= ESTOP_DEBOUNCE_MS && estop_state != estop_sample_prev) {
    estop_state = estop_sample_prev;
    if (estop_state) {
#if defined(ESTOP_REQUIRE_HOLD)
      estop_flag = 1U;
#else
      if (estop_latch_flag) {
        estop_latch_flag = 0U;
        estop_flag = 0U;
      } else {
        estop_flag = 1U;
#if defined(ESTOP_BUTTON_NO)
        estop_latch_flag = 1U;
#endif
      }
#endif
    } else {
#if defined(ESTOP_REQUIRE_HOLD)
      estop_flag = 0U;
#else
#if defined(ESTOP_BUTTON_NO)
      if (!estop_latch_flag) {
        estop_flag = 0U;
      }
#else
      estop_flag = 0U;
#endif
#endif
    }
  }
#endif
}

#if defined(PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF)
void setBandwidth(char* cmd) {
  float new_bandwidth = current_bandwidth;
  commander.scalar(&new_bandwidth, cmd);

  if (new_bandwidth > 0 && new_bandwidth <= 1000) {
    current_bandwidth = new_bandwidth;

    motor.PID_current_d.P = phase_inductance * current_bandwidth * _2PI;
    motor.PID_current_d.I = motor.PID_current_d.P * phase_resistance / phase_inductance;
    motor.LPF_current_d.Tf = 1 / (_2PI * 3.0f * current_bandwidth);

    motor.PID_current_q.P = phase_inductance * current_bandwidth * _2PI;
    motor.PID_current_q.I = motor.PID_current_q.P * phase_resistance / phase_inductance;
    motor.LPF_current_q.Tf = 1 / (_2PI * 3.0f * current_bandwidth);

    Serial.printf("Current bandwidth set to: %.1f Hz\n", current_bandwidth);
    Serial.printf("Updated PID_P: %.6f, PID_I: %.6f, LPF_Tf: %.6f\n",
                  motor.PID_current_d.P, motor.PID_current_d.I, motor.LPF_current_d.Tf);
  } else {
    Serial.printf("Invalid bandwidth: %.1f Hz (must be 0-1000)\n", new_bandwidth);
  }
}

void onSetABZResolution(char* cmd) {
  float requestedPprFloat = 16384.0f;
  commander.scalar(&requestedPprFloat, cmd);
  long requestedPpr = (long)requestedPprFloat;

  if (requestedPpr < 1) {
    requestedPpr = 1;
  } else if (requestedPpr > 16384) {
    requestedPpr = 16384;
  }

  const uint16_t rawAbz = static_cast<uint16_t>(requestedPpr - 1L);
  encoder2.setABZResolution(rawAbz);

  const uint16_t readRawAbz = encoder2.getABZResolution();
  Serial.printf("ABZ rb ppr=%u", (unsigned)(readRawAbz + 1u));
}

void onPWMInputControl(char* cmd) {
#if defined(PWM_INPUT)
  if (cmd == nullptr || cmd[0] == '\0') {
    Serial.printf("C=%u\n", pwm_input_control_enabled ? 1u : 0u);
    return;
  }

  if (cmd[0] == '0') {
    pwm_input_control_enabled = false;
    Serial.println("C0");
  } else if (cmd[0] == '1') {
    pwm_input_control_enabled = true;
    Serial.println("C1");
  }
#else
  (void)cmd;
  Serial.println("PWM_INPUT off");
#endif
}

void onMotor(char* cmd) {
  commander.motor(&motor, cmd);
}
#endif

void motor_characterisation(void) {
  void characteriseMotor(float alignStrength);
  float R = motor.phase_resistance;
  float L_d = motor.axis_inductance.d;
  float L_q = motor.axis_inductance.q;

  Serial.print("Resistance: ");
  Serial.print(R);
  Serial.println(" Ohms");

  Serial.print("D-axis inductance: ");
  Serial.print(L_d * 1000);
  Serial.println(" mH");

  Serial.print("Q-axis inductance: ");
  Serial.print(L_q * 1000);
  Serial.println(" mH");
}

