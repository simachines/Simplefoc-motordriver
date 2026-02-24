#include "config.h"
#include "BLDC_controller.h"

#if defined(VOLTAGE_SENSING)
static TIM_HandleTypeDef htim_adc_trig = {0};
static ADC_HandleTypeDef hadc1 = {0};
static ADC_HandleTypeDef hadc2 = {0};
static DMA_HandleTypeDef hdma_adc1 = {0};
static DMA_HandleTypeDef hdma_adc2 = {0};
#if defined(STM32F4)
static ADC_HandleTypeDef hadc3 = {0};
static DMA_HandleTypeDef hdma_adc3 = {0};
#endif

static volatile uint16_t adc1_dma_raw[2] = {0, 0};
static volatile uint16_t adc2_dma_raw[2] = {0, 0};
#if defined(STM32F4)
static volatile uint16_t adc3_dma_raw[1] = {0};
#endif
static volatile bool adc1_seq_ready = false;
static volatile bool adc2_seq_ready = false;
#if defined(STM32F4)
static volatile bool adc3_seq_ready = false;
#endif
static volatile uint16_t vbus_adc_dma_raw = 0;

static uint16_t ur = 0;
static uint16_t vr = 0;
static uint16_t wr = 0;
static uint16_t pwm_res = 0;
static constexpr uint16_t pwm_margin = 16;

extern RT_MODEL *const rtM_Right;
extern ExtU rtU_Right;
extern ExtY rtY_Right;

static uint32_t getApb1TimerClockHz(void) {
  RCC_ClkInitTypeDef clkInit = {0};
  uint32_t flashLatency = 0;
  HAL_RCC_GetClockConfig(&clkInit, &flashLatency);

  const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  return (clkInit.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk1 : (pclk1 * 2u);
}

static uint16_t adcToCenteredCurrent(uint16_t raw) {
  return static_cast<uint16_t>(static_cast<int32_t>(raw) - 2048);
}

static bool init_adc_trigger_timer(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();

  const uint32_t timerClockHz = getApb1TimerClockHz();
  if (timerClockHz == 0u) {
    return false;
  }

  uint32_t prescaler = 0u;
  uint32_t period = timerClockHz / PWM_FREQ;
  if (period == 0u) {
    return false;
  }

  while (period > 0x10000u) {
    ++prescaler;
    period = timerClockHz / ((prescaler + 1u) * PWM_FREQ);
    if (period == 0u) {
      return false;
    }
  }

  htim_adc_trig.Instance = TIM2;
  htim_adc_trig.Init.Prescaler = prescaler;
  htim_adc_trig.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_adc_trig.Init.Period = period - 1u;
  htim_adc_trig.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_adc_trig.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim_adc_trig) != HAL_OK) {
    return false;
  }

  TIM_MasterConfigTypeDef masterConfig = {0};
  masterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim_adc_trig, &masterConfig) != HAL_OK) {
    return false;
  }

  return HAL_TIM_Base_Start(&htim_adc_trig) == HAL_OK;
}

static void apply_bldc_step_from_dma(void) {
  rtU_Right.b_motEna = true;
  rtU_Right.z_ctrlModReq = CTRL_MOD_REQ;
  rtU_Right.b_hallA = 0;
  rtU_Right.b_hallB = 1;
  rtU_Right.b_hallC = 0;
  rtU_Right.i_phaAB = static_cast<int16_T>(static_cast<int32_t>(adcToCenteredCurrent(adc1_dma_raw[0])) * A2BIT_CONV);
  rtU_Right.i_phaBC = static_cast<int16_T>(static_cast<int32_t>(adcToCenteredCurrent(adc2_dma_raw[0])) * A2BIT_CONV);
  rtU_Right.i_DCLink = static_cast<int16_T>((static_cast<int32_t>(vbus_adc_dma_raw) - 2048) * A2BIT_CONV);
  rtU_Right.r_inpTgt = target_current;
  rtU_Right.a_mechAngle = 0;

  BLDC_controller_step(rtM_Right);

  ur = static_cast<uint16_t>(CLAMP(rtY_Right.DC_phaA + static_cast<int16_t>(pwm_res / 2u), pwm_margin, pwm_res - pwm_margin));
  vr = static_cast<uint16_t>(CLAMP(rtY_Right.DC_phaB + static_cast<int16_t>(pwm_res / 2u), pwm_margin, pwm_res - pwm_margin));
  wr = static_cast<uint16_t>(CLAMP(rtY_Right.DC_phaC + static_cast<int16_t>(pwm_res / 2u), pwm_margin, pwm_res - pwm_margin));

  RIGHT_TIM->RIGHT_TIM_U = ur;
  RIGHT_TIM->RIGHT_TIM_V = vr;
  RIGHT_TIM->RIGHT_TIM_W = wr;
}

static bool init_adc_dma_pipeline(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();

#if defined(STM32F4)
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();
  __HAL_RCC_ADC3_CLK_ENABLE();
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
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  hadc1.Instance = ADC1;
  hadc2.Instance = ADC2;
#if defined(STM32F4)
  hadc3.Instance = ADC3;
#endif

#if defined(STM32F4)
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 0;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

  hadc2.Init = hadc1.Init;
  hadc2.Init.NbrOfConversion = 2;

  hadc3.Init = hadc1.Init;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
#elif defined(STM32G4)
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;

  hadc2.Init = hadc1.Init;
#endif

  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    return false;
  }
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    return false;
  }
#if defined(STM32F4)
  if (HAL_ADC_Init(&hadc3) != HAL_OK) {
    return false;
  }
#endif

  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = ADC_CH_CURR_A;
#if defined(STM32G4)
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
#elif defined(STM32F4)
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
#endif
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    return false;
  }

  sConfig.Channel = ADC_CH_VBUS;
#if defined(STM32G4)
  sConfig.Rank = ADC_REGULAR_RANK_2;
#else
  sConfig.Rank = 2;
#endif
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    return false;
  }

  sConfig.Channel = ADC_CH_CURR_C;
#if defined(STM32G4)
  sConfig.Rank = ADC_REGULAR_RANK_1;
#else
  sConfig.Rank = 1;
#endif
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    return false;
  }

#if defined(STM32G4)
  sConfig.Channel = ADC_CH_CURR_C;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    return false;
  }
#elif defined(STM32F4)
  sConfig.Channel = ADC_CH_CURR_C;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    return false;
  }

  sConfig.Channel = ADC_CH_VBUS;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    return false;
  }
#endif

#if defined(STM32G4)
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    return false;
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    return false;
  }
#endif

#if defined(STM32F4)
  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc2.Instance = DMA2_Stream2;
  hdma_adc2.Init.Channel = DMA_CHANNEL_1;
  hdma_adc3.Instance = DMA2_Stream1;
  hdma_adc3.Init.Channel = DMA_CHANNEL_2;

  DMA_HandleTypeDef* dmas[3] = {&hdma_adc1, &hdma_adc2, &hdma_adc3};
  for (uint8_t i = 0; i < 3; i++) {
    dmas[i]->Init.Direction = DMA_PERIPH_TO_MEMORY;
    dmas[i]->Init.PeriphInc = DMA_PINC_DISABLE;
    dmas[i]->Init.MemInc = DMA_MINC_ENABLE;
    dmas[i]->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dmas[i]->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dmas[i]->Init.Mode = DMA_CIRCULAR;
    dmas[i]->Init.Priority = DMA_PRIORITY_HIGH;
    dmas[i]->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(dmas[i]) != HAL_OK) {
      return false;
    }
  }

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
  __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);
  __HAL_LINKDMA(&hadc3, DMA_Handle, hdma_adc3);

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
#elif defined(STM32G4)
  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
    return false;
  }

  hdma_adc2.Instance = DMA1_Channel2;
  hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
  hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc2.Init.Mode = DMA_CIRCULAR;
  hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) {
    return false;
  }

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
  __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
#endif

  if (!init_adc_trigger_timer()) {
    return false;
  }

  if (HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(adc1_dma_raw)), 2) != HAL_OK) {
    return false;
  }
  if (HAL_ADC_Start_DMA(&hadc2, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(adc2_dma_raw)), 2) != HAL_OK) {
    return false;
  }
#if defined(STM32F4)
  if (HAL_ADC_Start_DMA(&hadc3, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(adc3_dma_raw)), 1) != HAL_OK) {
    return false;
  }
#endif

  return true;
}
#endif

#if defined(ESTOP_ENABLE)
static uint8_t estop_state = 0U;
static uint8_t estop_sample_prev = 0U;
static uint32_t estop_change_tick = 0U;
#endif

RT_MODEL rtM_Left_;                     /* Real-time model */
RT_MODEL rtM_Right_;                    /* Real-time model */
RT_MODEL *const rtM_Left  = &rtM_Left_;
RT_MODEL *const rtM_Right = &rtM_Right_;

extern P rtP_Left;                      /* Block parameters (auto storage) */
DW       rtDW_Left;                     /* Observable states */
ExtU     rtU_Left;                      /* External inputs */
ExtY     rtY_Left;                      /* External outputs */

P        rtP_Right;                     /* Block parameters (auto storage) */
DW       rtDW_Right;                    /* Observable states */
ExtU     rtU_Right;                     /* External inputs */
ExtY     rtY_Right;                     /* External outputs */

float target_current_to_amps(int16_t target) {
  constexpr float TARGET_INPUT_MAX = 16000.0f;
  float normalized = static_cast<float>(target) / TARGET_INPUT_MAX;
  normalized = CLAMP(normalized, -1.0f, 1.0f);
  return normalized * maxCurrent;
}

#if defined(VOLTAGE_SENSING)
bool init_vbus_adc2_dma(void) {
  return init_adc_dma_pipeline();
}

float vbus_from_dma_counts(void) {
  const float pin_voltage = static_cast<float>(vbus_adc_dma_raw & 0x0FFFu) * VBUS_ADC_SCALE;
  return pin_voltage * v_bus_scale;
}

volatile uint32_t* vbus_adc2_dma_address(void) {
  return reinterpret_cast<volatile uint32_t*>(&vbus_adc_dma_raw);
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

void BLDC_Init(void) {
  pwm_res = static_cast<uint16_t>(LEFT_TIM->ARR);
  /* Set BLDC controller parameters */ 
  #if defined(ENCODER_X) || defined(ENCODER_Y)
  rtP_Left.b_angleMeasEna       = 1;            // Motor angle input: 0 = estimated angle, 1 = measured angle (e.g. if encoder is available)
  #else
  rtP_Left.b_angleMeasEna       = 0;            // Motor angle input: 0 = estimated angle, 1 = measured angle (e.g. if encoder is available)
  #endif
  rtP_Left.z_selPhaCurMeasABC   = 0;            // Left motor measured current phases {Blue, Yellow} = {iB, iC} -> do NOT change
  rtP_Left.z_ctrlTypSel         = CTRL_TYP_SEL;
  rtP_Left.b_diagEna            = DIAG_ENA;
  rtP_Left.i_max                = (I_MOT_MAX * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  rtP_Left.n_max                = N_MOT_MAX << 4;                       // fixdt(1,16,4)
  rtP_Left.b_fieldWeakEna       = FIELD_WEAK_ENA; 
  rtP_Left.id_fieldWeakMax      = (FIELD_WEAK_MAX * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  rtP_Left.a_phaAdvMax          = PHASE_ADV_MAX << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakHi        = FIELD_WEAK_HI << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakLo        = FIELD_WEAK_LO << 4;                   // fixdt(1,16,4)
  rtP_Left.n_polePairs          = N_POLE_PAIRS;                        // fixdt(1,16,4)
  rtP_Left.cf_idKi              = CFG_CF_IDKI;
  rtP_Left.cf_idKp              = CFG_CF_IDKP;
  rtP_Left.cf_iqKi              = CFG_CF_IQKI;
  rtP_Left.cf_iqKp              = CFG_CF_IQKP;

  rtP_Right                     = rtP_Left;     // Copy the Left motor parameters to the Right motor parameters
  rtP_Right.z_selPhaCurMeasABC  = 1;            // Right motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change

  /* Pack LEFT motor data into RTM */
  rtM_Left->defaultParam        = &rtP_Left;
  rtM_Left->dwork               = &rtDW_Left;
  rtM_Left->inputs              = &rtU_Left;
  rtM_Left->outputs             = &rtY_Left;

  /* Pack RIGHT motor data into RTM */
  rtM_Right->defaultParam       = &rtP_Right;
  rtM_Right->dwork              = &rtDW_Right;
  rtM_Right->inputs             = &rtU_Right;
  rtM_Right->outputs            = &rtY_Right;

  /* Initialize BLDC controllers */
  BLDC_controller_initialize(rtM_Left);
  BLDC_controller_initialize(rtM_Right);
}

#if defined(VOLTAGE_SENSING)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    adc1_seq_ready = true;
  } else if (hadc->Instance == ADC2) {
    adc2_seq_ready = true;
#if defined(STM32F4)
  } else if (hadc->Instance == ADC3) {
    adc3_seq_ready = true;
#endif
  }

#if defined(STM32F4)
  if (!(adc1_seq_ready && adc2_seq_ready && adc3_seq_ready)) {
    return;
  }
#else
  if (!(adc1_seq_ready && adc2_seq_ready)) {
    return;
  }
#endif

  adc1_seq_ready = false;
  adc2_seq_ready = false;
#if defined(STM32F4)
  adc3_seq_ready = false;
  vbus_adc_dma_raw = adc3_dma_raw[0];
#else
  vbus_adc_dma_raw = adc1_dma_raw[1];
#endif

  apply_bldc_step_from_dma();
}

extern "C" void DMA1_Channel1_IRQHandler(void) {
#if defined(STM32G4)
  HAL_DMA_IRQHandler(&hdma_adc1);
#endif
}

extern "C" void DMA1_Channel2_IRQHandler(void) {
#if defined(STM32G4)
  HAL_DMA_IRQHandler(&hdma_adc2);
#endif
}

extern "C" void DMA2_Stream0_IRQHandler(void) {
#if defined(STM32F4)
  HAL_DMA_IRQHandler(&hdma_adc1);
#endif
}

extern "C" void DMA2_Stream1_IRQHandler(void) {
#if defined(STM32F4)
  HAL_DMA_IRQHandler(&hdma_adc3);
#endif
}

extern "C" void DMA2_Stream2_IRQHandler(void) {
#if defined(STM32F4)
  HAL_DMA_IRQHandler(&hdma_adc2);
#endif
}
#endif
