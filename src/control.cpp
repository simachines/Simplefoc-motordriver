#include "config.h"

#if defined(BRAKE_CONTROL_ENABLED)
#if defined(STM32G4)
#define BRAKE_PWM_TIMER TIM16
#define BRAKE_PWM_CHANNEL TIM_CHANNEL_1
#define BRAKE_PWM_GPIO_PORT GPIOB
#define BRAKE_PWM_GPIO_PIN GPIO_PIN_4
#define BRAKE_PWM_GPIO_AF GPIO_AF1_TIM16
#elif defined(STM32F4)
#define BRAKE_PWM_TIMER TIM12
#define BRAKE_PWM_CHANNEL TIM_CHANNEL_1
#define BRAKE_PWM_GPIO_PORT GPIOB
#define BRAKE_PWM_GPIO_PIN GPIO_PIN_14
#define BRAKE_PWM_GPIO_AF GPIO_AF9_TIM12
#endif

static TIM_HandleTypeDef htim_brake = {0};

static uint32_t getBrakeTimerClockHz(void) {
  RCC_ClkInitTypeDef clkInit = {0};
  uint32_t flashLatency = 0;
  HAL_RCC_GetClockConfig(&clkInit, &flashLatency);

#if defined(STM32G4)
  const bool onApb2 = true;
#else
  const bool onApb2 = false;
#endif

  const uint32_t pclk = onApb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  const uint32_t divider = onApb2 ? clkInit.APB2CLKDivider : clkInit.APB1CLKDivider;
  return (divider == RCC_HCLK_DIV1) ? pclk : (pclk * 2u);
}

bool configureBrakePwm(void) {
#if defined(STM32G4)
  __HAL_RCC_TIM16_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
#elif defined(STM32F4)
  __HAL_RCC_TIM12_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
#endif

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BRAKE_PWM_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = BRAKE_PWM_GPIO_AF;
  HAL_GPIO_Init(BRAKE_PWM_GPIO_PORT, &GPIO_InitStruct);

  const uint32_t timerClkHz = getBrakeTimerClockHz();
  if (timerClkHz == 0u) {
    return false;
  }

  uint32_t prescaler = 0;
  uint32_t periodCounts = timerClkHz / BRAKE_PWM_FREQ;
  if (periodCounts == 0u) {
    return false;
  }

  while (periodCounts > 0xFFFFu) {
    prescaler++;
    periodCounts = timerClkHz / ((prescaler + 1u) * BRAKE_PWM_FREQ);
    if (periodCounts == 0u) {
      return false;
    }
  }

  htim_brake.Instance = BRAKE_PWM_TIMER;
  htim_brake.Init.Prescaler = prescaler;
  htim_brake.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_brake.Init.Period = periodCounts - 1u;
  htim_brake.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_brake.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim_brake) != HAL_OK) {
    return false;
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim_brake, &sConfigOC, BRAKE_PWM_CHANNEL) != HAL_OK) {
    return false;
  }

  if (HAL_TIM_PWM_Start(&htim_brake, BRAKE_PWM_CHANNEL) != HAL_OK) {
    return false;
  }

  pwmPeriodCounts = static_cast<uint16_t>(htim_brake.Init.Period + 1u);
  __HAL_TIM_SET_COMPARE(&htim_brake, BRAKE_PWM_CHANNEL, 0);
  return true;
}
#endif

#if defined(PWM_INPUT)
void calc_hw_pwm(void) {
  duty_ticks = pwmInput.getDutyCycleTicks();
  period_ticks = pwmInput.getPeriodTicks();

  if (period_ticks > 0u) {
    if (duty_ticks > period_ticks) {
      duty_ticks = period_ticks;
    }
    dutyPercent = (duty_ticks * 32000u) / period_ticks;
    target_current = dutyPercent - 16000u;
  } 
}
#endif

#if defined(CHECK_VBUS)
void check_vbus() {
#if defined(VOLTAGE_SENSING)
  if (vbus_adc2_ready) {
    v_bus = vbus_from_dma_counts();
  } else {
    return;
  }
#else
  v_bus = (float)supply_voltage_V;
#endif
  a = motor.Ua;
  b = motor.Ub;
  c = motor.Uc;
 // driver.voltage_power_supply = v_bus;
  //driver.voltage_limit = driver.voltage_power_supply * 0.9;

  if (v_bus > BRAKE_OVERVOLTAGE_RAMP_END_V) {
    motor.target = 0;
    motor.disable();
    v_error = 1;
     Serial.println("Overvoltage: Motor off");
  }
  if (v_bus < 12.0f) {
    motor.target = 0;
    motor.disable();
    v_error = 1;
    Serial.println("Undervoltage: Motor off");
  }
}
#endif

#if defined(BRAKE_CONTROL_ENABLED)
void brake_control(void) {
  regenCur = -currentsense.getDCCurrent(motor.electrical_angle) * 100 - MAX_REGEN_CURRENT;
  static float appliedBrakeDuty = 0.0f;
  const int16_t brkOnThresh = BRKRESACT_SENS;
  const int16_t brkOffThresh = (BRKRESACT_SENS > 1) ? (BRKRESACT_SENS / 2) : 0;
  float brakeDuty = 0.0f;
  constexpr float BRAKE_DUTY_RAMP_DOWN_STEP = 0.003f;

#if defined(BRAKE_VOLTAGE_RAMP_ENABLED)
  const bool overvoltageRequest = (v_bus > BRAKE_OVERVOLTAGE_RAMP_START_V);
#else
  const bool overvoltageRequest = false;
#endif

  if (regenCur < 0) {
    regenCur = 0;
  }

  if (!brake_active) {
    if ((regenCur > brkOnThresh) || overvoltageRequest) {
      brake_active = 1;
    }
  } else {
    if ((regenCur <= brkOffThresh) && !overvoltageRequest) {
      brake_active = 0;
    }
  }

  if (brake_active) {
    const float vbus_for_duty = (v_bus > 1.0f) ? v_bus : (float)supply_voltage_V;
    brakeDuty = ((float)regenCur * (float)BRAKE_RESISTANCE) / (vbus_for_duty * 10000.0f);

#if defined(BRAKE_VOLTAGE_RAMP_ENABLED)
    if (v_bus > BRAKE_OVERVOLTAGE_RAMP_START_V) {
      const float rampSpan = BRAKE_OVERVOLTAGE_RAMP_END_V - BRAKE_OVERVOLTAGE_RAMP_START_V;
      brakeDuty += (v_bus - BRAKE_OVERVOLTAGE_RAMP_START_V) / rampSpan;
    }
#endif
  }

  brakeDuty = CLAMP(brakeDuty, 0.0f, 0.90f);

  if (brakeDuty < appliedBrakeDuty) {
    appliedBrakeDuty -= BRAKE_DUTY_RAMP_DOWN_STEP;
    if (appliedBrakeDuty < brakeDuty) {
      appliedBrakeDuty = brakeDuty;
    }
    if (appliedBrakeDuty < 0.0f) {
      appliedBrakeDuty = 0.0f;
    }
  } else {
    appliedBrakeDuty = brakeDuty;
  }

  I_Bus = (int16_t)(appliedBrakeDuty * pwmPeriodCounts);
  __HAL_TIM_SET_COMPARE(&htim_brake, BRAKE_PWM_CHANNEL, I_Bus);
}
#endif

#if defined(BTS_BREAK)
static TIM_HandleTypeDef* getSimplefocBreakTimer() {
  if (!driver.params) {
    return nullptr;
  }

  auto* params = static_cast<STM32DriverParams*>(driver.params);
  for (int i = 0; i < 6; i++) {
    TIM_HandleTypeDef* handle = params->timers_handle[i];
    if (handle && IS_TIM_BREAK_INSTANCE(handle->Instance)) {
      return handle;
    }
  }

  return nullptr;
}

void configureBtsBreak(void) {
  TIM_HandleTypeDef* breakTimer = getSimplefocBreakTimer();
  if (!breakTimer) {
    Serial.println("No break-capable SimpleFOC timer found");
    return;
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BTS_OC_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = BTS_OC_ACTIVE_LOW ? GPIO_PULLUP : GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = BTS_OC_AF;
  HAL_GPIO_Init(BTS_OC_GPIO_PORT, &GPIO_InitStruct);

  uint32_t bdtr = breakTimer->Instance->BDTR;
  bdtr |= TIM_BDTR_BKE;
  bdtr |= TIM_BDTR_AOE;
  if (BTS_OC_ACTIVE_LOW) {
    bdtr |= TIM_BDTR_BKP;
  } else {
    bdtr &= ~TIM_BDTR_BKP;
  }
  breakTimer->Instance->BDTR = bdtr;

  __HAL_TIM_CLEAR_FLAG(breakTimer, TIM_FLAG_BREAK);
  __HAL_TIM_MOE_ENABLE(breakTimer);
}
#elif defined(BTS_OC_MONITOR)
static bool btsOcMonitorInitialized = false;
static bool btsOcReportedActive = false;

static void enableBtsOcGpioClock(void) {
#if defined(__HAL_RCC_GPIOA_CLK_ENABLE)
  if (BTS_OC_GPIO_PORT == GPIOA) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return;
  }
#endif
#if defined(__HAL_RCC_GPIOB_CLK_ENABLE)
  if (BTS_OC_GPIO_PORT == GPIOB) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return;
  }
#endif
#if defined(__HAL_RCC_GPIOC_CLK_ENABLE)
  if (BTS_OC_GPIO_PORT == GPIOC) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    return;
  }
#endif
#if defined(__HAL_RCC_GPIOD_CLK_ENABLE)
  if (BTS_OC_GPIO_PORT == GPIOD) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    return;
  }
#endif
#if defined(__HAL_RCC_GPIOE_CLK_ENABLE)
  if (BTS_OC_GPIO_PORT == GPIOE) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    return;
  }
#endif
}

void bts_oc_input_init(void) {
  enableBtsOcGpioClock();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = BTS_OC_GPIO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = BTS_OC_ACTIVE_LOW ? GPIO_PULLUP : GPIO_PULLDOWN;
  HAL_GPIO_Init(BTS_OC_GPIO_PORT, &gpio);

  btsOcMonitorInitialized = true;
  const bool activeNow = (HAL_GPIO_ReadPin(BTS_OC_GPIO_PORT, BTS_OC_GPIO_PIN) == (BTS_OC_ACTIVE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET));
  btsOcReportedActive = activeNow;
  if (activeNow) {
    Serial.println("BTS overcurrent active");
  }
}

void bts_oc_input_update(void) {
  if (!btsOcMonitorInitialized) {
    return;
  }

  const bool active = (HAL_GPIO_ReadPin(BTS_OC_GPIO_PORT, BTS_OC_GPIO_PIN) == (BTS_OC_ACTIVE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET));
  if (active && !btsOcReportedActive) {
    Serial.println("BTS overcurrent active");
    btsOcReportedActive = true;
  } else if (!active && btsOcReportedActive) {
    Serial.println("BTS overcurrent cleared");
    btsOcReportedActive = false;
  }
}
#endif

#if !defined(BTS_BREAK)
void configureBtsBreak(void) {}
#endif
