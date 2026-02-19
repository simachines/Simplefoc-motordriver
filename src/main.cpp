#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)
#include "drivers/hardware_specific/stm32/stm32_mcu.h"

#endif

//#if defined(USBCON)
//#define Serial SerialUSB
//#endif

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define BRAKE_RESISTOR PB_4
#ifndef BTS_ENABLE_PIN
#define BTS_ENABLE_PIN PA_11
#endif
#define BTS_ENABLE BTS_ENABLE_PIN
#define PH_B PA_9 
#define PH_C PA_8
#define PH_A PA_10
#define BTS_OC PB_12
#define BTS_OC_GPIO_PORT GPIOB
#define BTS_OC_GPIO_PIN GPIO_PIN_12
#define BTS_OC_AF GPIO_AF6_TIM1
#define BTS_OC_ACTIVE_LOW false
#define FAULT_LED_PIN LED_BUILTIN
#define VDO_PIN PA_0
#define currentPHA PA_2
#define currentPHC PA_3

//Encoder setup parameters
#define ENCODER_PPR 16384
#define ENCODER_PIN_A PB_6
#define ENCODER_PIN_B PB_7
#define MT6835_SPI_MOSI PB_5
#define MT6835_SPI_MISO PC_11
#define MT6835_SPI_SCK  PC_10
#define MT6835_SPI_CS   PA_15
#define RAD_2_DEG 57.2957795131f
#define PWM_FREQ 16000 //16kHz
//Motor setup parameters
constexpr int pole_pairs = 15;
float phase_resistance = 0.6;
float phase_inductance = 0.0003;
float motor_KV = _NC;
float maxCurrent = 10;
float alignStrength = 4;
float current_bandwidth = 100; //hz

#if defined(TIM15)
TIM_HandleTypeDef htim15;
#endif
TIM_HandleTypeDef htim3;
uint16_t pwmPeriodCounts = 0;

static void MX_TIM3_Init(void);
//static void MX_TIM15_Init(void);
#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)
static TIM_HandleTypeDef* simplefoc_getBreakTimer(void);
static void enableGpioPortClock(GPIO_TypeDef* port);
#endif
//static void configureBtsBreak(void);
//static void configureUnusedPins(void);
static void handleFaultLed(void);
static bool isBreakActive(void);
static void blinkFaultLed(void);
 void calc_hw_pwm(void);
 //void loop_time(void);
 void brake_control(void);

// Voltage monitoring variables
uint16_t supply_voltage_V = 24;
uint16_t supply_voltage_Vx10000 = supply_voltage_V * 10000;
uint32_t period_ticks = 0;
uint32_t duty_ticks = 0;
uint16_t dutyPercent = 0;
uint16_t target_current = 0;
int16_t I_Bus;
bool brake_active;
bool simplefoc_init=true;
bool simplefoc_init_finish=false;
u_int32_t current_time;
uint32_t t_debug = 0;
uint32_t t_pwm = 0;
uint16_t loop_dt = 0;
uint32_t t_last_loop = 0;

uint16_t MAX_REGEN_CURRENT = 0 * 100; // Amps * 100
uint16_t BRKRESACT_SENS = 1;     // Threshold in Amps x 100
uint16_t BRAKE_RESISTANCE = 5 * 100;   // Ohms * 100
// Motor and driver objects
SimpleFOCDebug debug;

BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance, motor_KV, phase_inductance);
BLDCDriver3PWM driver = BLDCDriver3PWM((int)PH_A, (int)PH_B, (int)PH_C, (int)BTS_ENABLE);
LowsideCurrentSense current_sense = LowsideCurrentSense(66.0f, (int)currentPHA, _NC, (int)currentPHC);
SPIClass SPI_3((int)MT6835_SPI_MOSI, (int)MT6835_SPI_MISO, (int)MT6835_SPI_SCK);
SPISettings mt6835_spi_settings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 encoder = MagneticSensorMT6835((int)MT6835_SPI_CS, mt6835_spi_settings);

Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }

void setBandwidth(char* cmd) {
  float new_bandwidth = current_bandwidth;  // Default to current value
  commander.scalar(&new_bandwidth, cmd);
  
  if (new_bandwidth > 0 && new_bandwidth <= 1000) {  // Reasonable limits
    current_bandwidth = new_bandwidth;
    
    // Update the PID parameters with new bandwidth
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

void setup() {
   //Wait for PSU to turn on
   //configureBtsBreak();
  //configureUnusedPins();
  //MX_TIM3_Init();
  //MX_TIM2_Init();

  Serial.begin(230400);
    debug.enable();
  //SimpleFOC_CORDIC_Config();      // initialize the CORDIC
  // Commander setup
  commander.add('B', setBandwidth, "Set current control bandwidth (Hz)");
  commander.add('M', onMotor, "my motor motion");
  pinMode(FAULT_LED_PIN, OUTPUT);
  pinMode(VDO_PIN, INPUT_PULLDOWN);
  delay(1000);

  while (digitalRead(VDO_PIN) == HIGH) {
    digitalWrite(FAULT_LED_PIN, HIGH);
    Serial.println("PSU UNDETECTED");
    delay(500); // Small delay to avoid busy-waiting
    digitalWrite(FAULT_LED_PIN, LOW);
    delay(500);
  }
  digitalWrite(FAULT_LED_PIN, HIGH);
  Serial.println("PSU DETECTED");

  // monitoring port
  motor.useMonitoring(Serial);
  
current_sense.gain_a *= -1;
current_sense.gain_c *= -1;
 
  driver.voltage_power_supply = supply_voltage_V;  // Convert mV to V for driver
  driver.voltage_limit = driver.voltage_power_supply;
  driver.pwm_frequency = PWM_FREQ;
  driver.enable_active_high = false;

  
  if (!driver.init()){
    simplefoc_init=false;
    Serial.printf("Driver init failed!\n");
    return;
  }
  current_sense.linkDriver(&driver);
  current_sense.init();

  // Motion control configuration
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

// Limits and parameters
  motor.PID_current_d.P = phase_inductance*current_bandwidth*_2PI;
  motor.PID_current_d.I = motor.PID_current_d.P*phase_resistance/phase_inductance;
  motor.PID_current_d.D = 0;
  motor.PID_current_d.output_ramp = 0;
  motor.LPF_current_d.Tf = 1/(_2PI*3.0f*current_bandwidth);

  motor.PID_current_q.P = phase_inductance*current_bandwidth*_2PI;
  motor.PID_current_q.I = motor.PID_current_q.P*phase_resistance/phase_inductance;
  motor.PID_current_q.D = 0;
  motor.PID_current_q.output_ramp = 0;
  motor.LPF_current_q.Tf = 1/(_2PI*3.0f*current_bandwidth);

  motor.voltage_limit = driver.voltage_power_supply * 0.58f;
  //motor.motion_downsample = downsample;
  motor.current_limit = maxCurrent;
  motor.phase_resistance = phase_resistance;
  motor.voltage_sensor_align = alignStrength;
  motor.monitor_downsample = 100;
  motor.monitor_variables = _MON_CURR_Q | _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
  motor.linkDriver(&driver); 
  motor.linkCurrentSense(&current_sense);
  motor.modulation_centered = 1;
  motor.init();
  motor.initFOC();
  encoder.init(&SPI_3);
  motor.linkSensor(&encoder);  
  
  if (!motor.initFOC()){
    simplefoc_init=false;
    Serial.printf("initFOC failed!\n");
    return;
  }

   int m_init = motor.init();
  Serial.printf("Motor init status: %d\n", m_init);

  int foc_init = motor.initFOC();
  Serial.printf("FOC init status: %d\n", foc_init);

  /* 
  if (simplefoc_init){
    simplefoc_init_finish=true;
    Serial.printf("SimpleFOC initialization complete.\n");
  motor.enable();    
  }
  else{
    motor.disable();
    Serial.printf("SimpleFOC initialization failed.\n");
  }
  */
}

void loop() {
  motor.loopFOC();
  commander.run();
  current_time = HAL_GetTick();


   if ((current_time - t_pwm ) >= 1){
    t_pwm  = current_time;
  //calc_hw_pwm();
  motor.move();
  float degrees = encoder.getMechanicalAngle() * RAD_2_DEG;
  //Serial.print(degrees);
  //motor.move(target_current);
  }

  //handleFaultLed();
  //Serial.print(degrees);
  //Serial.print("\t");
  //Serial.println(encoder.getVelocity());

  //loop_time();
  //if (simplefoc_init_finish){
   //brake_control();
  //}
 
  
}


/*
void loop_time(void){
loop_dt = current_time - t_last_loop;
t_last_loop = current_time;
if (current_time - t_debug >= 100) {
t_debug = current_time;
 Serial.print("loop time: ");
 Serial.print(loop_dt);
  }
}
  
void brake_control(void){
       I_Bus = -current_sense.getDCCurrent(motor.electrical_angle) * 100 - MAX_REGEN_CURRENT; // Negate to flip polarity
    if (I_Bus > BRKRESACT_SENS){ // If over max regen current
    brake_active = CLAMP(((((int32_t)I_Bus * BRAKE_RESISTANCE * pwmPeriodCounts) /(supply_voltage_Vx10000))) ,0, ((pwmPeriodCounts*90)/100));
     TIM3->CCR1 = brake_active;
    }else{
    brake_active = false;
     TIM3->CCR1 = brake_active;
    }
 }

static void MX_TIM3_Init(void){
  __HAL_RCC_TIM3_CLK_ENABLE();
                               
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

 uint32_t timClockHz = HAL_RCC_GetPCLK1Freq();
 /* TIM3 is on APB1. If APB1 prescaler != 1, timer clock is PCLK1*2 
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
  timClockHz *= 2U;
 }

 /* period counts = timer clock / PWM freq (no extra /2) 
 pwmPeriodCounts = timClockHz / PWM_FREQ;
 if (pwmPeriodCounts == 0U) {
  pwmPeriodCounts = 1U;
 }

 
  /* USER CODE BEGIN TIM3_Init 1 
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END TIM3_Init 1
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = pwmPeriodCounts;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim3);
 
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim3);
 
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_NONE;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
 
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 }
#if defined(TIM15)
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0

  /* USER CODE END TIM15_Init 0 

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN TIM15_Init 1
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END TIM15_Init 1 
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2

}
#endif
void calc_hw_pwm(void){
  /* Read capture registers directly from TIM3 handle (no IRQ required)
#if defined(TIM15)
  duty_ticks = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_2);   // CH2 captures high time (falling edge)
  period_ticks = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1); // CH1 captures period (rising edge)
  //hw_pwm_ready = 0;
  if (period_ticks > 0u) {
      if (duty_ticks > period_ticks) {
        duty_ticks = period_ticks;
      }
      dutyPercent  = (duty_ticks * 32000u) / period_ticks;
        target_current = dutyPercent - 16000u; 

        //duty_scaled = (duty_ticks * 32000u) / period_ticks;
        //pwmduty = duty_scaled - 16000u;             /* Center
       
    }
#else
  duty_ticks = 0;
  period_ticks = 0;
  dutyPercent = 0;
  target_current = 0;
#endif
}

#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)
static TIM_HandleTypeDef* simplefoc_getBreakTimer(void){
  if (!driver.params) {
    return nullptr;
  }
  auto params = static_cast<STM32DriverParams*>(driver.params);
  for (int i = 0; i < 6; i++) {
    TIM_HandleTypeDef* handle = params->timers_handle[i];
    if (handle && IS_TIM_BREAK_INSTANCE(handle->Instance)) {
      return handle;
    }
  }
  return nullptr;
}

static void enableGpioPortClock(GPIO_TypeDef* port){
  if (port == GPIOA) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  } else if (port == GPIOB) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  } else if (port == GPIOC) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  } else if (port == GPIOD) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  } else if (port == GPIOE) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  }
#if defined(GPIOF)
  else if (port == GPIOF) {
    __HAL_RCC_GPIOF_CLK_ENABLE();
  }
#endif
#if defined(GPIOG)
  else if (port == GPIOG) {
    __HAL_RCC_GPIOG_CLK_ENABLE();
  }
#endif
#if defined(GPIOH)
  else if (port == GPIOH) {
    __HAL_RCC_GPIOH_CLK_ENABLE();
  }
#endif
#if defined(GPIOI)
  else if (port == GPIOI) {
    __HAL_RCC_GPIOI_CLK_ENABLE();
  }
#endif
}
#endif
/*
static void configureBtsBreak(void){
#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)
  TIM_HandleTypeDef* breakTimer = simplefoc_getBreakTimer();
  if (!breakTimer) {
    return;
  }

  enableGpioPortClock(BTS_OC_GPIO_PORT);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BTS_OC_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = BTS_OC_ACTIVE_LOW ? GPIO_PULLUP : GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = BTS_OC_AF;
  HAL_GPIO_Init(BTS_OC_GPIO_PORT, &GPIO_InitStruct);

  uint32_t bdtr = breakTimer->Instance->BDTR;
  bdtr |= TIM_BDTR_BKE;
  if (BTS_OC_ACTIVE_LOW) {
    bdtr |= TIM_BDTR_BKP;
  } else {
    bdtr &= ~TIM_BDTR_BKP;
  }
  breakTimer->Instance->BDTR = bdtr;
#endif
}
 
#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)
struct IdlePin {
  GPIO_TypeDef* port;
  uint16_t pin;
};

#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)
static bool isBreakActive(void){
  TIM_HandleTypeDef* breakTimer = simplefoc_getBreakTimer();
  if (!breakTimer) {
    return false;
  }
  // MOE is forced low when the timer is broken and stays low until software re-enables the bridge
  return (breakTimer->Instance->BDTR & TIM_BDTR_MOE) == 0;
}

static void blinkFaultLed(void){
  const uint32_t blinkIntervalMs = 250;
  static uint32_t lastToggle = 0;
  static bool ledOn = false;
  uint32_t now = HAL_GetTick();
  if ((now - lastToggle) >= blinkIntervalMs) {
    lastToggle = now;
    ledOn = !ledOn;
    digitalWrite(FAULT_LED_PIN, ledOn ? HIGH : LOW);
  }
}
#else
static bool isBreakActive(void){
  return false;
}

static void blinkFaultLed(void){}
#endif

static void handleFaultLed(void){
#if defined(_STM32_DEF_) || defined(TARGET_STM32H7)
  if (isBreakActive()) {
    blinkFaultLed();
  } else {
    digitalWrite(FAULT_LED_PIN, HIGH);
  }
#else
  digitalWrite(FAULT_LED_PIN, HIGH);
#endif
}*/