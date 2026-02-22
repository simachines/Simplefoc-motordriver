#include <Arduino.h>
#include "SimpleFOC.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "drivers/hardware_specific/stm32/stm32_mcu.h"
//#define BTS_BREAK
//#define PWM_INPUT
#define BRAKE_CONTROL_ENABLED
//#define BRAKE_PWM_TEST_MODE
#define VOLTAGE_SENSING
#define BRAKE_VOLTAGE_RAMP_ENABLED
#if defined(PWM_INPUT)
#include "utilities/stm32pwm/STM32PWMInput.h"
#endif
#if defined(STM32G4)
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#endif
//#define monitoring_enabled

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#if defined(STM32G4)
#define BRAKE_RESISTOR PB4
#define BTS_ENABLE_PIN PA11
#define BTS_ENABLE BTS_ENABLE_PIN
#define PH_B PA9 
#define PH_C PA8
#define PH_A PA10
#define BTS_OC PB12
#define BTS_OC_GPIO_PORT GPIOB
#define BTS_OC_GPIO_PIN GPIO_PIN_12
#define BTS_OC_AF GPIO_AF6_TIM1
#define BTS_OC_ACTIVE_LOW false
#define FAULT_LED_PIN LED_BUILTIN
#define A_VBUS PA0
#define currentPHA PA1
#define currentPHB _NC
#define currentPHC PA2
#define ENCODER_PIN_A PB6
#define ENCODER_PIN_B PB7
#define MT6835_SPI_MOSI PB5
#define MT6835_SPI_MISO PC11
#define MT6835_SPI_SCK  PC10
#define MT6835_SPI_CS   PA15
uint16_t BRAKE_RESISTANCE = 5 * 100;   // Ohms * 100
constexpr int pole_pairs = 15;
constexpr int supply_voltage_V = 24;
constexpr float ADC_REF_V = 3.3f;
float phase_resistance = 0.3f; // Ohms
float motor_KV = _NC;
float maxCurrent = 10;
float alignStrength = 4;
#if defined(PWM_INPUT)
STM32PWMInput pwmInput = STM32PWMInput(PA3);
#endif
#if defined(BTS_BREAK)
static void configureBtsBreak(void);
#endif
#endif

#if defined(STM32F4)
#define BRAKE_RESISTOR PB14
#define BTS_ENABLE_PIN PE8
#define BTS_ENABLE BTS_ENABLE_PIN
#define PH_B PE11
#define PH_C PE13
#define PH_A PE9
#define BTS_OC PE15
#define BTS_OC_GPIO_PORT GPIOE
#define BTS_OC_GPIO_PIN GPIO_PIN_15
#define BTS_OC_AF GPIO_AF6_TIM1
#define BTS_OC_ACTIVE_LOW false
#define FAULT_LED_PIN LED_BUILTIN
#define A_VBUS PA1
#define currentPHA PA2
#define currentPHB _NC
#define currentPHC PA3
#define ENCODER_PIN_A PC6
#define ENCODER_PIN_B PB5
#define MT6835_SPI_MOSI PB15
#define MT6835_SPI_MISO PC2
#define MT6835_SPI_SCK  PB13
#define MT6835_SPI_CS   PB12
uint16_t BRAKE_RESISTANCE = 2 * 100;   // Ohms * 100
float phase_resistance = 5.0f; // Ohms
float motor_KV = 6.25f;
constexpr int pole_pairs = 6;
constexpr int supply_voltage_V = 24;
constexpr float ADC_REF_V = 3.0f;
float maxCurrent = 2;
float alignStrength = 8;
#if defined(PWM_INPUT)
STM32PWMInput pwmInput = STM32PWMInput(PE5);
#endif
#endif


#define ENCODER_PPR 16384
#define RAD_2_DEG 57.2957795131f
#define PWM_FREQ 16000 //16kHz
#define BRAKE_PWM_FREQ 20000 //20kHz
#define BRAKE_PWM_TEST_DUTY_PERCENT 25U

//Motor setup parameters
float degrees = 0;
float phase_inductance = 0.0003;
float current_bandwidth = 100; //hz
float a, b, c;
uint16_t pwmPeriodCounts = 0;
int supply_voltage_Vx10000 = supply_voltage_V * 10000;
uint32_t period_ticks = 0;
uint32_t duty_ticks = 0;
uint16_t dutyPercent = 0;
int16_t target_current = 0;
int16_t I_Bus;
bool brake_active;
bool break_active;
bool simplefoc_init=true;
bool simplefoc_init_finish=false;
bool v_error = 0;
uint32_t current_time;
uint32_t t_debug = 0;
uint32_t t_pwm = 0;
uint16_t loop_dt = 0;
uint32_t t_last_loop = 0;
uint32_t t_control_us = 0;
constexpr uint32_t CONTROL_LOOP_PERIOD_US = 500; // 2kHz
volatile uint32_t main_loop_counter = 0;
volatile uint32_t control_loop_counter = 0;
volatile uint32_t main_loop_hz = 0;
volatile uint32_t control_loop_hz = 0;
uint32_t speed_calc_last_us = 0;
uint32_t main_loop_counter_prev = 0;
uint32_t control_loop_counter_prev = 0;
uint16_t MAX_REGEN_CURRENT = 0 * 100; // Amps * 100
uint16_t BRKRESACT_SENS = 1;     // Threshold in Amps x 100
int16_t regenCur;
float v_bus = 0.00f;

#if defined(BRAKE_VOLTAGE_RAMP_ENABLED)
constexpr float BRAKE_OVERVOLTAGE_RAMP_START_V = supply_voltage_V + 0.5f;
constexpr float BRAKE_OVERVOLTAGE_RAMP_END_V = supply_voltage_V + 1.5f;
#endif
constexpr float VBUS_RESISTOR_TOP_OHMS = 10000.0f;
constexpr float VBUS_RESISTOR_BOTTOM_OHMS = 1000.0f;
constexpr float v_bus_scale = (VBUS_RESISTOR_TOP_OHMS + VBUS_RESISTOR_BOTTOM_OHMS) / VBUS_RESISTOR_BOTTOM_OHMS; // divider ratio
constexpr float ADC_MAX_COUNTS = 4095.0f;
constexpr float VBUS_ADC_SCALE = ADC_REF_V / ADC_MAX_COUNTS;
#if defined(VOLTAGE_SENSING)
static ADC_HandleTypeDef hadc2 = {0};
static DMA_HandleTypeDef hdma_adc2 = {0};
static volatile uint32_t vbus_adc2_dma_raw = 0;
#endif
static bool vbus_adc2_ready = false;
// Motor and driver objects
SimpleFOCDebug debug;
BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance, motor_KV, phase_inductance);
BLDCDriver3PWM driver = BLDCDriver3PWM((int)PH_A, (int)PH_B, (int)PH_C, (int)BTS_ENABLE);
#if defined(STM32F4)
LowsideCurrentSense currentsense = LowsideCurrentSense(0.035f, 50.0f, currentPHA, currentPHB, currentPHC);
#elif defined(STM32G4)
LowsideCurrentSense currentsense = LowsideCurrentSense(0.066f, currentPHA, currentPHB, currentPHC);
#endif
STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, ENCODER_PIN_A, ENCODER_PIN_B, _NC);
SPIClass SPI_3((int)MT6835_SPI_MOSI, (int)MT6835_SPI_MISO, (int)MT6835_SPI_SCK);
SPISettings mt6835_spi_settings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 encoder2 = MagneticSensorMT6835((int)MT6835_SPI_CS, mt6835_spi_settings);
Commander commander = Commander(Serial);
void calc_hw_pwm();
void check_vbus();
#if defined(BRAKE_CONTROL_ENABLED)
 void brake_control(void);
 static bool configureBrakePwm(void);
#endif
#if defined(VOLTAGE_SENSING)
static bool init_vbus_adc2_dma(void);
static inline float vbus_from_dma_counts(void);
static inline volatile uint32_t* vbus_adc2_dma_address(void);
#endif
static inline float target_current_to_amps(int16_t target);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }

static inline uint32_t get_systick_time_us() {
  const uint32_t reload = SysTick->LOAD + 1u;
  uint32_t ms_start = 0;
  uint32_t ms_end = 0;
  uint32_t ticks = 0;

  do {
    ms_start = HAL_GetTick();
    ticks = SysTick->VAL;
    ms_end = HAL_GetTick();
  } while (ms_start != ms_end);

  const uint32_t sub_ms_us = ((reload - ticks) * 1000u) / reload;
  return (ms_start * 1000u) + sub_ms_us;
}

#if defined(VOLTAGE_SENSING)
static bool init_vbus_adc2_dma(void) {
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

static inline float vbus_from_dma_counts(void) {
  const float pin_voltage = static_cast<float>(vbus_adc2_dma_raw & 0x0FFFu) * VBUS_ADC_SCALE;
  return pin_voltage * v_bus_scale;
}

static inline volatile uint32_t* vbus_adc2_dma_address(void) {
  return &vbus_adc2_dma_raw;
}
#endif

static inline float target_current_to_amps(int16_t target) {
  constexpr float TARGET_INPUT_MAX = 16000.0f;
  float normalized = static_cast<float>(target) / TARGET_INPUT_MAX;
  normalized = CLAMP(normalized, -1.0f, 1.0f);
  return normalized * maxCurrent;
}

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
  const bool onApb2 = true;  // TIM16 is on APB2
#else
  const bool onApb2 = false; // TIM12 is on APB1
#endif

  const uint32_t pclk = onApb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  const uint32_t divider = onApb2 ? clkInit.APB2CLKDivider : clkInit.APB1CLKDivider;
  return (divider == RCC_HCLK_DIV1) ? pclk : (pclk * 2u);
}

static bool configureBrakePwm(void) {
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

void motor_characterisation(void){
void characteriseMotor(float alignStrength);
  // Access phase resistance (already stored)
float R = motor.phase_resistance;

// Access inductance values (v2.4.0+)
float L_d = motor.axis_inductance.d;  // D-axis inductance
float L_q = motor.axis_inductance.q;  // Q-axis inductance

Serial.print("Resistance: ");
Serial.print(R);
Serial.println(" Ohms");

Serial.print("D-axis inductance: ");
Serial.print(L_d * 1000);  // Convert H to mH
Serial.println(" mH");

Serial.print("Q-axis inductance: ");
Serial.print(L_q * 1000);  // Convert H to mH
Serial.println(" mH");
}

void setup(){

	Serial.begin(921600);
  debug.enable();
  delay(3000);
	#if defined(STM32G4)
    SimpleFOC_CORDIC_Config();      // initialize the CORDIC
	#endif
    commander.add('B', setBandwidth, "Set current control bandwidth (Hz)");
    commander.add('M', onMotor, "my motor motion");
	pinMode(FAULT_LED_PIN, OUTPUT);
#if defined(VOLTAGE_SENSING)
  vbus_adc2_ready = init_vbus_adc2_dma();
  if (!vbus_adc2_ready) {
    Serial.println("VBUS ADC2 init failed");
  } else {
    Serial.printf("VBUS ADC2 DMA addr: 0x%08lX\n", (uint32_t)vbus_adc2_dma_address());
  }
#else
  vbus_adc2_ready = false;
  v_bus = (float)supply_voltage_V;
#endif

  #if defined(PWM_INPUT)
   if (pwmInput.initialize() != 0) {
      Serial.println("PWM input init failed");
    }
    #endif
  currentsense.gain_a *= -1;
  currentsense.gain_c *= -1;
  driver.voltage_power_supply = supply_voltage_V;  // Convert mV to V for driver
  driver.voltage_limit = driver.voltage_power_supply*0.9;
  driver.pwm_frequency = PWM_FREQ;
  driver.enable_active_high = false;
   if (!driver.init()){
    simplefoc_init=false;
    Serial.printf("Driver init failed!\n");
    return;
  }
   encoder.init();
   Serial.printf("Encoder init status: %d\n", encoder.initialized);
  #if defined(BTS_BREAK)
  configureBtsBreak();
  #endif
  #if defined(BRAKE_CONTROL_ENABLED)
  if (!configureBrakePwm()) {
    Serial.println("Brake PWM init failed");
  }
  #if defined(BRAKE_PWM_TEST_MODE)
  else {
    const uint32_t testDuty = (static_cast<uint32_t>(pwmPeriodCounts) * BRAKE_PWM_TEST_DUTY_PERCENT) / 100u;
    __HAL_TIM_SET_COMPARE(&htim_brake, BRAKE_PWM_CHANNEL, testDuty);
    Serial.printf("BRAKE PWM TEST MODE: %u%% duty\n", BRAKE_PWM_TEST_DUTY_PERCENT);
  }
  #endif
  #endif

  motor.linkSensor(&encoder); 
  motor.linkDriver(&driver); 
  motor.linkCurrentSense(&currentsense);
  currentsense.linkDriver(&driver);

  #if defined(VOLTAGE_SENSING)
    v_bus = vbus_adc2_ready ? vbus_from_dma_counts() : 0.0f;
  	 while (vbus_adc2_ready && (v_bus < supply_voltage_V - 1.0f  || v_bus > supply_voltage_V + 1.0f)) {
    digitalWrite(FAULT_LED_PIN, HIGH);
    Serial.printf("PSU UNDER/OVER VOLTAGE: %.2f V\n", v_bus);
    delay(250); // Small delay to avoid busy-waiting
    digitalWrite(FAULT_LED_PIN, LOW);
    delay(250);
    v_bus = vbus_adc2_ready ? vbus_from_dma_counts() : 0.0f;
  }
  #else
    v_bus = (float)supply_voltage_V;
  #endif
  digitalWrite(FAULT_LED_PIN, HIGH);
  Serial.printf("PSU NOMINAL: %.2f V\n", v_bus);
   
    int cs_init = currentsense.init();
  Serial.printf("Current sense init status: %d\n", cs_init);
  #if (monitoring_enabled)
  motor.useMonitoring(Serial); 
  #endif 
  //motor.controller = MotionControlType::torque;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::estimated_current;
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
  motor.monitor_variables = _MON_CURR_Q | _MON_TARGET | _MON_CURR_D;
  
  motor.modulation_centered = 1;
 
  int m_init = motor.init();
  Serial.printf("Motor init status: %d\n", m_init);

  currentsense.skip_align = true; // before initFOC()

  motor_characterisation();
  motor_characterisation();
  motor_characterisation();

  int foc_init = motor.initFOC();
  Serial.printf("FOC init status: %d\n", foc_init);
}

void loop() {
	 main_loop_counter++;
	 motor.loopFOC();
     commander.run();
  motor.move(target_current_to_amps(-target_current));
     check_vbus();
  const uint32_t now_us = get_systick_time_us();
  if ((uint32_t)(now_us - t_control_us) >= CONTROL_LOOP_PERIOD_US) {
    t_control_us += CONTROL_LOOP_PERIOD_US;
    control_loop_counter++;
    #if (monitoring_enabled)
    motor.monitor();
    #endif
  #if defined(PWM_INPUT)
    calc_hw_pwm();
  #endif
  #if defined(BRAKE_CONTROL_ENABLED)
    #if defined(BRAKE_PWM_TEST_MODE)
    const uint32_t testDuty = (static_cast<uint32_t>(pwmPeriodCounts) * BRAKE_PWM_TEST_DUTY_PERCENT) / 100u;
    __HAL_TIM_SET_COMPARE(&htim_brake, BRAKE_PWM_CHANNEL, testDuty);
    #else
    brake_control();
    #endif
  #endif
    
    
    degrees = encoder.getMechanicalAngle() * RAD_2_DEG;
  }

  //current_time = now_us / 1000u;
  // if ((current_time - t_pwm ) >= 200){
   // t_pwm  = current_time;
  //Serial.println("Hello world!");
  //motor.move(target_current);
  //}

  if ((uint32_t)(now_us - speed_calc_last_us) >= 1000000u) {
    const uint32_t dt_us = now_us - speed_calc_last_us;
    const uint32_t main_delta = main_loop_counter - main_loop_counter_prev;
    const uint32_t control_delta = control_loop_counter - control_loop_counter_prev;

    main_loop_hz = (uint32_t)(((uint64_t)main_delta * 1000000ull) / dt_us);
    control_loop_hz = (uint32_t)(((uint64_t)control_delta * 1000000ull) / dt_us);

    main_loop_counter_prev = main_loop_counter;
    control_loop_counter_prev = control_loop_counter;
    speed_calc_last_us = now_us;
  }

}
#if defined(PWM_INPUT)
void calc_hw_pwm(void){
  /* Read capture registers directly from TIM3 handle (no IRQ required)*/
  duty_ticks = pwmInput.getDutyCycleTicks();
  period_ticks  =  pwmInput.getPeriodTicks(); 
  
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
    else {
  target_current = 0;
  }
}
#endif

void check_vbus() {
#if defined(VOLTAGE_SENSING)
  if (vbus_adc2_ready) {
    v_bus = vbus_from_dma_counts();
  }
  else {
    return;
  }
#else
  v_bus = (float)supply_voltage_V;
#endif
a = motor.Ua;
b = motor.Ub;
c = motor.Uc;
  driver.voltage_power_supply = v_bus;
  driver.voltage_limit = driver.voltage_power_supply*0.5;
  motor.voltage_limit = driver.voltage_power_supply *0.58f;
  
  if (v_bus > 26.0f) {
    motor.target = 0;
    motor.disable();
    v_error = 1;
    Serial.printf("Overvoltage: Motor off\n");
  }/*
  if (v_bus < 20.0f) {
    motor.target = 0;
    motor.disable();
    v_error = 1;
    Serial.printf("Undervoltage: Motor off\n");
  }
  return v_error;
   */
}

#if defined(BRAKE_CONTROL_ENABLED)

 void brake_control(void){
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
      if ((regenCur < brkOffThresh) && !overvoltageRequest) {
        brake_active = 0;
      }
    }



    if (brake_active) {
      const float vbus_for_duty = (v_bus > 1.0f) ? v_bus : (float)supply_voltage_V;
      brakeDuty = ((float)regenCur * (float)BRAKE_RESISTANCE) / (vbus_for_duty * 10000.0f);

#if defined(BRAKE_VOLTAGE_RAMP_ENABLED)
      if ( v_bus > BRAKE_OVERVOLTAGE_RAMP_START_V) {
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

static void configureBtsBreak(void){

  TIM_HandleTypeDef* breakTimer = getSimplefocBreakTimer();
  if (!breakTimer) {
    Serial.println("No break-capable SimpleFOC timer found");
    return;
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BTS_OC_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
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


}
static bool isBreakActive(void){
  TIM_HandleTypeDef* breakTimer = getSimplefocBreakTimer();
  if (!breakTimer) {
    return false;
  }
  // MOE is forced low when the timer is broken and stays low until software re-enables the bridge
  return (breakTimer->Instance->BDTR & TIM_BDTR_MOE) == 0;
}

static void handleFaultLed(void){
  if (isBreakActive()) {
    break_active = true;
  } else {
    break_active = false;
  }
}
#endif