#include <Arduino.h>
#include "SimpleFOC.h"
//#include "SimpleFOCDrivers.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "drivers/hardware_specific/stm32/stm32_mcu.h"
//#define BTS_BREAK
//#define PWM_INPUT
//#define BRAKE_CONTROL_ENABLED
#if defined(PWM_INPUT)
#include "utilities/stm32pwm/STM32PWMInput.h"
#endif
#if defined(STM32G4)
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#endif

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
#define currentPHC PA2
#define ENCODER_PIN_A PB6
#define ENCODER_PIN_B PB7
#define MT6835_SPI_MOSI PB5
#define MT6835_SPI_MISO PC11
#define MT6835_SPI_SCK  PC10
#define MT6835_SPI_CS   PA15
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
#define currentPHC PA3
#define ENCODER_PIN_A PC6
#define ENCODER_PIN_B PB5
#define MT6835_SPI_MOSI PB15
#define MT6835_SPI_MISO PC2
#define MT6835_SPI_SCK  PB13
#define MT6835_SPI_CS   PB12
#if defined(PWM_INPUT)
STM32PWMInput pwmInput = STM32PWMInput(PB14);
#endif


#endif


#define ENCODER_PPR 16384
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

uint16_t pwmPeriodCounts = 0;
uint16_t supply_voltage_V = 24;
uint16_t supply_voltage_Vx10000 = supply_voltage_V * 10000;
uint32_t period_ticks = 0;
uint32_t duty_ticks = 0;
uint16_t dutyPercent = 0;
uint16_t target_current = 0;
int16_t I_Bus;
bool brake_active;
bool break_active;
bool simplefoc_init=true;
bool simplefoc_init_finish=false;
bool v_error = 0;
u_int32_t current_time;
uint32_t t_debug = 0;
uint32_t t_pwm = 0;
uint16_t loop_dt = 0;
uint32_t t_last_loop = 0;
uint16_t MAX_REGEN_CURRENT = 0 * 100; // Amps * 100
uint16_t BRKRESACT_SENS = 1;     // Threshold in Amps x 100
uint16_t BRAKE_RESISTANCE = 5 * 100;   // Ohms * 100
constexpr float VBUS_RESISTOR_TOP_OHMS = 10000.0f;
constexpr float VBUS_RESISTOR_BOTTOM_OHMS = 1000.0f;
constexpr float v_bus_scale = (VBUS_RESISTOR_TOP_OHMS + VBUS_RESISTOR_BOTTOM_OHMS) / VBUS_RESISTOR_BOTTOM_OHMS; // divider ratio
constexpr float ADC_REF_V = 3.3f;
constexpr float ADC_MAX_COUNTS = 4095.0f;
constexpr float VBUS_ADC_SCALE = ADC_REF_V * ADC_MAX_COUNTS;
// Motor and driver objects
SimpleFOCDebug debug;
BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance, motor_KV, phase_inductance);
BLDCDriver3PWM driver = BLDCDriver3PWM((int)PH_A, (int)PH_B, (int)PH_C, (int)BTS_ENABLE);
LowsideCurrentSense current_sense = LowsideCurrentSense(66.0f, (int)currentPHA, _NC, (int)currentPHC);
STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, ENCODER_PIN_A, ENCODER_PIN_B, _NC);
SPIClass SPI_3((int)MT6835_SPI_MOSI, (int)MT6835_SPI_MISO, (int)MT6835_SPI_SCK);
SPISettings mt6835_spi_settings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 encoder2 = MagneticSensorMT6835((int)MT6835_SPI_CS, mt6835_spi_settings);
Commander commander = Commander(Serial);
void calc_hw_pwm();
void check_vbus();
#if defined(BRAKE_CONTROL_ENABLED)
 void brake_control(void);
#endif
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
void setup(){

	Serial.begin(230400);
  delay(3000);
	#if defined(STM32G4)
    SimpleFOC_CORDIC_Config();      // initialize the CORDIC
	#endif
    commander.add('B', setBandwidth, "Set current control bandwidth (Hz)");
    commander.add('M', onMotor, "my motor motion");
	pinMode(FAULT_LED_PIN, OUTPUT);
  pinMode(A_VBUS, INPUT_ANALOG);
	
  #if defined(PWM_INPUT)
   if (pwmInput.initialize() != 0) {
      Serial.println("PWM input init failed");
    }
    #endif
  current_sense.gain_a *= -1;
  current_sense.gain_c *= -1;
  driver.voltage_power_supply = supply_voltage_V;  // Convert mV to V for driver
  driver.voltage_limit = driver.voltage_power_supply*0.9;
  driver.pwm_frequency = PWM_FREQ;
  driver.enable_active_high = false;
   if (!driver.init()){
    simplefoc_init=false;
    Serial.printf("Driver init failed!\n");
    return;
  }
  #if defined(BTS_BREAK)
  configureBtsBreak();
  #endif

  motor.linkSensor(&encoder); 
  motor.linkDriver(&driver); 
  motor.linkCurrentSense(&current_sense);
  current_sense.linkDriver(&driver);
  current_sense.init();
  
 float v_bus = analogRead(A_VBUS) * VBUS_ADC_SCALE * v_bus_scale;
	 while (v_bus < 23) {
    digitalWrite(FAULT_LED_PIN, HIGH);
    Serial.printf("PSU UNDETECTED : %.5f V\n", v_bus);
    delay(500); // Small delay to avoid busy-waiting
    digitalWrite(FAULT_LED_PIN, LOW);
    delay(500);
    v_bus = analogRead(A_VBUS) * VBUS_ADC_SCALE * v_bus_scale;
  }
  digitalWrite(FAULT_LED_PIN, HIGH);
  Serial.printf("PSU DETECTED: %.5f V\n", v_bus);
  motor.useMonitoring(Serial);  
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
  motor.modulation_centered = 1;
 
  int m_init = motor.init();
  Serial.printf("Motor init status: %d\n", m_init);

  int foc_init = motor.initFOC();
  Serial.printf("FOC init status: %d\n", foc_init);
}

void loop() {
	 motor.loopFOC();
     commander.run();
     motor.move();
  current_time = HAL_GetTick();


   if ((current_time - t_pwm ) >= 200){
    t_pwm  = current_time;
    #if defined(PWM_INPUT)
  calc_hw_pwm();
  #endif
  check_vbus();
  float degrees = encoder.getMechanicalAngle() * RAD_2_DEG;
  //Serial.println("Hello world!");
  //motor.move(target_current);
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
  duty_ticks = 0;
  period_ticks = 0;
  dutyPercent = 0;
  target_current = 0;
  }
#endif

void check_vbus() {
  float v_bus = _readADCVoltageLowSide(A_VBUS,current_sense.params)*v_bus_scale;
  //
  driver.voltage_power_supply = v_bus;
  driver.voltage_limit = driver.voltage_power_supply*0.9;
  motor.voltage_limit = driver.voltage_power_supply *0.58f;
  /*
  if (v_bus > 26.0f) {
    motor.target = 0;
    motor.disable();
    v_error = 1;
    Serial.printf("Overvoltage: Motor off\n");
  }
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
    static int16_t regenCur = -current_sense.getDCCurrent(motor.electrical_angle) * 100 - MAX_REGEN_CURRENT;;
    const int16_t brkOnThresh = BRKRESACT_SENS;
    const int16_t brkOffThresh = (BRKRESACT_SENS > 1) ? (BRKRESACT_SENS / 2) : 0;

    if (regenCur < 0) {
      regenCur = 0;
    }

    if (!brake_active) {
      if (regenCur > brkOnThresh) {
        brake_active = 1;
      }
    } else {
      if (regenCur < brkOffThresh) {
        brake_active = 0;
      }
    }

    if (brake_active) {
      I_Bus = CLAMP(((((int32_t)regenCur * BRAKE_RESISTANCE * pwmPeriodCounts) /(supply_voltage_Vx10000))) ,0, (((uint32_t)pwmPeriodCounts*90)/100));
      TIM16->CCR1 = I_Bus;
    }else{
     TIM16->CCR1 = I_Bus;
    }
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