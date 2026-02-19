#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "drivers/hardware_specific/stm32/stm32_mcu.h"
#define PWM_INPUT
#if defined(PWM_INPUT)
#include "utilities/stm32pwm/STM32PWMInput.h"
#endif
#if defined(STM32G4)
#include "utilities/stm32_cordic/STM32CORDIC.h"	
#endif

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#if defined(STM32G4)
#define BRAKE_RESISTOR PB_4
#define BTS_ENABLE_PIN PA_11
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
#define currentPHA PA_1
#define currentPHC PA_2
#define ENCODER_PIN_A PB_6
#define ENCODER_PIN_B PB_7
#define MT6835_SPI_MOSI PB_5
#define MT6835_SPI_MISO PC_11
#define MT6835_SPI_SCK  PC_10
#define MT6835_SPI_CS   PA_15
#if defined(PWM_INPUT)
STM32PWMInput pwmInput = STM32PWMInput(PA3);
#endif
TIM_HandleTypeDef htim3;
static void MX_TIM3_Init(void);
#endif

#if defined(STM32F4)
#define BRAKE_RESISTOR PB_4
#define BTS_ENABLE_PIN PA_11
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
#define ENCODER_PIN_A PB_6
#define ENCODER_PIN_B PB_7
#define MT6835_SPI_MOSI PB_5
#define MT6835_SPI_MISO PC_11
#define MT6835_SPI_SCK  PC_10
#define MT6835_SPI_CS   PA_15
#if defined(PWM_INPUT)
STM32PWMInput pwmInput = STM32PWMInput(PA3);
#endif
TIM_HandleTypeDef htim3;
static void MX_TIM3_Init(void);
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
void setup(){
	//activate USB CDC driver
	
	Serial.begin(230400);
	#if defined(STM32G4)
    SimpleFOC_CORDIC_Config();      // initialize the CORDIC
	#endif
    commander.add('B', setBandwidth, "Set current control bandwidth (Hz)");
    commander.add('M', onMotor, "my motor motion");
	pinMode(FAULT_LED_PIN, OUTPUT);
    pinMode(VDO_PIN, INPUT_PULLDOWN);
	delay(1000);
	 while (digitalRead(VDO_PIN) != HIGH) {
    digitalWrite(FAULT_LED_PIN, HIGH);
    Serial.println("PSU UNDETECTED");
    delay(500); // Small delay to avoid busy-waiting
    digitalWrite(FAULT_LED_PIN, LOW);
    delay(500);
  }
  digitalWrite(FAULT_LED_PIN, HIGH);
  Serial.println("PSU DETECTED");
   motor.useMonitoring(Serial);  
   if (pwmInput.initialize() != 0) {
      Serial.println("PWM input init failed");
    }
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
}

void loop() {
	 motor.loopFOC();
     commander.run();
     motor.move();
  current_time = HAL_GetTick();


   if ((current_time - t_pwm ) >= 200){
    t_pwm  = current_time;
  //calc_hw_pwm();
  
  float degrees = encoder.getMechanicalAngle() * RAD_2_DEG;
  Serial.println("Hello world!");
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
