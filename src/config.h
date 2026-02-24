#pragma once

#include <Arduino.h>
#include "SimpleFOC.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "drivers/hardware_specific/stm32/stm32_mcu.h"
#define ONE_AXIS_VARIANT
//#define BTS_BREAK
#define PWM_INPUT
#define BRAKE_CONTROL_ENABLED
//#define BRAKE_PWM_TEST_MODE
#define VOLTAGE_SENSING
#define BRAKE_VOLTAGE_RAMP_ENABLED
#define CHECK_VBUS
//#define ESTOP_REQUIRE_HOLD
#define ESTOP_ENABLE

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
#define currentPHB _NC
#define currentPHC PA2
#define ENCODER_PIN_A PB6
#define ENCODER_PIN_B PB7
#define MT6835_SPI_MOSI PB5
#define MT6835_SPI_MISO PC11
#define MT6835_SPI_SCK  PC10
#define MT6835_SPI_CS   PA15
#define ESTOP_PORT GPIOC
#define ESTOP_PIN GPIO_PIN_13
#define ESTOP_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define ESTOP_ACTIVE_STATE GPIO_PIN_RESET
constexpr int pole_pairs = 15;
constexpr int supply_voltage_V = 24;
constexpr float ADC_REF_V = 3.3f;
#define ADC_CH_VBUS ADC_CHANNEL_1
#define ADC_CH_CURR_A ADC_CHANNEL_2
#define ADC_CH_CURR_C ADC_CHANNEL_3
#define RIGHT_TIM TIM1
#define LEFT_TIM TIM1
#define RIGHT_TIM_U CCR1
#define RIGHT_TIM_V CCR2
#define RIGHT_TIM_W CCR3
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
#define BTS_OC_AF GPIO_AF1_TIM1
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
#define ESTOP_PORT GPIOA
#define ESTOP_PIN GPIO_PIN_0
#define ESTOP_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define ESTOP_ACTIVE_STATE GPIO_PIN_RESET
constexpr int pole_pairs = 6;
constexpr int supply_voltage_V = 24;
constexpr float ADC_REF_V = 3.0f;
#define ADC_CH_VBUS ADC_CHANNEL_1
#define ADC_CH_CURR_A ADC_CHANNEL_2
#define ADC_CH_CURR_C ADC_CHANNEL_3
#define RIGHT_TIM TIM1
#define LEFT_TIM TIM1
#define RIGHT_TIM_U CCR1
#define RIGHT_TIM_V CCR2
#define RIGHT_TIM_W CCR3
#endif

#define ENCODER_PPR 16384
#define RAD_2_DEG 57.2957795131f
#define PWM_FREQ 16000
#define BRAKE_PWM_FREQ 20000
#define BRAKE_PWM_TEST_DUTY_PERCENT 25U
#define ESTOP_DEBOUNCE_MS 30U

constexpr uint32_t CONTROL_LOOP_PERIOD_US = 500;
constexpr float VBUS_RESISTOR_TOP_OHMS = 10000.0f;
constexpr float VBUS_RESISTOR_BOTTOM_OHMS = 1000.0f;
constexpr float v_bus_scale = (VBUS_RESISTOR_TOP_OHMS + VBUS_RESISTOR_BOTTOM_OHMS) / VBUS_RESISTOR_BOTTOM_OHMS;
constexpr float ADC_MAX_COUNTS = 4095.0f;
constexpr float VBUS_ADC_SCALE = ADC_REF_V / ADC_MAX_COUNTS;

#if defined(BRAKE_VOLTAGE_RAMP_ENABLED)
constexpr float BRAKE_OVERVOLTAGE_RAMP_START_V = supply_voltage_V + 1.0f;
constexpr float BRAKE_OVERVOLTAGE_RAMP_END_V = supply_voltage_V + 2.0f;
#endif

extern uint16_t brake_resistance_x100;
extern float phase_resistance;
extern float motor_KV;
extern float maxCurrent;
extern float alignStrength;

#if defined(PWM_INPUT)
extern STM32PWMInput pwmInput;
extern bool pwm_input_control_enabled;
#endif

extern float degrees;
extern float phase_inductance;
extern float current_bandwidth;
extern float a, b, c;
extern uint16_t pwmPeriodCounts;
extern uint32_t period_ticks;
extern uint32_t duty_ticks;
extern uint16_t dutyPercent;
extern int16_t target_current;
extern int16_t I_Bus;
extern bool brake_active;
extern bool break_active;
extern bool simplefoc_init;
extern bool v_error;
extern uint32_t current_time;
extern uint32_t t_pwm;
extern uint16_t max_regen_current_x100;
extern uint16_t brake_res_activation_sens_x100;
extern int16_t regenCur;
extern float v_bus;
extern bool vbus_adc2_ready;
extern float target;
extern SimpleFOCDebug debug;
extern BLDCMotor motor;
extern BLDCDriver3PWM driver;
extern LowsideCurrentSense currentsense;
extern STM32HWEncoder encoder;
extern SPIClass SPI_3;
extern SPISettings mt6835_spi_settings;
extern MagneticSensorMT6835 encoder2;
#if defined(PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF)
extern Commander commander;
#endif

void setup();
void loop();

void calc_hw_pwm(void);
#if defined(CHECK_VBUS)
void check_vbus();
#else
static inline void check_vbus() {}
#endif

#if defined(BRAKE_CONTROL_ENABLED)
void brake_control(void);
bool configureBrakePwm(void);
#endif

#if defined(BTS_BREAK)
void configureBtsBreak(void);
#endif

bool init_vbus_adc2_dma(void);
float vbus_from_dma_counts(void);
volatile uint32_t* vbus_adc2_dma_address(void);
float target_current_to_amps(int16_t target);

#if defined(PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF)
void setBandwidth(char* cmd);
void onSetABZResolution(char* cmd);
void onPWMInputControl(char* cmd);
void onMotor(char* cmd);
#endif

void estop_init(void);
void estop_update(void);
#if defined(ESTOP_ENABLE)
extern volatile uint8_t estop_flag;
extern volatile uint8_t estop_latch_flag;
static inline uint8_t estop_active(void) { return estop_flag; }
static inline uint8_t estop_latched(void) { return estop_latch_flag; }
#else
static inline uint8_t estop_active(void) { return 0U; }
static inline uint8_t estop_latched(void) { return 0U; }
#endif

void motor_characterisation(void);
void BLDC_Init(void);

#ifdef ONE_AXIS_VARIANT
/* ###### CONTROL VIA ENCODER ######
 * This variant is for using encoder for motor control.
*/
#undef MOTOR_LEFT_ENA                    // Undefines any default defines
#undef  CTRL_MOD_REQ
#undef  CTRL_TYP_SEL
#undef  DIAG_ENA
#undef BAT_CELLS
#undef  INACTIVITY_TIMEOUT
#undef N_MOT_MAX
#undef I_MOT_MAX
#undef I_DC_MAX
#undef FIELD_WEAK_ENA
#undef QP
#undef QI
#undef DP
#undef DI
////////////////////////////////////////
//#define ESTOP_REQUIRE_HOLD           // Require the button to stay pressed for the estop to remain active
#define ESTOP_ENABLE                //ESTOP functionality enabled
#define GD32F103Rx                  // define if you are using a GD32F103Rx MCU to set system clock to 108MHz  
#define HOCP                        // Tie PA6/PB12 hardware over-current signals into TIM1/TIM8 break inputs
#define BEEPER_OFF                  //use led as beeper
#define ENCODER_X                   //enable X encoder to right motor
//#define ENCODER_Y                 //enable Y encoder to left motor
#define INTBRK_L_EN                 //enable brake resistor control on PHASE A left side driver, do not disable if break reistor is connected 
//#define EXTBRK_EN                 // enable brake resistor control pin on left uart port, pick PA2 or PA3 below
#ifdef EXTBRK_EN                         
#define EXTBRK_USE_CH3              // PA2      
//#define EXTBRK_USE_CH4            // PA3
#endif

#define BAT_CELLS               12      // battery number of cells. Normal Hoverboard battery: 10s = 36V nominal, 42V full charge. For 36V battery use 10, for 24V use 6, for 48V use 13 etc.
#define N_POLE_PAIRS    6   

#define COM_CTRL        0               // [-] Commutation Control Type
#define SIN_CTRL        1               // [-] Sinusoidal Control Type
#define FOC_CTRL        2               // [-] Field Oriented Control (FOC) Type

#define OPEN_MODE       0               // [-] OPEN mode
#define VLT_MODE        1               // [-] VOLTAGE mode
#define SPD_MODE        2               // [-] SPEED mode
#define TRQ_MODE        3               // [-] TORQUE mode

// Enable/Disable Motor
#define MOTOR_LEFT_ENA                  // [-] Enable LEFT motor.  Comment-out if this motor is not needed to be operational
#define MOTOR_RIGHT_ENA                 // [-] Enable RIGHT motor. Comment-out if this motor is not needed to be operational

// Control selections
#define CTRL_TYP_SEL    FOC_CTRL        // [-] Control type selection: COM_CTRL, SIN_CTRL, FOC_CTRL (default)
#define CTRL_MOD_REQ    TRQ_MODE        // [-] Control mode request: OPEN_MODE, VLT_MODE (default), SPD_MODE, TRQ_MODE. Note: SPD_MODE and TRQ_MODE are only available for CTRL_FOC!
#define DIAG_ENA        0   
//Q axis control gains                      
#define QP              0.6f                                  //[-] P gain
#define QI              400.0f                                //[-] I gain
     
//D axis control gains
#define DP              0.6f                                   //[-] P gain   
#define DI              400.0f                                 //[-] I gain


///Dont touch
#define QaI              (float)(QI/(PWM_FREQ/3.0f))      //Integrator scaling//                     
#define DaI              (float)(DI/(PWM_FREQ/3.0f))      //Integrator scaling// 
/* Compile-time float -> fixed-point conversion helpers */
#define FIXDT_ROUND_TO_INT(x)        ((int32_t)(((x) >= 0.0f) ? ((x) + 0.5f) : ((x) - 0.5f)))
#define FIXDT_FROM_FLOAT(x, frac)    FIXDT_ROUND_TO_INT((x) * (float)(1U << (frac)))
#define FIXDT_CLAMP_U16(x)           ((uint16_t)(((x) < 0) ? 0 : (((x) > 65535) ? 65535 : (x)))) 
///End of Dont touch 
#define QP_FIXDT_12                  FIXDT_CLAMP_U16(FIXDT_FROM_FLOAT(QP, 12))
#define DP_FIXDT_12                  FIXDT_CLAMP_U16(FIXDT_FROM_FLOAT(DP, 12))
#define QaI_FIXDT_16                 FIXDT_CLAMP_U16(FIXDT_FROM_FLOAT(QaI, 16))
#define DaI_FIXDT_16                 FIXDT_CLAMP_U16(FIXDT_FROM_FLOAT(DaI, 16))

/* Unified gain macros used by BLDC_Init() */
#define CFG_CF_IDKI                  DaI_FIXDT_16
#define CFG_CF_IDKP                  DP_FIXDT_12
#define CFG_CF_IQKI                  QaI_FIXDT_16
#define CFG_CF_IQKP                  QP_FIXDT_12

#define A2BIT_CONV             50 

#if defined (INTBRK_L_EN) || defined (EXTBRK_EN)

  #define BRAKE_RESISTANCE 300                // [Ohm]3ohm X100 Value of the braking resistor. Set it to your own brake resistor resistance, increase the resistance here a bit for example I use 2.2ohm but I set to 3ohm here to be safe. 
  #define BRKRESACT_SENS    40 / 20           //[A]40mA  Brake resistor activation sensitivity. Set same as MAX_REGEN_CURRENT if using battery. If using psu set 40mA-60mA. 
  #define MAX_REGEN_CURRENT 0 / 20            // [A]0mA  Maximum regenerative current that can be dissipated in the PSU or BATTERY. Set in 20mA steps 0, 20, 40, 60, 80, 100 etc. Set 0 for PSU!

#endif  

#if defined ENCODER_X
#define ENCODER_X_PPR              16384    // Pulses per revolution
#define ALIGNMENT_X_POWER        3276      // [-] Voltage used for sensor alignment. [-16000, 16000]
#endif
#if defined ENCODER_Y
#define ENCODER_Y_PPR            2048        // Pulses per revolution 
#define ALIGNMENT_Y_POWER        3276        // [-] Voltage used for sensor alignment. [-16000, 16000]
#endif

  #define FLASH_WRITE_KEY        0x1011    // Flash memory writing key.
  
  #define CTRL_TYP_SEL           FOC_CTRL   
  #define CTRL_MOD_REQ           TRQ_MODE  
  
#define TANK_STEERING                    // Each input controls each wheel
#define HSPWM                             //Bypass PWM post proccessing for faster response
//#define MOTOR_LEFT_ENA                  //  Enable LEFT motor.  Keeping left motor disabled. This is important for breaking resistor control if connected to left side driver in place of the motor
#define MOTOR_RIGHT_ENA                 //  Enable RIGHT motor. Comment-out if this motor is not needed to be operational                        
#define DIAG_ENA                 0               // [-] disable diag if using motor at stall
#define INACTIVITY_TIMEOUT       100            // [s] Time of inactivity after which hoverboard shuts off
// Limitation settings
#define I_MOT_MAX                10              // [A] Maximum single motor current limit
#define I_DC_MAX                 13              // [A] Maximum stage2 DC Link current limit (Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
#define N_MOT_MAX                1900            // [rpm] Maximum motor speed limit


#define DC_LINK_WATCHDOG_ENABLE               //Disables the motor without warning incase of under or overvoltage, disable if using hoverboard as vehicle
#define FIELD_WEAK_ENA  0               // [-] Field Weakening / Phase Advance enable flag: 0 = Disabled (default), 1 = Enabled
#define FIELD_WEAK_MAX  5               // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed. Up to 10A has been tested using 10" wheels.
#define PHASE_ADV_MAX   25              // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
#define FIELD_WEAK_HI   1000            // (1000, 1500] Input target High threshold for reaching maximum Field Weakening / Phase Advance. Do NOT set this higher than 1500.
#define FIELD_WEAK_LO   750             // ( 500, 1000] Input target Low threshold for starting Field Weakening / Phase Advance. Do NOT set this higher than 1000.

//#define RC_PWM_RIGHT           0         // Use RC PWM as input on the RIGHT cable. (duty cycle mapped to 0 to -1000, 0, 1000) Number indicates priority for dual-input. Disable DEBUG_SERIAL_USART3!
#define HW_PWM                   0         // Set to 0 or 1 depending on which motor you want to control also Use hw pwm pin PB5 on left side L_MTR_HALL_PHA  or could also be L_MTR_HALL_PHC 
//#define CONTROL_ADC            1         // use ADC as input pn pins PA2 and PA3, cant be used with extbrk on PA2/PA3  
//#define SW_PWM_RIGHT           0         // Use PWM input capture on PB10 and PB11 (duty cycle mapped to 0 to -16000, 0, 16000)
//#define SW_PWM_LEFT            1         // Use PWM input capture on PA2 and PA3 (duty cycle mapped to 0 to -16000, 0, 16000)   (cant be use with extbrk on PA2/PA3)
//#define CONTROL_PPM_LEFT       0         // use PPM-Sum as input on the LEFT cable. Number indicates priority for dual-input. Disable DEBUG_SERIAL_USART2!
//#define PPM_NUM_CHANNELS       1         // total number of PPM channels to receive, even if they are not used.
//#define CONTROL_SERIAL_USART3  0         // left sensor board cable, disable if ADC or PPM is used! For Arduino control check the hoverSerial.ino
//#define FEEDBACK_SERIAL_USART3           // left sensor board cable, disable if ADC or PPM is used!
  #define PRI_INPUT1             0, -16000, 0, 16000,   0    //change depending on input type (may be -1000, 0, 1000 or -16000, 0, 16000)
  #define PRI_INPUT2             2, -16000, 0, 16000,   0    //change depending on input type (may be -1000, 0, 1000 or -16000, 0, 16000)
  #define RATE                   32767     //leave to max rate 32767 if you want instant response  (may be needed if you need slower response)                  
  #define FILTER                 65535     //leave to max filter 65535 if you want instant response (may be needed if input is noisy)
  //#define INVERT_R_DIRECTION             //invert right motor direction
  //#define INVERT_L_DIRECTION             //invert left motor direction
  //#define DEBUG_SERIAL_USART3            // left sensor cable debug
#endif
