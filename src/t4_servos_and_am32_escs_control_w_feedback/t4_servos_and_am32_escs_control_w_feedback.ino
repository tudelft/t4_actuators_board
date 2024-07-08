/* 
  This software is and an embedded application to run on a Teensy 4.0 or 4.1 board.
  Hardware to actuate Serial Bus Servos, PWM Servos and ESCs and return Force, RPM, Angle and much more as realtime return information.
  This current version, can at very high speed, actuate a maximum 12 Servos, 10 SerialBus and 2 PWM and additionally command 4 ESC's.  

  The ESC telemetry should be the KISS compatible telemetry, Opensource AM32 ESC firmware can deliver that reliably.

  COMPILE:
  Install the Arduino IDE and Teensyduino board support to compile and upload this code to your Teensy based actuator controller board.

  NOTES:
  We had unreliability issues with ESCs running BLHeli_32 firmware, lucky most of those ESCs can be flashed with AM32
 
  There is an option to send a mode "Kill" to your servos, this feature exists so one can prevent broken servo gears in very specific scenarios.
  
  Thanks to Arda Yiğit and Jacques Gangloff for their DSHOT and ESCCMD libraries, they are awesome!

  Indeed surprisingly we only have 24H a day also, and  all the copy/paste repeated code is not super, but it works.
  Even as it is a working solution, would be great if you help to make it better wherever possible.

*/

#include <stdint.h>
#include <Arduino.h>
#include "ESCCMD.h"
#include "Definitions_SxS.h"

/* ONLY enable the line(s) if you are debugging your code, no need otherwise */
//#define INCLUDE_DEBUGCODE 0
//#define VERBOSE_MESSAGE

/* ONLY uncomment one of the next lines to test the motors running and servos moving without a flightcontroller */
//#define TEST_MOTORS 
//#define TEST_SERVOS

#ifdef INCLUDE_DEBUGCODE
#define DEBUG_serial Serial
#define DEBUG_serial_baud 115200
#endif

/* --------------------------- COMMON DEFINED VARIABLES -------------------------------- */
#define SMALLISHNUMBER (0.1f)
#define REASONABLYSMALLNUMBER (1e-3)
#define VERYSMALLNUMBER (1e-5)
#define EXTREMLYSMALLNUMBER (1e-6)

/* ------------------------ COMMUNICATION DEFINED VARIABLES ---------------------------- */
#define COMMUNICATION_SERIAL Serial4
#define COMMUNICATION_SERIAL_BAUD 921600 // BAUDrate for seraildatastream towards and from the Flightcontroller, 1500000 is maybe possible but untested
#define COMM_REFRESH_TIME_DEFAULT 1600   // ~500 Hz message loop
#define PIN_LED_T4 13
#define COMMON_TMR 5000
#define SERVO_STATE_MEM_BUF_SIZE 100 // Must be at least be larger than 99, 100 is already the sweetspot

/* --------------------------- ESC DEFINED VARIABLES ----------------------------------- */
#define ESCPID_NB_ESC 4   // Number of ESCs
#define ESCPID_MAX_ESC 4  // Max number of ESCs

#define MIN_DSHOT_CMD 100
#define MAX_DSHOT_CMD 1999

/* -------------------------- SERVOS DEFINED VARIABLES --------------------------------- */

/* PWM servos settings and declarations used for PWM based servo 11 and 12 */
#define SERVO11_PWM_PIN 2
#define SERVO12_PWM_PIN 3
#define PWM_SERVO_FREQUENCY 300  // [Hz] DO NOT SET TO A HIGHER VALUE THAN 400 Hz !!!

/* SERIAL BUS servos default settings 
   NOTE: The Rx and Tx pins at the Teensy must be tied together and connected via ~150 Ohm to the FEETECH data line! */
#define BAUDRATE_SERVO 1000000  // Baudrate for the servo communication
#define SERVO_MAX_COMD_DEFAULT 4095     // It tells us the step we use to control the servo angle for STS type of servos
#define SERVO_MAX_COMD_DEFAULT_SCS 1023 // It tells us the step we use to control the servo angle for SCS type of servos
#define DEFAULT_STEPS_FOR_FULL_ROTATION 4095
#define DEFAULT_STEPS_FOR_FULL_ROTATION_SCS 1023
#define DEFAULT_MULTIPLIER_DEGREES 100
#define SERVO_COMM_MARGIN_DEFAULT 80    // [uS] Margin of time given to the servo for the response [20] opti = 80 [100]
#define TIME_OF_SERVO_TX_DEFAULT 20     // [uS] Margin of time needed to avoid TX conflicts when the servos are killed [10] opti = 20 [50]

/* ---------------------------- LET'S GET DOWN TO IT ----------------------------------- */
byte START_BYTE_SERIAL_ACT_T4 = 0x9A;
elapsedMicros last_time_write_to_flightcontroller = 0;
int ack_comm_TX_flightcontroller = 0;

struct serial_act_t4_in myserial_act_t4_in;
volatile struct serial_act_t4_out myserial_act_t4_out;
volatile struct serial_act_t4_internal myserial_act_t4_internal;
volatile float extra_data_in[255], extra_data_out[255] __attribute__((aligned));

uint8_t rolling_message_out_id_cnt = 0;

uint16_t serial_act_t4_buf_in_cnt = 0;

uint8_t serial_act_t4_msg_buf_in[2 * sizeof(struct serial_act_t4_in)] = { 0 };
int serial_act_t4_received_packets = 0;
elapsedMillis old_time_frequency_in = 0, time_no_connection_flightcontroller = 0;
uint16_t serial_act_t4_message_frequency_in;
int serial_act_t4_missed_packets_in;

float comm_refresh_time, comm_refresh_frequency;

int Servo_1_arm, Servo_2_arm, Servo_3_arm, Servo_4_arm, Servo_5_arm, Servo_6_arm, Servo_7_arm, Servo_8_arm, Servo_9_arm, Servo_10_arm;
int Servo_11_arm, Servo_12_arm;// PWM servos

int ESC_1_arm, ESC_2_arm, ESC_3_arm, ESC_4_arm;

float PWM_to_pulse_multiplier;
float servo_11_state_memory[SERVO_STATE_MEM_BUF_SIZE], servo_12_state_memory[SERVO_STATE_MEM_BUF_SIZE];
float Servo11_state, Servo11_state_old, Servo12_state, Servo12_state_old;

float SERVO_11_FIRST_ORD_DYN_DEN, SERVO_11_FIRST_ORD_DYN_NUM, SERVO_11_MAX_PWM, SERVO_11_MIN_PWM, SERVO_11_ZERO_PWM;
float SERVO_11_MIN_ANGLE_DEG, SERVO_11_MAX_ANGLE_DEG;
int SERVO_11_DELAY_TS;

float SERVO_12_FIRST_ORD_DYN_DEN, SERVO_12_FIRST_ORD_DYN_NUM, SERVO_12_MAX_PWM, SERVO_12_MIN_PWM, SERVO_12_ZERO_PWM;
float SERVO_12_MIN_ANGLE_DEG, SERVO_12_MAX_ANGLE_DEG;
int SERVO_12_DELAY_TS;

int servo_1_is_SCS, servo_2_is_SCS, servo_3_is_SCS, servo_4_is_SCS, servo_5_is_SCS, servo_6_is_SCS, servo_7_is_SCS, servo_8_is_SCS, servo_9_is_SCS, servo_10_is_SCS;

elapsedMicros last_time_write_read_servo_cnt = 0;
int ack_write_read = 0;

float servo_1_max_command = SERVO_MAX_COMD_DEFAULT; 
float servo_2_max_command = SERVO_MAX_COMD_DEFAULT;
float servo_3_max_command = SERVO_MAX_COMD_DEFAULT; 
float servo_4_max_command = SERVO_MAX_COMD_DEFAULT;
float servo_5_max_command = SERVO_MAX_COMD_DEFAULT;
float servo_6_max_command = SERVO_MAX_COMD_DEFAULT;
float servo_7_max_command = SERVO_MAX_COMD_DEFAULT; 
float servo_8_max_command = SERVO_MAX_COMD_DEFAULT;
float servo_9_max_command = SERVO_MAX_COMD_DEFAULT;
float servo_10_max_command = SERVO_MAX_COMD_DEFAULT;

float steps_for_full_rotation_servo_1 = DEFAULT_STEPS_FOR_FULL_ROTATION; 
float steps_for_full_rotation_servo_2 = DEFAULT_STEPS_FOR_FULL_ROTATION;
float steps_for_full_rotation_servo_3 = DEFAULT_STEPS_FOR_FULL_ROTATION; 
float steps_for_full_rotation_servo_4 = DEFAULT_STEPS_FOR_FULL_ROTATION;
float steps_for_full_rotation_servo_5 = DEFAULT_STEPS_FOR_FULL_ROTATION; 
float steps_for_full_rotation_servo_6 = DEFAULT_STEPS_FOR_FULL_ROTATION;
float steps_for_full_rotation_servo_7 = DEFAULT_STEPS_FOR_FULL_ROTATION;
float steps_for_full_rotation_servo_8 = DEFAULT_STEPS_FOR_FULL_ROTATION;
float steps_for_full_rotation_servo_9 = DEFAULT_STEPS_FOR_FULL_ROTATION; 
float steps_for_full_rotation_servo_10 = DEFAULT_STEPS_FOR_FULL_ROTATION;

float degrees_accuracy_multiplier = DEFAULT_MULTIPLIER_DEGREES;
float steps_for_full_rotation = DEFAULT_STEPS_FOR_FULL_ROTATION;
float TIME_OF_SERVO_TX = TIME_OF_SERVO_TX; 
float SERVO_COMM_MARGIN = SERVO_COMM_MARGIN_DEFAULT;

//Servo 1:
#define SERVO1_serial Serial5
#define SERVO1_serialEnableOpenDrain Serial5EnableOpenDrain
//Servo 2:
#define SERVO2_serial Serial5
#define SERVO2_serialEnableOpenDrain Serial5EnableOpenDrain
//Servo 3:
#define SERVO3_serial Serial5
#define SERVO3_serialEnableOpenDrain Serial5EnableOpenDrain
//Servo 4:
#define SERVO4_serial Serial5
#define SERVO4_serialEnableOpenDrain Serial5EnableOpenDrain
//Servo 5:
#define SERVO5_serial Serial5
#define SERVO5_serialEnableOpenDrain Serial5EnableOpenDrain

/* ---------------- */

//Servo 6:
#define SERVO6_serial Serial7
#define SERVO6_serialEnableOpenDrain Serial7EnableOpenDrain
//Servo 7:
#define SERVO7_serial Serial7
#define SERVO7_serialEnableOpenDrain Serial7EnableOpenDrain
//Servo 8:
#define SERVO8_serial Serial7
#define SERVO8_serialEnableOpenDrain Serial7EnableOpenDrain
//Servo 9:
#define SERVO9_serial Serial7
#define SERVO9_serialEnableOpenDrain Serial7EnableOpenDrain
//Servo 10:
#define SERVO10_serial Serial7
#define SERVO10_serialEnableOpenDrain Serial7EnableOpenDrain

/* PREPARE HARDWARE TO BE IN TRISTATE MODE */
IMXRT_LPUART_t* s_pkuart_1 = &IMXRT_LPUART6;  // underlying hardware for Serial1
IMXRT_LPUART_t* s_pkuart_2 = &IMXRT_LPUART4;  // underlying hardware for Serial2
IMXRT_LPUART_t* s_pkuart_3 = &IMXRT_LPUART2;  // underlying hardware for Serial3
IMXRT_LPUART_t* s_pkuart_4 = &IMXRT_LPUART3;  // underlying hardware for Serial4
IMXRT_LPUART_t* s_pkuart_5 = &IMXRT_LPUART8;  // underlying hardware for Serial5
IMXRT_LPUART_t* s_pkuart_6 = &IMXRT_LPUART1;  // underlying hardware for Serial6
IMXRT_LPUART_t* s_pkuart_7 = &IMXRT_LPUART7;  // underlying hardware for Serial7

/* Define time interrupts for the SERVOs: */
IntervalTimer SERVO_COMM_WRITE_READ_TIMER;

int iter_counter_SERVO = 0;

int Servo_1_ID = 1;  
int Servo_2_ID = 2;  

int Servo_3_ID = 3;  
int Servo_4_ID = 4;  

int Servo_5_ID = 5;  
int Servo_6_ID = 6;  

int Servo_7_ID = 7;  
int Servo_8_ID = 8; 

int Servo_9_ID = 9; 
int Servo_10_ID = 10; 

volatile int Target_position_servo_1, Target_position_servo_2, Target_position_servo_3, Target_position_servo_4, Target_position_servo_5;
volatile int Target_position_servo_6, Target_position_servo_7, Target_position_servo_8, Target_position_servo_9, Target_position_servo_10;

volatile int position_ack_servo_1_lost, position_ack_servo_2_lost, position_ack_servo_3_lost, position_ack_servo_4_lost, position_ack_servo_5_lost;
volatile int position_ack_servo_6_lost, position_ack_servo_7_lost, position_ack_servo_8_lost, position_ack_servo_9_lost, position_ack_servo_10_lost;

volatile int feedback_servo_1_lost, feedback_servo_2_lost, feedback_servo_3_lost, feedback_servo_4_lost, feedback_servo_5_lost;
volatile int feedback_servo_6_lost, feedback_servo_7_lost, feedback_servo_8_lost, feedback_servo_9_lost, feedback_servo_10_lost;

elapsedMicros feedback_1_time_uS_counter = 0, feedback_2_time_uS_counter = 0, feedback_3_time_uS_counter = 0, feedback_4_time_uS_counter = 0, feedback_5_time_uS_counter = 0;
elapsedMicros feedback_6_time_uS_counter = 0, feedback_7_time_uS_counter = 0, feedback_8_time_uS_counter = 0, feedback_9_time_uS_counter = 0, feedback_10_time_uS_counter = 0;

elapsedMicros timer_count_servo = 0, timer_count_main = 0, timer_count_esc = 0;

/************************ SETUP ************************/
void setup(void) {
  /* Setup Debugging port */
  #ifdef INCLUDE_DEBUGCODE
  DEBUG_serial.begin(DEBUG_serial_baud);
  #endif

  /* Setup Servos's */
  InitServos();

  /* Setup ESC's */
  ESCCMD_init(ESCPID_NB_ESC);  // Initialize the CMD subsystem

  /* Arming ESCs */
  ESCCMD_arm_all();

  /* Start the periodic loop */
  ESCCMD_start_timer();

  /* Setup connection with Flightcontroller */
  COMMUNICATION_SERIAL.begin(COMMUNICATION_SERIAL_BAUD);

  /* LED to monitor the connection with Flightcontroller */
  pinMode(PIN_LED_T4, OUTPUT);
  analogWriteFrequency(PIN_LED_T4, 500);

  /* Start the timer for the Communication and Servos */
  SERVO_COMM_WRITE_READ_TIMER.begin(ServosAndCommTic, 10);  // Interrupt routine for the tick to servos and communication

  while (millis() < COMMON_TMR) ESCCMD_tic(); // Give time to the ESC to initialize properly
}

/************************ LOOP ************************/
void loop(void) {

  writeReadServos();

  SendReceiveFlightcontroller();

  ServoRoutine();

  EscRoutine();

  GenerateCommands(); 
  
  /* ONLY enable those functions for debugging purposes! One can enable multiple functions at the same time. 
     The #ifdef INCLUDE_DEBUGCODE must be enabled at the top of the code.
     Note that it can have an actuators response performance impact, so use wisely. */
  if (timer_count_main > COMMON_TMR) {  // 200 Hz loop
    #ifdef INCLUDE_DEBUGCODE                       
    //  DisplayEscVoltage();
    //  DisplayEscCurrent();
    //  DisplayEscRpm();
    //  DisplayEscErr();
    //  DisplayEscCmd();
    //  DebugServoLostFeedbackPackets();
    //  DebugUpdateTimePosPackets();
    //  DebugServoLostAckPackets();
    //  DebugServoPosition();
    //  DebugConnection();
    //  DebugReceivedCmdESCs();
    #endif
    timer_count_main = 0;
  }

}

/*****************************************************************************/
/*************************** USER DEFINED FCN ********************************/
/*****************************************************************************/

#ifdef INCLUDE_DEBUGCODE
void DebugReceivedCmdESCs(void) {
  DEBUG_serial.print("ESC_1_cmd_dshot:");
  DEBUG_serial.print(myserial_act_t4_in.esc_1_dshot_cmd_int);
  DEBUG_serial.print("ESC_2_cmd_dshot:");
  DEBUG_serial.print(myserial_act_t4_in.esc_2_dshot_cmd_int);
  DEBUG_serial.print("ESC_3_cmd_dshot:");
  DEBUG_serial.print(myserial_act_t4_in.esc_3_dshot_cmd_int);
  DEBUG_serial.print("ESC_4_cmd_dshot:");
  DEBUG_serial.println(myserial_act_t4_in.esc_4_dshot_cmd_int);
}
#endif

void GenerateCommands(void){

  #ifdef TEST_MOTORS 
  #warning "TEST_MOTORS is defined, Motors via ESCs will be run tested, do not use in stable product releases!"
  myserial_act_t4_in.esc_arm_int = 0;
  bitSet(myserial_act_t4_in.esc_arm_int, 0); //ESC 1 - comment to kill 
  bitSet(myserial_act_t4_in.esc_arm_int, 1); //ESC 2 - comment to kill
  bitSet(myserial_act_t4_in.esc_arm_int, 2); //ESC 3 - comment to kill
  bitSet(myserial_act_t4_in.esc_arm_int, 3); //ESC 4 - comment to kill
  time_no_connection_flightcontroller = 0;
  myserial_act_t4_in.esc_1_dshot_cmd_int = 600;
  myserial_act_t4_in.esc_2_dshot_cmd_int = 700;
  myserial_act_t4_in.esc_3_dshot_cmd_int = 800;
  myserial_act_t4_in.esc_4_dshot_cmd_int = 900;
  #endif

  #ifdef TEST_SERVOS
  #warning "TEST_SERVOS is defined, servos will be tested, do not use in stable product releases!"
  myserial_act_t4_in.servo_arm_int = 0;
  bitSet(myserial_act_t4_in.servo_arm_int, 0); //Servo 1 - comment to kill
  bitSet(myserial_act_t4_in.servo_arm_int, 1); //Servo 2 - comment to kill
  bitSet(myserial_act_t4_in.servo_arm_int, 2); //Servo 3 - comment to kill 
  bitSet(myserial_act_t4_in.servo_arm_int, 3); //Servo 4 - comment to kill 
  bitSet(myserial_act_t4_in.servo_arm_int, 4); //Servo 5 - comment to kill 
  bitSet(myserial_act_t4_in.servo_arm_int, 5); //Servo 6 - comment to kill 
  bitSet(myserial_act_t4_in.servo_arm_int, 6); //Servo 7 - comment to kill 
  bitSet(myserial_act_t4_in.servo_arm_int, 7); //Servo 8 - comment to kill
  bitSet(myserial_act_t4_in.servo_arm_int, 8); //Servo 9 - comment to kill 
  bitSet(myserial_act_t4_in.servo_arm_int, 9); //Servo 10 - comment to kill 
  time_no_connection_flightcontroller = 0;
  myserial_act_t4_in.servo_1_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  myserial_act_t4_in.servo_2_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  myserial_act_t4_in.servo_3_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  myserial_act_t4_in.servo_4_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  myserial_act_t4_in.servo_5_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  myserial_act_t4_in.servo_6_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  myserial_act_t4_in.servo_7_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  myserial_act_t4_in.servo_8_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  myserial_act_t4_in.servo_9_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  myserial_act_t4_in.servo_10_cmd_int = 60*degrees_accuracy_multiplier*sin(2*PI*0.1*millis()/1000);
  // servo_1_is_SCS = 1;  // Enabled only for testing FeeTech SCS types of servos instead of STS types
  #endif

  if(comm_refresh_frequency < .1) comm_refresh_frequency = 1/(COMM_REFRESH_TIME_DEFAULT*1e-6); // Apply default values if no meaningful values were received:

  comm_refresh_time = 1/(comm_refresh_frequency*1e-6);

  /* Correct in case we have sent a extra_data_in made up of zeros: */
  if (fabs(SERVO_11_FIRST_ORD_DYN_DEN < 1e-5)) SERVO_11_FIRST_ORD_DYN_DEN = -0.9802; // Magic number bonus ;-)
  if (fabs(SERVO_12_FIRST_ORD_DYN_NUM < 1e-5)) SERVO_12_FIRST_ORD_DYN_NUM = 0.0198;
  if (SERVO_11_MAX_PWM < .1) SERVO_11_MAX_PWM = 1900;   // Yeah, if it does not suit your scenario change at will...
  if (SERVO_11_MIN_PWM < .1) SERVO_11_MIN_PWM = 1100;   //Ditto
  if (SERVO_11_ZERO_PWM < .1) SERVO_11_ZERO_PWM = 1500; //...indeed.
  if (fabs(SERVO_11_MIN_ANGLE_DEG) < .1) SERVO_11_MIN_ANGLE_DEG = -60;
  if (fabs(SERVO_11_MAX_ANGLE_DEG) < .1) SERVO_11_MAX_ANGLE_DEG = 60;
  if((float)SERVO_11_DELAY_TS < .1) SERVO_11_DELAY_TS = 5;

  if (fabs(SERVO_12_FIRST_ORD_DYN_DEN < 1e-5)) SERVO_12_FIRST_ORD_DYN_DEN = -0.9802;
  if (fabs(SERVO_12_FIRST_ORD_DYN_NUM < 1e-5)) SERVO_12_FIRST_ORD_DYN_NUM = 0.0198;
  if (SERVO_12_MAX_PWM < .1) SERVO_12_MAX_PWM = 1900; // Yeah, if it does not suit your scenario change at will...
  if (SERVO_12_MIN_PWM < .1) SERVO_12_MIN_PWM = 1100; // Ditto
  if (SERVO_12_ZERO_PWM < .1) SERVO_12_ZERO_PWM = 1500; //...indeed.
  if (fabs(SERVO_12_MIN_ANGLE_DEG) < .1) SERVO_12_MIN_ANGLE_DEG = -60;
  if (fabs(SERVO_12_MAX_ANGLE_DEG) < .1) SERVO_12_MAX_ANGLE_DEG = 60;
  if((float)SERVO_12_DELAY_TS < .1) SERVO_12_DELAY_TS = 5;

  if (servo_1_max_command < 10.) servo_1_max_command = SERVO_MAX_COMD_DEFAULT;
  if (servo_2_max_command < 10.) servo_2_max_command = SERVO_MAX_COMD_DEFAULT;
  if (servo_3_max_command < 10.) servo_3_max_command = SERVO_MAX_COMD_DEFAULT;
  if (servo_4_max_command < 10.) servo_4_max_command = SERVO_MAX_COMD_DEFAULT;
  if (servo_5_max_command < 10.) servo_5_max_command = SERVO_MAX_COMD_DEFAULT;
  if (servo_6_max_command < 10.) servo_6_max_command = SERVO_MAX_COMD_DEFAULT;
  if (servo_7_max_command < 10.) servo_7_max_command = SERVO_MAX_COMD_DEFAULT;
  if (servo_8_max_command < 10.) servo_8_max_command = SERVO_MAX_COMD_DEFAULT;
  if (servo_9_max_command < 10.) servo_9_max_command = SERVO_MAX_COMD_DEFAULT;
  if (servo_10_max_command < 10.) servo_10_max_command = SERVO_MAX_COMD_DEFAULT;

  if(fabs(steps_for_full_rotation) < 1e-3 ){
    steps_for_full_rotation_servo_1 = DEFAULT_STEPS_FOR_FULL_ROTATION;
    steps_for_full_rotation_servo_2 = DEFAULT_STEPS_FOR_FULL_ROTATION;
    steps_for_full_rotation_servo_3 = DEFAULT_STEPS_FOR_FULL_ROTATION;
    steps_for_full_rotation_servo_4 = DEFAULT_STEPS_FOR_FULL_ROTATION;
    steps_for_full_rotation_servo_5 = DEFAULT_STEPS_FOR_FULL_ROTATION;
    steps_for_full_rotation_servo_6 = DEFAULT_STEPS_FOR_FULL_ROTATION;
    steps_for_full_rotation_servo_7 = DEFAULT_STEPS_FOR_FULL_ROTATION;
    steps_for_full_rotation_servo_8 = DEFAULT_STEPS_FOR_FULL_ROTATION;
    steps_for_full_rotation_servo_9 = DEFAULT_STEPS_FOR_FULL_ROTATION;
    steps_for_full_rotation_servo_10 = DEFAULT_STEPS_FOR_FULL_ROTATION;
  }
  else {
    steps_for_full_rotation_servo_1 = steps_for_full_rotation;
    steps_for_full_rotation_servo_2 = steps_for_full_rotation;
    steps_for_full_rotation_servo_3 = steps_for_full_rotation;
    steps_for_full_rotation_servo_4 = steps_for_full_rotation;
    steps_for_full_rotation_servo_5 = steps_for_full_rotation;
    steps_for_full_rotation_servo_6 = steps_for_full_rotation;
    steps_for_full_rotation_servo_7 = steps_for_full_rotation;
    steps_for_full_rotation_servo_8 = steps_for_full_rotation;
    steps_for_full_rotation_servo_9 = steps_for_full_rotation;
    steps_for_full_rotation_servo_10 = steps_for_full_rotation;
  }

  if(fabs(degrees_accuracy_multiplier) < 1e-3 ){
    degrees_accuracy_multiplier = DEFAULT_MULTIPLIER_DEGREES;
  }
  
  if(servo_1_is_SCS || servo_2_is_SCS || servo_3_is_SCS || servo_4_is_SCS || servo_5_is_SCS || servo_6_is_SCS || servo_7_is_SCS || servo_8_is_SCS || servo_9_is_SCS || servo_10_is_SCS){
    SERVO_COMM_MARGIN = 220; // Only change this value if you know what you are doing
    TIME_OF_SERVO_TX = 60;   // Only change this value if you know what you are doing
  }
  else {
    SERVO_COMM_MARGIN = SERVO_COMM_MARGIN_DEFAULT;
    TIME_OF_SERVO_TX = TIME_OF_SERVO_TX_DEFAULT;
  }

  if(servo_1_is_SCS){
    steps_for_full_rotation_servo_1 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_1_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  if(servo_2_is_SCS){
    steps_for_full_rotation_servo_2 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_2_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  if(servo_3_is_SCS){
    steps_for_full_rotation_servo_3 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_3_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  if(servo_4_is_SCS){
    steps_for_full_rotation_servo_4 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_4_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  if(servo_5_is_SCS){
    steps_for_full_rotation_servo_5 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_5_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  if(servo_6_is_SCS){
    steps_for_full_rotation_servo_6 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_6_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  if(servo_7_is_SCS){
    steps_for_full_rotation_servo_7 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_7_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  if(servo_8_is_SCS){
    steps_for_full_rotation_servo_8 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_8_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  if(servo_9_is_SCS){
    steps_for_full_rotation_servo_9 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_9_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  if(servo_10_is_SCS){
    steps_for_full_rotation_servo_10 = DEFAULT_STEPS_FOR_FULL_ROTATION_SCS; 
    servo_10_max_command = SERVO_MAX_COMD_DEFAULT_SCS;
  }

  // Extract Servo and ESC arm status from the bitmask in myserial_act_t4_in
  if (time_no_connection_flightcontroller < COMMON_TMR) {
    Servo_1_arm = (myserial_act_t4_in.servo_arm_int & 0x01) >> 0;
    Servo_2_arm = (myserial_act_t4_in.servo_arm_int & 0x02) >> 1;
    Servo_3_arm = (myserial_act_t4_in.servo_arm_int & 0x04) >> 2;
    Servo_4_arm = (myserial_act_t4_in.servo_arm_int & 0x08) >> 3;
    Servo_5_arm = (myserial_act_t4_in.servo_arm_int & 0x10) >> 4;
    Servo_6_arm = (myserial_act_t4_in.servo_arm_int & 0x20) >> 5;
    Servo_7_arm = (myserial_act_t4_in.servo_arm_int & 0x40) >> 6;
    Servo_8_arm = (myserial_act_t4_in.servo_arm_int & 0x80) >> 7;
    Servo_9_arm = (myserial_act_t4_in.servo_arm_int & 0x100) >> 8;
    Servo_10_arm = (myserial_act_t4_in.servo_arm_int & 0x200) >> 9;
    Servo_11_arm = (myserial_act_t4_in.servo_arm_int & 0x400) >> 10;
    Servo_12_arm = (myserial_act_t4_in.servo_arm_int & 0x800) >> 11;

    ESC_1_arm = (myserial_act_t4_in.esc_arm_int & 0x01) >> 0;
    ESC_2_arm = (myserial_act_t4_in.esc_arm_int & 0x02) >> 1;
    ESC_3_arm = (myserial_act_t4_in.esc_arm_int & 0x04) >> 2;
    ESC_4_arm = (myserial_act_t4_in.esc_arm_int & 0x08) >> 3;
  } 
  else {
    Servo_1_arm = 0; 
    Servo_2_arm = 0;
    Servo_3_arm = 0;
    Servo_4_arm = 0;
    Servo_5_arm = 0;
    Servo_6_arm = 0;
    Servo_7_arm = 0;
    Servo_8_arm = 0;
    Servo_9_arm = 0;
    Servo_10_arm = 0;

    Servo_11_arm = 0;
    Servo_12_arm = 0;

    ESC_1_arm = 0;
    ESC_2_arm = 0;
    ESC_3_arm = 0;
    ESC_4_arm = 0; 
  }

  #define FULLROTATION 360
  /* Apply received message to actuators: [servo arming status is managed in the loop automatically, no need for if statement] */
  Target_position_servo_1 = (int)(constrain(steps_for_full_rotation_servo_1/FULLROTATION * myserial_act_t4_in.servo_1_cmd_int / (degrees_accuracy_multiplier) + (servo_1_max_command / 2.0), 0, servo_1_max_command));
  Target_position_servo_2 = (int)(constrain(steps_for_full_rotation_servo_2/FULLROTATION * myserial_act_t4_in.servo_2_cmd_int / (degrees_accuracy_multiplier) + (servo_2_max_command / 2.0), 0, servo_2_max_command));
  Target_position_servo_3 = (int)(constrain(steps_for_full_rotation_servo_3/FULLROTATION * myserial_act_t4_in.servo_3_cmd_int / (degrees_accuracy_multiplier) + (servo_3_max_command / 2.0), 0, servo_3_max_command));
  Target_position_servo_4 = (int)(constrain(steps_for_full_rotation_servo_4/FULLROTATION * myserial_act_t4_in.servo_4_cmd_int / (degrees_accuracy_multiplier) + (servo_4_max_command / 2.0), 0, servo_4_max_command));
  Target_position_servo_5 = (int)(constrain(steps_for_full_rotation_servo_5/FULLROTATION * myserial_act_t4_in.servo_5_cmd_int / (degrees_accuracy_multiplier) + (servo_5_max_command / 2.0), 0, servo_5_max_command));
  Target_position_servo_6 = (int)(constrain(steps_for_full_rotation_servo_6/FULLROTATION * myserial_act_t4_in.servo_6_cmd_int / (degrees_accuracy_multiplier) + (servo_6_max_command / 2.0), 0, servo_6_max_command));
  Target_position_servo_7 = (int)(constrain(steps_for_full_rotation_servo_7/FULLROTATION * myserial_act_t4_in.servo_7_cmd_int / (degrees_accuracy_multiplier) + (servo_7_max_command / 2.0), 0, servo_7_max_command));
  Target_position_servo_8 = (int)(constrain(steps_for_full_rotation_servo_8/FULLROTATION * myserial_act_t4_in.servo_8_cmd_int / (degrees_accuracy_multiplier) + (servo_8_max_command / 2.0), 0, servo_8_max_command));
  Target_position_servo_9 = (int)(constrain(steps_for_full_rotation_servo_9/FULLROTATION * myserial_act_t4_in.servo_9_cmd_int / (degrees_accuracy_multiplier) + (servo_9_max_command / 2.0), 0, servo_9_max_command));
  Target_position_servo_10 = (int)(constrain(steps_for_full_rotation_servo_10/FULLROTATION * myserial_act_t4_in.servo_10_cmd_int / (degrees_accuracy_multiplier) + (servo_10_max_command / 2.0), 0, servo_10_max_command));

  /* ONLY Enable next line below for debugging purposes */
  //DEBUG_serial.println(Target_position_servo_1);

  if(ESC_1_arm){
    ESCCMD_throttle(0, constrain(myserial_act_t4_in.esc_1_dshot_cmd_int, MIN_DSHOT_CMD, MAX_DSHOT_CMD));
  } 
  else {
    ESCCMD_stop(0);
  }

  if(ESC_2_arm){
    ESCCMD_throttle(1, constrain(myserial_act_t4_in.esc_2_dshot_cmd_int, MIN_DSHOT_CMD, MAX_DSHOT_CMD));
  } 
  else {
    ESCCMD_stop(1);
  }

  if(ESC_3_arm){
    ESCCMD_throttle(2, constrain(myserial_act_t4_in.esc_3_dshot_cmd_int, MIN_DSHOT_CMD, MAX_DSHOT_CMD));
  } 
  else {
    ESCCMD_stop(2);
  }

  if(ESC_4_arm){
    ESCCMD_throttle(3, constrain(myserial_act_t4_in.esc_4_dshot_cmd_int, MIN_DSHOT_CMD, MAX_DSHOT_CMD));
  }
  else {
    ESCCMD_stop(3);
  }
  
}

/* Parse the incoming message from Flightcontroller and assign them to local variables */
void serial_act_t4_parse_msg_in(void) {

  /* Copy received buffer to structure */
  memmove(&myserial_act_t4_in, &serial_act_t4_msg_buf_in[1], sizeof(struct serial_act_t4_in) - 1);
  extra_data_in[myserial_act_t4_in.rolling_msg_in_id] = myserial_act_t4_in.rolling_msg_in;

  comm_refresh_frequency = extra_data_in[0];
  
  /* Assign rolling message to variables: */
  SERVO_11_FIRST_ORD_DYN_DEN = extra_data_in[1];
  SERVO_11_FIRST_ORD_DYN_NUM = extra_data_in[2];
  SERVO_11_MAX_PWM = extra_data_in[3];
  SERVO_11_MIN_PWM = extra_data_in[4];
  SERVO_11_ZERO_PWM = extra_data_in[5];
  SERVO_11_MIN_ANGLE_DEG = extra_data_in[6];
  SERVO_11_MAX_ANGLE_DEG = extra_data_in[7];
  SERVO_11_DELAY_TS = (int)extra_data_in[8];
  
  SERVO_12_FIRST_ORD_DYN_DEN = extra_data_in[9];
  SERVO_12_FIRST_ORD_DYN_NUM = extra_data_in[10];
  SERVO_12_MAX_PWM = extra_data_in[11];
  SERVO_12_MIN_PWM = extra_data_in[12];
  SERVO_12_ZERO_PWM = extra_data_in[13];
  SERVO_12_MIN_ANGLE_DEG = extra_data_in[14];
  SERVO_12_MAX_ANGLE_DEG = extra_data_in[15];
  SERVO_12_DELAY_TS = (int)extra_data_in[16];
  
  servo_1_max_command = extra_data_in[17];
  servo_2_max_command = extra_data_in[18];
  servo_3_max_command = extra_data_in[19];
  servo_4_max_command = extra_data_in[20];
  servo_5_max_command = extra_data_in[21];
  servo_6_max_command = extra_data_in[22];
  servo_7_max_command = extra_data_in[23];
  servo_8_max_command = extra_data_in[24];
  servo_9_max_command = extra_data_in[25];
  servo_10_max_command = extra_data_in[26];

  degrees_accuracy_multiplier = extra_data_in[27];

  steps_for_full_rotation = extra_data_in[28];

  servo_1_is_SCS = (int) extra_data_in[29];
  servo_2_is_SCS = (int) extra_data_in[30];
  servo_3_is_SCS = (int) extra_data_in[31];
  servo_4_is_SCS = (int) extra_data_in[32];
  servo_5_is_SCS = (int) extra_data_in[33];
  servo_6_is_SCS = (int) extra_data_in[34];
  servo_7_is_SCS = (int) extra_data_in[35];
  servo_8_is_SCS = (int) extra_data_in[36];
  servo_9_is_SCS = (int) extra_data_in[37];
  servo_10_is_SCS = (int) extra_data_in[38];

}

/* Collect the telemetry mesage from the ESCs */
void CollectEscTelem(void) {

  /* ESC RPM */
  int16_t local_rpm1 = 0, local_rpm2 = 0, local_rpm3 = 0, local_rpm4 = 0;
  ESCCMD_read_rpm(0, &local_rpm1);
  ESCCMD_read_rpm(1, &local_rpm2);
  ESCCMD_read_rpm(2, &local_rpm3);
  ESCCMD_read_rpm(3, &local_rpm4);
  myserial_act_t4_internal.esc_1_rpm_int = (int16_t)(abs(local_rpm1 * 10));
  myserial_act_t4_internal.esc_2_rpm_int = (int16_t)(abs(local_rpm2 * 10));
  myserial_act_t4_internal.esc_3_rpm_int = (int16_t)(abs(local_rpm3 * 10));
  myserial_act_t4_internal.esc_4_rpm_int = (int16_t)(abs(local_rpm4 * 10));

  /* ESC ERROR CODE */
  int8_t local_err1 = 0, local_err2 = 0, local_err3 = 0, local_err4 = 0;
  ESCCMD_read_err(0, &local_err1);
  ESCCMD_read_err(1, &local_err2);
  ESCCMD_read_err(2, &local_err3);
  ESCCMD_read_err(3, &local_err4);
  myserial_act_t4_internal.esc_1_error_code_int = (int16_t)(local_err1);
  myserial_act_t4_internal.esc_2_error_code_int = (int16_t)(local_err2);
  myserial_act_t4_internal.esc_3_error_code_int = (int16_t)(local_err3);
  myserial_act_t4_internal.esc_4_error_code_int = (int16_t)(local_err4);

  /* ESC VOLTAGE */
  uint16_t local_volt1 = 0, local_volt2 = 0, local_volt3 = 0, local_volt4 = 0;
  ESCCMD_read_volt(0, &local_volt1);
  ESCCMD_read_volt(1, &local_volt2);
  ESCCMD_read_volt(2, &local_volt3);
  ESCCMD_read_volt(3, &local_volt4);
  myserial_act_t4_internal.esc_1_voltage_int = (int16_t)(local_volt1);
  myserial_act_t4_internal.esc_2_voltage_int = (int16_t)(local_volt2);
  myserial_act_t4_internal.esc_3_voltage_int = (int16_t)(local_volt3);
  myserial_act_t4_internal.esc_4_voltage_int = (int16_t)(local_volt4);

  /* ESC CURRENT */
  uint16_t local_current1 = 0, local_current2 = 0, local_current3 = 0, local_current4 = 0;
  ESCCMD_read_amp(0, &local_current1);
  ESCCMD_read_amp(1, &local_current2);
  ESCCMD_read_amp(2, &local_current3);
  ESCCMD_read_amp(3, &local_current4);
  myserial_act_t4_internal.esc_1_current_int = (int16_t)(local_current1);
  myserial_act_t4_internal.esc_2_current_int = (int16_t)(local_current2);
  myserial_act_t4_internal.esc_3_current_int = (int16_t)(local_current3);
  myserial_act_t4_internal.esc_4_current_int = (int16_t)(local_current4);

  /* ESC Current CURRENT ;) consumption */
  uint16_t local_consumption1 = 0, local_consumption2 = 0, local_consumption3 = 0, local_consumption4 = 0;
  ESCCMD_read_mah(0, &local_consumption1);
  ESCCMD_read_mah(1, &local_consumption2);
  ESCCMD_read_mah(2, &local_consumption3);
  ESCCMD_read_mah(3, &local_consumption4);
  myserial_act_t4_internal.esc_1_consumed_mAh_int = (int16_t)(local_consumption1);
  myserial_act_t4_internal.esc_2_consumed_mAh_int = (int16_t)(local_consumption2);
  myserial_act_t4_internal.esc_3_consumed_mAh_int = (int16_t)(local_consumption3);
  myserial_act_t4_internal.esc_4_consumed_mAh_int = (int16_t)(local_consumption4);
}

/* Receive and send message from Flightcontroller */
void SendReceiveFlightcontroller(void) {

  /* SENDING PACKET */
  if (ack_comm_TX_flightcontroller == 0) {

    /* Collect the ESC telemetry: */
    CollectEscTelem();

    /* Fill the myserial_act_t4_out structure to be sent to the Flightcontroller: */
    myserial_act_t4_out.esc_1_rpm_int = myserial_act_t4_internal.esc_1_rpm_int;
    myserial_act_t4_out.esc_2_rpm_int = myserial_act_t4_internal.esc_2_rpm_int;
    myserial_act_t4_out.esc_3_rpm_int = myserial_act_t4_internal.esc_3_rpm_int;
    myserial_act_t4_out.esc_4_rpm_int = myserial_act_t4_internal.esc_4_rpm_int;
    myserial_act_t4_out.esc_1_error_code_int = myserial_act_t4_internal.esc_1_error_code_int;
    myserial_act_t4_out.esc_2_error_code_int = myserial_act_t4_internal.esc_2_error_code_int;
    myserial_act_t4_out.esc_3_error_code_int = myserial_act_t4_internal.esc_3_error_code_int;
    myserial_act_t4_out.esc_4_error_code_int = myserial_act_t4_internal.esc_4_error_code_int;
    myserial_act_t4_out.esc_1_current_int = myserial_act_t4_internal.esc_1_current_int;
    myserial_act_t4_out.esc_2_current_int = myserial_act_t4_internal.esc_2_current_int;
    myserial_act_t4_out.esc_3_current_int = myserial_act_t4_internal.esc_3_current_int;
    myserial_act_t4_out.esc_4_current_int = myserial_act_t4_internal.esc_4_current_int;
    myserial_act_t4_out.esc_1_voltage_int = myserial_act_t4_internal.esc_1_voltage_int;
    myserial_act_t4_out.esc_2_voltage_int = myserial_act_t4_internal.esc_2_voltage_int;
    myserial_act_t4_out.esc_3_voltage_int = myserial_act_t4_internal.esc_3_voltage_int;
    myserial_act_t4_out.esc_4_voltage_int = myserial_act_t4_internal.esc_4_voltage_int;
    myserial_act_t4_out.servo_1_angle_int = myserial_act_t4_internal.servo_1_angle_int;
    myserial_act_t4_out.servo_2_angle_int = myserial_act_t4_internal.servo_2_angle_int;
    myserial_act_t4_out.servo_3_angle_int = myserial_act_t4_internal.servo_3_angle_int;
    myserial_act_t4_out.servo_4_angle_int = myserial_act_t4_internal.servo_4_angle_int;
    myserial_act_t4_out.servo_5_angle_int = myserial_act_t4_internal.servo_5_angle_int;
    myserial_act_t4_out.servo_6_angle_int = myserial_act_t4_internal.servo_6_angle_int;
    myserial_act_t4_out.servo_7_angle_int = myserial_act_t4_internal.servo_7_angle_int;
    myserial_act_t4_out.servo_8_angle_int = myserial_act_t4_internal.servo_8_angle_int;
    myserial_act_t4_out.servo_9_angle_int = myserial_act_t4_internal.servo_9_angle_int;
    myserial_act_t4_out.servo_10_angle_int = myserial_act_t4_internal.servo_10_angle_int;
    myserial_act_t4_out.servo_11_angle_int = myserial_act_t4_internal.servo_11_angle_int;
    myserial_act_t4_out.servo_1_load_int = myserial_act_t4_internal.servo_1_load_int;
    myserial_act_t4_out.servo_2_load_int = myserial_act_t4_internal.servo_2_load_int;
    myserial_act_t4_out.servo_3_load_int = myserial_act_t4_internal.servo_3_load_int;
    myserial_act_t4_out.servo_4_load_int = myserial_act_t4_internal.servo_4_load_int;
    myserial_act_t4_out.servo_5_load_int = myserial_act_t4_internal.servo_5_load_int;
    myserial_act_t4_out.servo_6_load_int = myserial_act_t4_internal.servo_6_load_int;
    myserial_act_t4_out.servo_7_load_int = myserial_act_t4_internal.servo_7_load_int;
    myserial_act_t4_out.servo_8_load_int = myserial_act_t4_internal.servo_8_load_int;
    myserial_act_t4_out.servo_9_load_int = myserial_act_t4_internal.servo_9_load_int;
    myserial_act_t4_out.servo_10_load_int = myserial_act_t4_internal.servo_10_load_int;

    #define LOSTMAXIMUM 1000

    /* Generate the bitmask for the health of the servos, based on the feedback update time: */
    if (myserial_act_t4_internal.servo_1_feedback_update_time_us < COMMON_TMR && feedback_servo_1_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 0;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 0); 
    }

    if (myserial_act_t4_internal.servo_2_feedback_update_time_us < COMMON_TMR && feedback_servo_2_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 1;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 1); 
    }

    if (myserial_act_t4_internal.servo_3_feedback_update_time_us < COMMON_TMR && feedback_servo_3_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 2;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 2); 
    }

    if (myserial_act_t4_internal.servo_4_feedback_update_time_us < COMMON_TMR && feedback_servo_4_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 3;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 3); 
    }

    if (myserial_act_t4_internal.servo_5_feedback_update_time_us < COMMON_TMR && feedback_servo_5_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 4;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 4); 
    }

    if (myserial_act_t4_internal.servo_6_feedback_update_time_us < COMMON_TMR && feedback_servo_6_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 5;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 5); 
    }

    if (myserial_act_t4_internal.servo_7_feedback_update_time_us < COMMON_TMR && feedback_servo_7_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 6;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 6); 
    }

    if (myserial_act_t4_internal.servo_8_feedback_update_time_us < COMMON_TMR && feedback_servo_8_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 7;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 7); 
    }

    if (myserial_act_t4_internal.servo_9_feedback_update_time_us < COMMON_TMR && feedback_servo_9_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 8;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 8); 
    }

    if (myserial_act_t4_internal.servo_10_feedback_update_time_us < COMMON_TMR && feedback_servo_10_lost < LOSTMAXIMUM){
      myserial_act_t4_out.bitmask_servo_health |= 1 << 9;
    }
    else { 
      myserial_act_t4_out.bitmask_servo_health &= ~(1 << 9); 
    }
    
    /* Fill the extra data to be sent to the Flightcontroller: */
    extra_data_out[0] = (float) myserial_act_t4_internal.esc_1_consumed_mAh_int; 
    extra_data_out[1] = (float) myserial_act_t4_internal.esc_2_consumed_mAh_int;
    extra_data_out[2] = (float) myserial_act_t4_internal.esc_3_consumed_mAh_int;
    extra_data_out[3] = (float) myserial_act_t4_internal.esc_4_consumed_mAh_int;
    extra_data_out[4] = (float) myserial_act_t4_internal.servo_1_speed_int; 
    extra_data_out[5] = (float) myserial_act_t4_internal.servo_2_speed_int;
    extra_data_out[6] = (float) myserial_act_t4_internal.servo_3_speed_int;
    extra_data_out[7] = (float) myserial_act_t4_internal.servo_4_speed_int;
    extra_data_out[8] = (float) myserial_act_t4_internal.servo_5_speed_int;
    extra_data_out[9] = (float) myserial_act_t4_internal.servo_6_speed_int;
    extra_data_out[10] = (float) myserial_act_t4_internal.servo_7_speed_int;
    extra_data_out[11] = (float) myserial_act_t4_internal.servo_8_speed_int;
    extra_data_out[12] = (float) myserial_act_t4_internal.servo_9_speed_int;
    extra_data_out[13] = (float) myserial_act_t4_internal.servo_10_speed_int;
    extra_data_out[14] = (float) myserial_act_t4_internal.servo_1_volt_int;
    extra_data_out[15] = (float) myserial_act_t4_internal.servo_2_volt_int;
    extra_data_out[16] = (float) myserial_act_t4_internal.servo_3_volt_int;
    extra_data_out[17] = (float) myserial_act_t4_internal.servo_4_volt_int;
    extra_data_out[18] = (float) myserial_act_t4_internal.servo_5_volt_int;
    extra_data_out[19] = (float) myserial_act_t4_internal.servo_6_volt_int;
    extra_data_out[20] = (float) myserial_act_t4_internal.servo_7_volt_int;
    extra_data_out[21] = (float) myserial_act_t4_internal.servo_8_volt_int;
    extra_data_out[22] = (float) myserial_act_t4_internal.servo_9_volt_int;
    extra_data_out[23] = (float) myserial_act_t4_internal.servo_10_volt_int;
    extra_data_out[24] = (float) myserial_act_t4_internal.servo_1_temp_int;
    extra_data_out[25] = (float) myserial_act_t4_internal.servo_2_temp_int;
    extra_data_out[26] = (float) myserial_act_t4_internal.servo_3_temp_int;
    extra_data_out[27] = (float) myserial_act_t4_internal.servo_4_temp_int;
    extra_data_out[28] = (float) myserial_act_t4_internal.servo_5_temp_int;
    extra_data_out[29] = (float) myserial_act_t4_internal.servo_6_temp_int;
    extra_data_out[30] = (float) myserial_act_t4_internal.servo_7_temp_int;
    extra_data_out[31] = (float) myserial_act_t4_internal.servo_8_temp_int;
    extra_data_out[32] = (float) myserial_act_t4_internal.servo_9_temp_int;
    extra_data_out[33] = (float) myserial_act_t4_internal.servo_10_temp_int;
    extra_data_out[34] = (float) myserial_act_t4_internal.servo_1_feedback_update_time_us;
    extra_data_out[35] = (float) myserial_act_t4_internal.servo_2_feedback_update_time_us;
    extra_data_out[36] = (float) myserial_act_t4_internal.servo_3_feedback_update_time_us;
    extra_data_out[37] = (float) myserial_act_t4_internal.servo_4_feedback_update_time_us;
    extra_data_out[38] = (float) myserial_act_t4_internal.servo_5_feedback_update_time_us;
    extra_data_out[39] = (float) myserial_act_t4_internal.servo_6_feedback_update_time_us;
    extra_data_out[40] = (float) myserial_act_t4_internal.servo_7_feedback_update_time_us;
    extra_data_out[41] = (float) myserial_act_t4_internal.servo_8_feedback_update_time_us;
    extra_data_out[42] = (float) myserial_act_t4_internal.servo_9_feedback_update_time_us;
    extra_data_out[43] = (float) myserial_act_t4_internal.servo_10_feedback_update_time_us;
    extra_data_out[44] = (float) feedback_servo_1_lost; 
    extra_data_out[45] = (float) feedback_servo_2_lost;
    extra_data_out[46] = (float) feedback_servo_3_lost;
    extra_data_out[47] = (float) feedback_servo_4_lost;
    extra_data_out[48] = (float) feedback_servo_5_lost;
    extra_data_out[49] = (float) feedback_servo_6_lost;
    extra_data_out[50] = (float) feedback_servo_7_lost;
    extra_data_out[51] = (float) feedback_servo_8_lost;
    extra_data_out[52] = (float) feedback_servo_9_lost;
    extra_data_out[53] = (float) feedback_servo_10_lost;
    extra_data_out[54] = (float) position_ack_servo_1_lost; 
    extra_data_out[55] = (float) position_ack_servo_2_lost;
    extra_data_out[56] = (float) position_ack_servo_3_lost;
    extra_data_out[57] = (float) position_ack_servo_4_lost;
    extra_data_out[58] = (float) position_ack_servo_5_lost;
    extra_data_out[59] = (float) position_ack_servo_6_lost;
    extra_data_out[60] = (float) position_ack_servo_7_lost;
    extra_data_out[61] = (float) position_ack_servo_8_lost;
    extra_data_out[62] = (float) position_ack_servo_9_lost;
    extra_data_out[63] = (float) position_ack_servo_10_lost;

    /* Assign rolling message: */
    myserial_act_t4_out.rolling_msg_out = extra_data_out[rolling_message_out_id_cnt];
    myserial_act_t4_out.rolling_msg_out_id = rolling_message_out_id_cnt;
    rolling_message_out_id_cnt++;
    if (rolling_message_out_id_cnt > 63) rolling_message_out_id_cnt = 0;

    /* Calculate checksum for outbound packet: */
    uint8_t* buf_send = (uint8_t*)&myserial_act_t4_out;
    myserial_act_t4_out.checksum_out = 0;
    for (uint16_t i = 0; i < sizeof(struct serial_act_t4_out) - 1; i++) {
      myserial_act_t4_out.checksum_out += buf_send[i];
    }

    /* Send out packet to buffer: */
    COMMUNICATION_SERIAL.write(START_BYTE_SERIAL_ACT_T4);
    COMMUNICATION_SERIAL.write(buf_send, sizeof(struct serial_act_t4_out));

    ack_comm_TX_flightcontroller = 1;
    last_time_write_to_flightcontroller = 0;
  }

  /* RECEIVING PACKET */
  /* Reset received packets to zero every 5 second to update the statistics */
  if (old_time_frequency_in > COMMON_TMR) {
    serial_act_t4_received_packets = 0;
    old_time_frequency_in = 0;
  }

  /* Collect packets on the buffer if available: */
  while (COMMUNICATION_SERIAL.available()) {
    uint8_t serial_act_t4_byte_in;
    serial_act_t4_byte_in = COMMUNICATION_SERIAL.read();
    if ((serial_act_t4_byte_in == START_BYTE_SERIAL_ACT_T4) || (serial_act_t4_buf_in_cnt > 0)) {
      serial_act_t4_msg_buf_in[serial_act_t4_buf_in_cnt] = serial_act_t4_byte_in;
      serial_act_t4_buf_in_cnt++;
    }

    if (serial_act_t4_buf_in_cnt > sizeof(struct serial_act_t4_in)) {
      serial_act_t4_buf_in_cnt = 0;
      uint8_t checksum_in_local = 0;
      for (uint16_t i = 1; i < sizeof(struct serial_act_t4_in); i++) {
        checksum_in_local += serial_act_t4_msg_buf_in[i];
      }

      if (checksum_in_local == serial_act_t4_msg_buf_in[sizeof(struct serial_act_t4_in)]) {
        serial_act_t4_parse_msg_in();
        serial_act_t4_received_packets++;
        time_no_connection_flightcontroller = 0;
      }
      else {
        serial_act_t4_missed_packets_in++;
      }
    }
  }

  serial_act_t4_message_frequency_in = (uint16_t)((1.0 * serial_act_t4_received_packets / (old_time_frequency_in)) * 1000.0);
  /* Write the frequency to the status led: */
  analogWrite(PIN_LED_T4, serial_act_t4_message_frequency_in * 3);
}

/* Servo routine to be run inside the main loop */
void ServoRoutine(void) {

  if (timer_count_servo >= 2000) {  //Run at ~500 Hz
    timer_count_servo = 0;
    writeEstimatePwmServos();
  }

}

/* Write the PWM servos and estimate the dynamics to be sent back to the flightcontroller */
void writeEstimatePwmServos(void) {

  /* Protection to max and min values, in case the Flightcontroller is not yet connected: */
  if (SERVO_11_MAX_PWM == 0 || SERVO_11_MIN_PWM == 0 || SERVO_11_ZERO_PWM == 0) {
    SERVO_11_MAX_PWM = 1500;
    SERVO_11_MIN_PWM = 1500;
    SERVO_11_ZERO_PWM = 1500;
  }

  if (SERVO_12_MAX_PWM == 0 || SERVO_12_MIN_PWM == 0 || SERVO_12_ZERO_PWM == 0) {
    SERVO_12_MAX_PWM = 1500;
    SERVO_12_MIN_PWM = 1500;
    SERVO_12_ZERO_PWM = 1500;
  }

  /* Apply the pwm values to the servos: */
  int servo11_PWM_value = SERVO_11_ZERO_PWM;

  if (myserial_act_t4_in.servo_11_cmd_int >= 0) {
    servo11_PWM_value += (int)(((myserial_act_t4_in.servo_11_cmd_int / degrees_accuracy_multiplier) / SERVO_11_MAX_ANGLE_DEG) * (SERVO_11_MAX_PWM - SERVO_11_ZERO_PWM));
  }
  else {
    servo11_PWM_value += (int)(((myserial_act_t4_in.servo_11_cmd_int / degrees_accuracy_multiplier) / SERVO_11_MIN_ANGLE_DEG) * (SERVO_11_MIN_PWM - SERVO_11_ZERO_PWM));
  }

  int servo12_PWM_value = SERVO_12_ZERO_PWM;

  if (myserial_act_t4_in.servo_12_cmd_int >= 0) {
    servo12_PWM_value += (int)(((myserial_act_t4_in.servo_12_cmd_int / degrees_accuracy_multiplier) / SERVO_12_MAX_ANGLE_DEG) * (SERVO_12_MAX_PWM - SERVO_12_ZERO_PWM));
  }
  else {
    servo12_PWM_value += (int)(((myserial_act_t4_in.servo_12_cmd_int / degrees_accuracy_multiplier) / SERVO_12_MIN_ANGLE_DEG) * (SERVO_12_MIN_PWM - SERVO_12_ZERO_PWM));
  }

  /* Bound the PWM output: */
  servo11_PWM_value = constrain(servo11_PWM_value, min(SERVO_11_MIN_PWM, SERVO_11_MAX_PWM), max(SERVO_11_MIN_PWM, SERVO_11_MAX_PWM));
  servo12_PWM_value = constrain(servo12_PWM_value, min(SERVO_12_MIN_PWM, SERVO_12_MAX_PWM), max(SERVO_12_MIN_PWM, SERVO_12_MAX_PWM));

  /* WRITE TO PIN: */
  if(Servo_11_arm){
    analogWrite(SERVO11_PWM_PIN, PWM_to_pulse_multiplier * servo11_PWM_value);
  }
  else {
    analogWrite(SERVO11_PWM_PIN, 0.0);
  }

  if(Servo_12_arm){
    analogWrite(SERVO12_PWM_PIN, PWM_to_pulse_multiplier * servo12_PWM_value);
  }
  else {
    analogWrite(SERVO12_PWM_PIN, 0.0);
  }

  /* Servo 11 estimation */
  Servo11_state = -SERVO_11_FIRST_ORD_DYN_DEN * Servo11_state_old + SERVO_11_FIRST_ORD_DYN_NUM * servo_11_state_memory[SERVO_STATE_MEM_BUF_SIZE - SERVO_11_DELAY_TS - 1];
  Servo11_state = constrain(Servo11_state, SERVO_11_MIN_ANGLE_DEG, SERVO_11_MAX_ANGLE_DEG);
  /* Assign servo 11 to telemetry back for the estimation: */
  myserial_act_t4_internal.servo_11_angle_int = (int16_t)(Servo11_state * degrees_accuracy_multiplier);

  /* Servo 12 estimation */
  Servo12_state = -SERVO_12_FIRST_ORD_DYN_DEN * Servo12_state_old + SERVO_12_FIRST_ORD_DYN_NUM * servo_12_state_memory[SERVO_STATE_MEM_BUF_SIZE - SERVO_12_DELAY_TS - 1];
  Servo12_state = constrain(Servo12_state, SERVO_12_MIN_ANGLE_DEG, SERVO_12_MAX_ANGLE_DEG);
  /* Assign servo 12 to telemetry back for the estimation: */
  myserial_act_t4_internal.servo_12_angle_int = (int16_t)(Servo12_state * degrees_accuracy_multiplier);

  /* Assign the memory variables: */
  Servo11_state_old = Servo11_state;
  Servo12_state_old = Servo12_state;
  for (int j = 1; j < SERVO_STATE_MEM_BUF_SIZE; j++) {
    servo_11_state_memory[j - 1] = servo_11_state_memory[j];
    servo_12_state_memory[j - 1] = servo_12_state_memory[j];
  }

  servo_11_state_memory[SERVO_STATE_MEM_BUF_SIZE - 1] = myserial_act_t4_in.servo_11_cmd_int / degrees_accuracy_multiplier;
  servo_12_state_memory[SERVO_STATE_MEM_BUF_SIZE - 1] = myserial_act_t4_in.servo_12_cmd_int / degrees_accuracy_multiplier;

}

/* ESC routine to be run inside the main loop */
void EscRoutine(void) {

  static int ret;

  ret = ESCCMD_tic(); // Keep timing awake
  if (ret == ESCCMD_TIC_OCCURED) {  //run at tick time, around 500 Hz
  }

  if (timer_count_esc >= 2000) {  //Run at about 500 Hz
    timer_count_esc = 0;
  }

}

/* Debug the Ack received by the servos after we set a desired angle */
#ifdef INCLUDE_DEBUGCODE
void DebugServoPosition(void) {

  DEBUG_serial.print("Servo_1_position_deg:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_1_angle_int / degrees_accuracy_multiplier);
  DEBUG_serial.print(",Servo_2_position_deg:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_2_angle_int / degrees_accuracy_multiplier);
  DEBUG_serial.print(",Servo_3_position_deg:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_3_angle_int / degrees_accuracy_multiplier);
  DEBUG_serial.print(",Servo_4_position_deg:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_4_angle_int / degrees_accuracy_multiplier);
  DEBUG_serial.print(",Servo_5_position_deg:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_5_angle_int / degrees_accuracy_multiplier);
  DEBUG_serial.print(",Servo_6_position_deg:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_6_angle_int / degrees_accuracy_multiplier);
  DEBUG_serial.print(",Servo_7_position_deg:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_7_angle_int / degrees_accuracy_multiplier);
  DEBUG_serial.print(",Servo_8_position_deg:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_8_angle_int / degrees_accuracy_multiplier);
  DEBUG_serial.print(",Servo_9_position_deg:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_9_angle_int / degrees_accuracy_multiplier);
  DEBUG_serial.print(",Servo_10_position_deg:");
  DEBUG_serial.println(myserial_act_t4_internal.servo_10_angle_int / degrees_accuracy_multiplier);
}

/* Debug the Ack received by the servos after we set a desired angle */
void DebugServoLostAckPackets(void) {

  DEBUG_serial.print("Ack_S1_lost:");
  DEBUG_serial.print(position_ack_servo_1_lost);
  DEBUG_serial.print(",Ack_S2_lost:");
  DEBUG_serial.print(position_ack_servo_2_lost);
  DEBUG_serial.print(",Ack_S3_lost:");
  DEBUG_serial.print(position_ack_servo_3_lost);
  DEBUG_serial.print(",Ack_S4_lost:");
  DEBUG_serial.print(position_ack_servo_4_lost);
  DEBUG_serial.print(",Ack_S5_lost:");
  DEBUG_serial.print(position_ack_servo_5_lost);
  DEBUG_serial.print(",Ack_S6_lost:");
  DEBUG_serial.print(position_ack_servo_6_lost);
  DEBUG_serial.print(",Ack_S7_lost:");
  DEBUG_serial.print(position_ack_servo_7_lost);
  DEBUG_serial.print(",Ack_S8_lost:");
  DEBUG_serial.print(position_ack_servo_8_lost);
  DEBUG_serial.print(",Ack_S9_lost:");
  DEBUG_serial.print(position_ack_servo_9_lost);
  DEBUG_serial.print(",Ack_S10_lost:");
  DEBUG_serial.println(position_ack_servo_10_lost);

}

/* Displays the amount of lost position feedback packets */
void DebugServoLostFeedbackPackets(void) {

  DEBUG_serial.print("Feedback_S1_lost:");
  DEBUG_serial.print(feedback_servo_1_lost);
  DEBUG_serial.print(",Feedback_S2_lost:");
  DEBUG_serial.print(feedback_servo_2_lost);
  DEBUG_serial.print(",Feedback_S3_lost:");
  DEBUG_serial.print(feedback_servo_3_lost);
  DEBUG_serial.print(",Feedback_S4_lost:");
  DEBUG_serial.print(feedback_servo_4_lost);
  DEBUG_serial.print(",Feedback_S5_lost:");
  DEBUG_serial.print(feedback_servo_5_lost);
  DEBUG_serial.print(",Feedback_S6_lost:");
  DEBUG_serial.print(feedback_servo_6_lost);
  DEBUG_serial.print(",Feedback_S7_lost:");
  DEBUG_serial.print(feedback_servo_7_lost);
  DEBUG_serial.print(",Feedback_S8_lost:");
  DEBUG_serial.print(feedback_servo_8_lost);
  DEBUG_serial.print(",Feedback_S9_lost:");
  DEBUG_serial.print(feedback_servo_9_lost);
  DEBUG_serial.print(",Feedback_S10_lost:");
  DEBUG_serial.println(feedback_servo_10_lost);

}

/* Displays the refresh rate of each servo (not in PWM) */
void DebugUpdateTimePosPackets(void) {

  DEBUG_serial.print("Pos_S1_time_update:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_1_feedback_update_time_us);
  DEBUG_serial.print(",Pos_S2_time_update:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_2_feedback_update_time_us);
  DEBUG_serial.print(",Pos_S3_time_update:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_3_feedback_update_time_us);
  DEBUG_serial.print(",Pos_S4_time_update:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_4_feedback_update_time_us);
  DEBUG_serial.print(",Pos_S5_time_update:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_5_feedback_update_time_us);
  DEBUG_serial.print(",Pos_S6_time_update:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_6_feedback_update_time_us);
  DEBUG_serial.print(",Pos_S7_time_update:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_7_feedback_update_time_us);
  DEBUG_serial.print(",Pos_S8_time_update:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_8_feedback_update_time_us);
  DEBUG_serial.print(",Pos_S9_time_update:");
  DEBUG_serial.print(myserial_act_t4_internal.servo_9_feedback_update_time_us);
  DEBUG_serial.print(",Pos_S10_time_update:");
  DEBUG_serial.println(myserial_act_t4_internal.servo_10_feedback_update_time_us);

}

/* Displays some info about the connection with the flightcontroller */
void DebugConnection(void) {

  DEBUG_serial.print("Missed_packet:");
  DEBUG_serial.print(serial_act_t4_missed_packets_in);
  DEBUG_serial.print(",Frequency_in:");
  DEBUG_serial.println(serial_act_t4_message_frequency_in);

}
#endif //INCLUDE_DEBUGCODE

/* Initialize all the serials of the servos including the tristate mode and PWM servos */
void InitServos(void) {

  SERVO1_serial.begin(BAUDRATE_SERVO);  //1,2,3,4,5 on the same serial, not needed to recall them
  SERVO1_serialEnableOpenDrain(true);

  SERVO6_serial.begin(BAUDRATE_SERVO);  //6,7,8,9,10 on the same serial, not needed to recall them
  SERVO6_serialEnableOpenDrain(true);

  // Serial 1 half duplex mode:
  // s_pkuart_1->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 2 half duplex mode:
  // s_pkuart_2->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 3 half duplex mode:
  // s_pkuart_3->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 4 half duplex mode:
  // s_pkuart_4->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 5 half duplex mode:
  s_pkuart_5->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;  //Servos 1,2,3,4,5 on the same serial, not needed to recall it

  // Serial 6 half duplex mode:
  // s_pkuart_6->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  //Serial 7 half duplex mode:
  s_pkuart_7->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;  //Servos 6,7,8,9,10 on the same serial, not needed to recall it

  /* INIT PWM Servos: */
  pinMode(SERVO11_PWM_PIN, OUTPUT);
  pinMode(SERVO12_PWM_PIN, OUTPUT);
  analogWriteFrequency(SERVO11_PWM_PIN, PWM_SERVO_FREQUENCY); // Servo frequency
  analogWriteFrequency(SERVO12_PWM_PIN, PWM_SERVO_FREQUENCY); // Servo frequency
  analogWriteResolution(12);                                  // 4096 values for duty cycle
  PWM_to_pulse_multiplier = (1e-6 * 4096.0 * PWM_SERVO_FREQUENCY);

}

/* Send instructions to servo 1 */
void SendInstructionServo1(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }

  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo */
  SERVO1_serial.clear();
  SERVO1_serialEnableOpenDrain(false);
  SERVO1_serial.write(buffer_tx, buffer_tx_idx);
  SERVO1_serialEnableOpenDrain(true);

#ifdef VERBOSE_MESSAGE
  PrintHex("Instruction sent servo 1: ", buffer_tx, buffer_tx_idx);
#endif
}

/* Send instructions to servo 2 */
void SendInstructionServo2(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }

  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo */
  SERVO2_serial.clear();
  SERVO2_serialEnableOpenDrain(false);
  SERVO2_serial.write(buffer_tx, buffer_tx_idx);
  SERVO2_serialEnableOpenDrain(true);

#ifdef VERBOSE_MESSAGE
  PrintHex("Instruction sent servo 2: ", buffer_tx, buffer_tx_idx);
#endif
}

/* Send instructions to servo 3 */
void SendInstructionServo3(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }

  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo */
  SERVO3_serial.clear();
  SERVO3_serialEnableOpenDrain(false);
  SERVO3_serial.write(buffer_tx, buffer_tx_idx);
  SERVO3_serialEnableOpenDrain(true);

#ifdef VERBOSE_MESSAGE
  PrintHex("Instruction sent servo 3: ", buffer_tx, buffer_tx_idx);
#endif
}

/* Send instructions to servo 4 */
void SendInstructionServo4(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }

  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo */
  SERVO4_serial.clear();
  SERVO4_serialEnableOpenDrain(false);
  SERVO4_serial.write(buffer_tx, buffer_tx_idx);
  SERVO4_serialEnableOpenDrain(true);

#ifdef VERBOSE_MESSAGE
  PrintHex("Instruction sent servo 4: ", buffer_tx, buffer_tx_idx);
#endif
}

/* Send instructions to servo 5 */
void SendInstructionServo5(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }

  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo */
  SERVO5_serial.clear();
  SERVO5_serialEnableOpenDrain(false);
  SERVO5_serial.write(buffer_tx, buffer_tx_idx);
  SERVO5_serialEnableOpenDrain(true);

#ifdef VERBOSE_MESSAGE
  PrintHex("Instruction sent servo 5: ", buffer_tx, buffer_tx_idx);
#endif
}

/* Send instructions to servo 6 */
void SendInstructionServo6(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }

  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo  */
  SERVO6_serial.clear();
  SERVO6_serialEnableOpenDrain(false);
  SERVO6_serial.write(buffer_tx, buffer_tx_idx);
  SERVO6_serialEnableOpenDrain(true);

  #ifdef VERBOSE_MESSAGE
    PrintHex("Instruction sent servo 6: ", buffer_tx, buffer_tx_idx);
  #endif
}

/* Send instructions to servo 7 */
void SendInstructionServo7(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }

  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo */
  SERVO7_serial.clear();
  SERVO7_serialEnableOpenDrain(false);
  SERVO7_serial.write(buffer_tx, buffer_tx_idx);
  SERVO7_serialEnableOpenDrain(true);

  #ifdef VERBOSE_MESSAGE
    PrintHex("Instruction sent servo 7: ", buffer_tx, buffer_tx_idx);
  #endif
}

/* Send instructions to servo 8 */
void SendInstructionServo8(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }

  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo */
  SERVO8_serial.clear();
  SERVO8_serialEnableOpenDrain(false);
  SERVO8_serial.write(buffer_tx, buffer_tx_idx);
  SERVO8_serialEnableOpenDrain(true);

  #ifdef VERBOSE_MESSAGE
    PrintHex("Instruction sent servo 8: ", buffer_tx, buffer_tx_idx);
  #endif
}

/* Send instructions to servo 9 */
void SendInstructionServo9(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }

  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo */
  SERVO9_serial.clear();
  SERVO9_serialEnableOpenDrain(false);
  SERVO9_serial.write(buffer_tx, buffer_tx_idx);
  SERVO9_serialEnableOpenDrain(true);

  #ifdef VERBOSE_MESSAGE
    PrintHex("Instruction sent servo 9: ", buffer_tx, buffer_tx_idx);
  #endif
}

/* Send instructions to servo 10 */
void SendInstructionServo10(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
  /* -------- SEND INSTRUCTION ------------ */
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++) {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++) {
    u8_Checksum += buffer_tx[i];
  }
  
  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  /* Send the instruction to servo */
  SERVO10_serial.clear();
  SERVO10_serialEnableOpenDrain(false);
  SERVO10_serial.write(buffer_tx, buffer_tx_idx);
  SERVO10_serialEnableOpenDrain(true);

  #ifdef VERBOSE_MESSAGE
    PrintHex("Instruction sent servo 10: ", buffer_tx, buffer_tx_idx);
  #endif
}

/* Print a desired buffer in HEX mode */
#ifdef VERBOSE_MESSAGE
void PrintHex(const char* s8_Text, const byte* data, int count) {
  DEBUG_serial.print(s8_Text);

  for (int i = 0; i < count; i++) {
    if (i > 0)
      DEBUG_serial.print(" ");

    if (data[i] <= 0xF)
      DEBUG_serial.print("0");

    DEBUG_serial.print(data[i], HEX);
  }
  DEBUG_serial.println();

}
#endif

/* 1 16-bit split into 2 8 digits */
void SplitByte(uint8_t* DataL, uint8_t* DataH, uint16_t Data) {

  *DataH = (Data >> 8);
  *DataL = (Data & 0xff);

}

/* 2 8-digit combinations for 1 16-digit number */
uint16_t CompactBytes(uint8_t DataL, uint8_t DataH) {

  uint16_t Data;
  Data = DataL;
  Data <<= 8;
  Data |= DataH;
  return Data;

}

/* Keep track of the tick to Servos and flightcontroller communication */
void ServosAndCommTic(void) {

  /* Servo tic */
  if (ack_write_read && last_time_write_read_servo_cnt >= SERVO_COMM_MARGIN) {
    iter_counter_SERVO++;
    ack_write_read = 0;
    if (iter_counter_SERVO > 20) {
      iter_counter_SERVO = 1;
    }
  }

  /* Communication tic */
  if (ack_comm_TX_flightcontroller && last_time_write_to_flightcontroller >= comm_refresh_time) {
    ack_comm_TX_flightcontroller = 0;
  }

}

/* Interrupt routine to read and write to servos */
void writeReadServos(void) {

  byte buffer_servo[50] = { 0 };
  int buffer_servo_idx = 0;
  byte u8_Data_1[7] = { 0 };  //Send position frame
  byte u8_Data_2[2] = { 0 };  //request data frame

  if (ack_write_read == 0) {

    /* Receive feedback from servo 5 */
    if (iter_counter_SERVO == 1) {  
      while (SERVO5_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO5_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 5: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_5_feedback_update_time_us = (int16_t)feedback_5_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_5_ID) {

        /* Position */
        if(servo_5_is_SCS){
          myserial_act_t4_internal.servo_5_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_5_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_5);
        }
        else {
          myserial_act_t4_internal.servo_5_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_5_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_5);
        }
        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_5_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_5_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_5_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_5_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_5_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_5_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_5_time_uS_counter = 0;
      }
      else {
        feedback_servo_5_lost++;
      }
      SERVO5_serial.flush();
    }

    /* Receive feedback from servo 10 */
    if (iter_counter_SERVO == 2) {  
      while (SERVO10_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO10_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 10: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_10_feedback_update_time_us = (int16_t)feedback_10_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_10_ID) {

        /* Position */
        if(servo_10_is_SCS){
          myserial_act_t4_internal.servo_10_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_10_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_10);
        }
        else {
          myserial_act_t4_internal.servo_10_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_10_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_10);
        }

        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_10_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_10_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_10_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_10_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_10_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_10_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_10_time_uS_counter = 0;
      }
      else {
        feedback_servo_10_lost++;
      }
      SERVO10_serial.flush();
    }

    /* Send position target to servo 1 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 1) {  
      if (Servo_1_arm) {
        if(servo_1_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_1);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo1(Servo_1_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 1          
        }
        else {
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_1);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo1(Servo_1_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 1
        }
      } 
      else { 
        /* disable torque on servo */
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo1(Servo_1_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }
    
    /* Send position target to servo 6 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 2) {  
      if (Servo_6_arm) {
        if(servo_6_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_6);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo6(Servo_6_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 6
        }
        else {
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_6);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo6(Servo_6_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 6
        }
      } 
      else { 
        /* disable torque on servo */
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo6(Servo_6_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    /* Receive Ack from servo 1 */
    if (iter_counter_SERVO == 3) {  
      /* Collect Ack response from servo 1 */
      while (SERVO1_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO1_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 1: ", &buffer_servo[0], 6);
      #endif
      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_1_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_1_lost++;
      }
      SERVO1_serial.flush();
    }

    /* Receive Ack from servo 6 */
    if (iter_counter_SERVO == 4) {  
      /* Collect Ack response from servo 6 */
      while (SERVO6_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO6_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 6: ", &buffer_servo[0], 6);
      #endif
      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_6_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_6_lost++;
      }
      SERVO6_serial.flush();
    }

    /* Send position target to servo 2 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 3) {  
      if (Servo_2_arm) {
        if(servo_2_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_2);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo2(Servo_2_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 2
        }
        else { 
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_2);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo2(Servo_2_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 2
        }
      } 
      else { 
        /* disable torque on servo */
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo2(Servo_2_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    /* Send position target to servo 7 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 4) {  
      if (Servo_7_arm) {
        if(servo_7_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_7);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo7(Servo_7_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 7
        }
        else {
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_7);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo7(Servo_7_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 7
        }
      } 
      else { 
        /* disable torque on servo */
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo7(Servo_7_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    /* Receive Ack from servo 2 */
    if (iter_counter_SERVO == 5) {  
      /* Collect Ack response from servo 2 */
      while (SERVO2_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO2_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 2: ", &buffer_servo[0], 6);
      #endif
      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_2_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_2_lost++;
      }
      SERVO2_serial.flush();
    }

    /* Receive Ack from servo 7 */
    if (iter_counter_SERVO == 6) {  
      /* Collect Ack response from servo 7 */
      while (SERVO7_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO7_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 7: ", &buffer_servo[0], 6);
      #endif
      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_7_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_7_lost++;
      }
      SERVO7_serial.flush();
    }

    /* Send position target to servo 3 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 5) {
      if (Servo_3_arm) {
        if(servo_3_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_3);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo3(Servo_3_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 3
        }
        else {
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_3);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo3(Servo_3_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 3
        }
      } 
      else { 
        //disable torque on servo
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo3(Servo_3_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    /* Send position target to servo 8 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 6) {  
      if (Servo_8_arm) {
        if(servo_8_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_8);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo8(Servo_8_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 8
        }
        else {
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_8);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo8(Servo_8_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 8
        }
      }
      else { 
        //disable torque on servo
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo8(Servo_8_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    /* Receive Ack from servo 3  */
    if (iter_counter_SERVO == 7) {  
      /* Collect Ack response from servo 3 */
      while (SERVO3_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO3_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 3: ", &buffer_servo[0], 6);
      #endif

      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_3_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_3_lost++;
      }
      SERVO3_serial.flush();
    }

    /* Receive Ack from servo 8 */
    if (iter_counter_SERVO == 8) {  
      /* Collect Ack response from servo 8 */
      while (SERVO8_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO8_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 8: ", &buffer_servo[0], 6);
      #endif

      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_8_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_8_lost++;
      }
      SERVO8_serial.flush();
    }

    /* Send position target to servo 4 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 7) { 
      if (Servo_4_arm) {
        if(servo_4_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_4);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo4(Servo_4_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 4
        }
        else {
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_4);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo4(Servo_4_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 4
        }
      }
      else {
        //disable torque on servo
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo4(Servo_4_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    /* Send position target to servo 9 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 8) {
      if (Servo_9_arm) {
        if(servo_9_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_9);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo9(Servo_9_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 9
        }
        else {
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_9);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo9(Servo_9_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 9
        }
      } 
      else {
        //disable torque on servo
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo9(Servo_9_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    /* Receive Ack from servo 4 */
    if (iter_counter_SERVO == 9) {  
      /* Collect Ack response from servo 4 */
      while (SERVO4_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO4_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 4: ", &buffer_servo[0], 6);
      #endif
      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_4_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_4_lost++;
      }
      SERVO4_serial.flush();
    }

    /* Receive Ack from servo 9 */
    if (iter_counter_SERVO == 10) {  
      /* Collect Ack response from servo 9 */
      while (SERVO9_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO9_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 9: ", &buffer_servo[0], 6);
      #endif
      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_9_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_9_lost++;
      }
      SERVO9_serial.flush();
    }

    /* Send position target to servo 5 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 9) {  
      if (Servo_5_arm) {
        if(servo_5_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_5);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo5(Servo_5_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 5
        }
        else {
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_5);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo5(Servo_5_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 5
        }
      } 
      else { 
        //disable torque on servo
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo5(Servo_5_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    /* Send position target to servo 10 or disable torque in case of servo KILL */
    if (iter_counter_SERVO == 10) {  
      if (Servo_10_arm) {
        if(servo_10_is_SCS){
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[2], &u8_Data_1[1], Target_position_servo_10);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo10(Servo_10_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 10
        }
        else {
          u8_Data_1[0] = SBS_GOAL_POSITION_L;
          SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_10);
          SplitByte(&u8_Data_1[4], &u8_Data_1[3], 0);
          SplitByte(&u8_Data_1[6], &u8_Data_1[5], 0);
          SendInstructionServo10(Servo_10_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));  //Servo 10
        }
      } 
      else { 
        //disable torque on servo
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo10(Servo_10_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    /* Receive Ack from servo 5 */
    if (iter_counter_SERVO == 11) {  
      /* Collect Ack response from servo 5 */
      while (SERVO5_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO5_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 5: ", &buffer_servo[0], 6);
      #endif
      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_5_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_5_lost++;
      }
      SERVO5_serial.flush();
    }

    /* Receive Ack from servo 10 */
    if (iter_counter_SERVO == 12) {  
      /* Collect Ack response from servo 10*/
      while (SERVO10_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO10_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++) {  //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 10: ", &buffer_servo[0], 6);
      #endif
      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_10_ID) == 0) {  //If checksum is incorrect, increase counter.
        position_ack_servo_10_lost++;
      }
      SERVO10_serial.flush();
    }
    
    /* Request feedback from servo 1 */
    if (iter_counter_SERVO == 11) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo1(Servo_1_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Request feedback from servo 6 */
    if (iter_counter_SERVO == 12) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo6(Servo_6_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Receive feedback from servo 1 */
    if (iter_counter_SERVO == 13) {  
      while (SERVO1_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO1_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 1: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_1_feedback_update_time_us = (int16_t)feedback_1_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_1_ID) {

        /* Position */
        if(servo_1_is_SCS){
          myserial_act_t4_internal.servo_1_angle_int = (int16_t)((CompactBytes(buffer_servo[5], buffer_servo[6]) - servo_1_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_1);
        }
        else {
          myserial_act_t4_internal.servo_1_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_1_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_1);
        }
        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_1_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_1_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_1_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_1_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_1_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_1_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_1_time_uS_counter = 0;
      } 
      else {
        feedback_servo_1_lost++;
      }
      SERVO1_serial.flush();
    }

    //Receive feedback from servo 6
    if (iter_counter_SERVO == 14) {  
      while (SERVO6_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO6_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 6: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_6_feedback_update_time_us = (int16_t)feedback_6_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_6_ID) {

        /* Position */
        if(servo_6_is_SCS){
          myserial_act_t4_internal.servo_6_angle_int = (int16_t)((CompactBytes(buffer_servo[5], buffer_servo[6]) - servo_6_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_6);
        }
        else {
          myserial_act_t4_internal.servo_6_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_6_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_6);
        }
        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_6_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_6_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_6_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_6_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_6_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_6_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_6_time_uS_counter = 0;
      } 
      else {
        feedback_servo_6_lost++;
      }
      SERVO1_serial.flush();
    }

    /* Request feedback from servo 2 */
    if (iter_counter_SERVO == 13) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo2(Servo_2_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Request feedback from servo 7 */
    if (iter_counter_SERVO == 14) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo7(Servo_7_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Receive feedback from servo 2 */
    if (iter_counter_SERVO == 15) {  
      while (SERVO2_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO2_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 2: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_2_feedback_update_time_us = (int16_t)feedback_2_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_2_ID) {

        /* Position */
        if(servo_2_is_SCS){
          myserial_act_t4_internal.servo_2_angle_int = (int16_t)((CompactBytes(buffer_servo[5], buffer_servo[6]) - servo_2_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_2);
        }
        else {
          myserial_act_t4_internal.servo_2_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_2_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_2);
        }
        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_2_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_2_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_2_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_2_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_2_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_2_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_2_time_uS_counter = 0;
      }
      else {
        feedback_servo_2_lost++;
      }
      SERVO2_serial.flush();
    }

    /* Receive feedback from servo 7 */
    if (iter_counter_SERVO == 16) {  
      while (SERVO7_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO7_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 7: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_7_feedback_update_time_us = (int16_t)feedback_7_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_7_ID) {

        /* Position */
        if(servo_7_is_SCS){
          myserial_act_t4_internal.servo_7_angle_int = (int16_t)((CompactBytes(buffer_servo[5], buffer_servo[6]) - servo_7_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_7);
        }
        else {
          myserial_act_t4_internal.servo_7_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_7_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_7);
        }
        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_7_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_7_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_7_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_7_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_7_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_7_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_7_time_uS_counter = 0;
      }
      else {
        feedback_servo_7_lost++;
      }
      SERVO7_serial.flush();
    }

    /* Request feedback from servo 3 */
    if (iter_counter_SERVO == 15) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo3(Servo_3_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Request feedback from servo 8 */
    if (iter_counter_SERVO == 16) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo8(Servo_8_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Receive feedback from servo 3 */
    if (iter_counter_SERVO == 17) {  
      while (SERVO3_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO3_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 3: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_3_feedback_update_time_us = (int16_t)feedback_3_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_3_ID) {

        /* Position */
        if(servo_3_is_SCS){
          myserial_act_t4_internal.servo_3_angle_int = (int16_t)((CompactBytes(buffer_servo[5], buffer_servo[6]) - servo_3_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_3);
        }
        else {
          myserial_act_t4_internal.servo_3_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_3_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_3);
        }
        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_3_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_3_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_3_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_3_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_3_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_3_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_3_time_uS_counter = 0;
      }
      else {
        feedback_servo_3_lost++;
      }
      SERVO3_serial.flush();
    }

    /* Receive feedback from servo 8 */
    if (iter_counter_SERVO == 18) {  
      while (SERVO8_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO8_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 8: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_8_feedback_update_time_us = (int16_t)feedback_8_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_8_ID) {

        /* Position */
        if(servo_8_is_SCS){
          myserial_act_t4_internal.servo_8_angle_int = (int16_t)((CompactBytes(buffer_servo[5], buffer_servo[6]) - servo_8_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_8);
        }
        else {
          myserial_act_t4_internal.servo_8_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_8_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_8);
        }
        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_8_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_8_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_8_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_8_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_8_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_8_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_8_time_uS_counter = 0;
      }
      else {
        feedback_servo_8_lost++;
      }
      SERVO8_serial.flush();
    }

    /* Request feedback from servo 4 */
    if (iter_counter_SERVO == 17) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo4(Servo_4_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Request feedback from servo  9 */
    if (iter_counter_SERVO == 18) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo9(Servo_9_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Receive feedback from servo 4 */
    if (iter_counter_SERVO == 19) {  
      while (SERVO4_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO4_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 4: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_4_feedback_update_time_us = (int16_t)feedback_4_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_4_ID) {

        /* Position */
        if(servo_4_is_SCS){
          myserial_act_t4_internal.servo_4_angle_int = (int16_t)((CompactBytes(buffer_servo[5], buffer_servo[6]) - servo_4_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_4);
        }
        else {
          myserial_act_t4_internal.servo_4_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_4_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_4);
        }
        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_4_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_4_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_4_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_4_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_4_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_4_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_4_time_uS_counter = 0;
      }
      else {
        feedback_servo_4_lost++;
      }
      SERVO4_serial.flush();
    }

    /* Receive feedback from servo 9 */
    if (iter_counter_SERVO == 20) {  
      while (SERVO9_serial.available()) {
        buffer_servo[buffer_servo_idx] = SERVO9_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++) {  //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
            PrintHex("Instruction received from servo 9: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_internal.servo_9_feedback_update_time_us = (int16_t)feedback_9_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_9_ID) {

        /* Position */
        if(servo_9_is_SCS){
          myserial_act_t4_internal.servo_9_angle_int = (int16_t)((CompactBytes(buffer_servo[5], buffer_servo[6]) - servo_9_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_9);
        }
        else {
          myserial_act_t4_internal.servo_9_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) - servo_9_max_command/2) * (360*degrees_accuracy_multiplier) / steps_for_full_rotation_servo_9);
        }
        /* Speed */
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_internal.servo_9_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign) {
          myserial_act_t4_internal.servo_9_speed_int *= -1;
        }

        /* Load */
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_internal.servo_9_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign) {
          myserial_act_t4_internal.servo_9_load_int *= -1;
        }

        /* Voltage */
        myserial_act_t4_internal.servo_9_volt_int = (int16_t)(buffer_servo[11]);

        /* Temperature */
        myserial_act_t4_internal.servo_9_temp_int = (int16_t)(buffer_servo[12]); 
               
        feedback_9_time_uS_counter = 0;
      }
      else {
        feedback_servo_9_lost++;
      }
      SERVO9_serial.flush();
    }

    /* Request feedback from servo 5 */
    if (iter_counter_SERVO == 19) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo5(Servo_5_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Request feedback from servo 10 */
    if (iter_counter_SERVO == 20) {  
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8;  //We want position, speed, load, voltage and temperature out!
      SendInstructionServo10(Servo_10_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    /* Say to the tick that it can now increase the counter reporting also the time information */
    ack_write_read = 1;
    last_time_write_read_servo_cnt = 0;
  }
}

/* Active or deactivate Serial 1 tristate mode for half duplex com. */
void Serial1EnableOpenDrain(bool bEnable) {
  if (bEnable) {
    Serial1.flush();                         // Make sure we output everything first before changing state.
    s_pkuart_1->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
  } 
  else {
    s_pkuart_1->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
  }
}

/* Active or deactivate Serial 2 tristate mode for half duplex com. */
void Serial2EnableOpenDrain(bool bEnable) {
  if (bEnable) {
    Serial2.flush();                         // Make sure we output everything first before changing state.
    s_pkuart_2->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
  } else {
    s_pkuart_2->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
  }
}

/* Active or deactivate Serial 3 tristate mode for half duplex com. */
void Serial3EnableOpenDrain(bool bEnable) {
  if (bEnable) {
    Serial3.flush();                         // Make sure we output everything first before changing state.
    s_pkuart_3->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
  } 
  else {
    s_pkuart_3->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
  }
}

/* Active or deactivate Serial 4 tristate mode for half duplex com. */
void Serial4EnableOpenDrain(bool bEnable) {
  if (bEnable) {
    Serial4.flush();                         // Make sure we output everything first before changing state.
    s_pkuart_4->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
  } 
  else {
    s_pkuart_4->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
  }
}

/* Active or deactivate Serial 5 tristate mode for half duplex com. */
void Serial5EnableOpenDrain(bool bEnable) {
  if (bEnable) {
    Serial5.flush();                         // Make sure we output everything first before changing state.
    s_pkuart_5->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
  } 
  else {
    s_pkuart_5->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
  }
}

/* Active or deactivate Serial 6 tristate mode for half duplex com. */
void Serial6EnableOpenDrain(bool bEnable) {
  if (bEnable) {
    Serial6.flush();                         // Make sure we output everything first before changing state.
    s_pkuart_6->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
  } 
  else {
    s_pkuart_6->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
  }
}

/* Active or deactivate Serial 7 tristate mode for half duplex com. */
void Serial7EnableOpenDrain(bool bEnable) {
  if (bEnable) {
    Serial7.flush();                         // Make sure we output everything first before changing state.
    s_pkuart_7->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
  } 
  else {
    s_pkuart_7->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
  }
}

#ifdef INCLUDE_DEBUGCODE
void DisplayEscVoltage(void) {
  uint16_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;
  ESCCMD_read_volt(0, &local_var1);
  DEBUG_serial.print("Voltage_ESC_1:");
  DEBUG_serial.print(local_var1 / 100.0);
  DEBUG_serial.print(",");
  ESCCMD_read_volt(1, &local_var2);
  DEBUG_serial.print("Voltage_ESC_2:");
  DEBUG_serial.print(local_var2 / 100.0);
  DEBUG_serial.print(",");
  ESCCMD_read_volt(2, &local_var3);
  DEBUG_serial.print("Voltage_ESC_3:");
  DEBUG_serial.print(local_var3 / 100.0);
  DEBUG_serial.print(",");
  ESCCMD_read_volt(3, &local_var4);
  DEBUG_serial.print("Voltage_ESC_4:");
  DEBUG_serial.println(local_var4 / 100.0);
}

void DisplayEscCurrent(void) {
  uint16_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;
  ESCCMD_read_amp(0, &local_var1);
  DEBUG_serial.print("Current_ESC_1:");
  DEBUG_serial.print(local_var1 / 100.0);
  DEBUG_serial.print(",");
  ESCCMD_read_amp(1, &local_var2);
  DEBUG_serial.print("Current_ESC_2:");
  DEBUG_serial.print(local_var2 / 100.0);
  DEBUG_serial.print(",");
  ESCCMD_read_amp(2, &local_var3);
  DEBUG_serial.print("Current_ESC_3:");
  DEBUG_serial.print(local_var3 / 100.0);
  DEBUG_serial.print(",");
  ESCCMD_read_amp(3, &local_var4);
  DEBUG_serial.print("Current_ESC_4:");
  DEBUG_serial.println(local_var4 / 100.0);
}

void DisplayEscRpm(void) {
  int16_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;
  ESCCMD_read_rpm(0, &local_var1);
  DEBUG_serial.print("Rpm_ESC_1:");
  DEBUG_serial.print(local_var1);
  DEBUG_serial.print(",");
  ESCCMD_read_rpm(1, &local_var2);
  DEBUG_serial.print("Rpm_ESC_2:");
  DEBUG_serial.print(local_var2);
  DEBUG_serial.print(",");
  ESCCMD_read_rpm(2, &local_var3);
  DEBUG_serial.print("Rpm_ESC_3:");
  DEBUG_serial.print(local_var3);
  DEBUG_serial.print(",");
  ESCCMD_read_rpm(3, &local_var4);
  DEBUG_serial.print("Rpm_ESC_4:");
  DEBUG_serial.println(local_var4);
}

void DisplayEscTemp(void) {
  uint8_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;
  ESCCMD_read_deg(0, &local_var1);
  DEBUG_serial.print("Temperature_ESC_1:");
  DEBUG_serial.print(local_var1);
  DEBUG_serial.print(",");
  ESCCMD_read_deg(1, &local_var2);
  DEBUG_serial.print("Temperature_ESC_2:");
  DEBUG_serial.print(local_var2);
  DEBUG_serial.print(",");
  ESCCMD_read_deg(2, &local_var3);
  DEBUG_serial.print("Temperature_ESC_3:");
  DEBUG_serial.print(local_var3);
  DEBUG_serial.print(",");
  ESCCMD_read_deg(3, &local_var4);
  DEBUG_serial.print("Temperature_ESC_4:");
  DEBUG_serial.println(local_var4);
}

void DisplayEscErr(void) {
  int8_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;
  ESCCMD_read_err(0, &local_var1);
  DEBUG_serial.print("Error_ESC_1:");
  DEBUG_serial.print(local_var1);
  DEBUG_serial.print(",");
  ESCCMD_read_err(1, &local_var2);
  DEBUG_serial.print("Error_ESC_2:");
  DEBUG_serial.print(local_var2);
  DEBUG_serial.print(",");
  ESCCMD_read_err(2, &local_var3);
  DEBUG_serial.print("Error_ESC_3:");
  DEBUG_serial.print(local_var3);
  DEBUG_serial.print(",");
  ESCCMD_read_err(3, &local_var4);
  DEBUG_serial.print("Error_ESC_4:");
  DEBUG_serial.println(local_var4);
}

void DisplayEscCmd(void) {
  uint16_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;
  ESCCMD_read_cmd(0, &local_var1);
  DEBUG_serial.print("Cmd_ESC_1:");
  DEBUG_serial.print(local_var1);
  DEBUG_serial.print(",");
  ESCCMD_read_cmd(1, &local_var2);
  DEBUG_serial.print("Cmd_ESC_2:");
  DEBUG_serial.print(local_var2);
  DEBUG_serial.print(",");
  ESCCMD_read_cmd(2, &local_var3);
  DEBUG_serial.print("Cmd_ESC_3:");
  DEBUG_serial.print(local_var3);
  DEBUG_serial.print(",");
  ESCCMD_read_cmd(3, &local_var4);
  DEBUG_serial.print("Cmd_ESC_4:");
  DEBUG_serial.println(local_var4);
}
#endif //INCLUDE_DEBUGCODE
