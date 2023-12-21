/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */

#include <TMCStepper.h>
#include <HardwareSerial.h>

#define EN_PIN            5 // Enable
// #define DIR_PIN          26 // Direction
// #define STEP_PIN         25 // Step
#define DIAG_PIN_2       19 // Diag Pin??
#define CS_PIN           42 // Chip select
#define SW_MOSI          66 // Software Master Out Slave In (MOSI)
#define SW_MISO          19 // Software Master In Slave Out (MISO)
#define SW_SCK           64 // Software Slave Clock (SCK)
#define SW_RX            16 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            17 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
//TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
//TMC2660Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2660Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
//TMC5160Stepper driver(CS_PIN, R_SENSE);
//TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

//TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);                     // Hardware Serial
//TMC2208Stepper driver(SW_RX, SW_TX, R_SENSE);                     // Software serial
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
// TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);

// Define a struct to hold stepper motor information
struct Stepper {
  int ENAPin;
  int DIRPin;
  int STPPin;
  int stepsPerRev;            // Steps per revolution (pulses per revolution)
  float speed_deg_s;          // Speed in degrees per second
  float lead;                 // Lead in millimeters for linear motion (set to 0 for rotary motion)
};

// Function to create and initialize a stepper motor
Stepper createStepper(int ENAPin, int DIRPin, int STPPin, int stepsPerRev, float speed_deg_s, float lead = 0) {
  Stepper stepper;
  stepper.ENAPin = ENAPin;
  stepper.DIRPin = DIRPin;
  stepper.STPPin = STPPin;
  stepper.stepsPerRev = stepsPerRev;
  stepper.speed_deg_s = speed_deg_s;
  stepper.lead = lead;

  pinMode(stepper.ENAPin, OUTPUT);
  digitalWrite(stepper.ENAPin, LOW);    // Enable driver in hardware

  pinMode(stepper.DIRPin, OUTPUT);
  pinMode(stepper.STPPin, OUTPUT);

  // Additional setup code specific to your stepper driver or configuration, if required

  return stepper;
}

// Create Steppers
//                   EXAMPLE = createStepper(ENAPin,  DIRPin,  STPPin, stepsPerRev,  speed_deg_s,  lead)
Stepper testStepper          = createStepper(    5,       26,      25,         200,       2000.0,   5.0); 

// Function to rotate the stepper motor to a specified angle (for rotary motion)
void rotateStepper(Stepper& stepper, float angle_degrees, float speed_deg_s = -1) {
  float set_speed = (speed_deg_s != -1) ? speed_deg_s : stepper.speed_deg_s;


  // Convert angle in degrees to steps
  long steps = angle_degrees * stepper.stepsPerRev / 360;

  // Set the direction based on the sign of steps
  if (steps >= 0) {
    digitalWrite(stepper.DIRPin, HIGH); // Clockwise
  } else {
    digitalWrite(stepper.DIRPin, LOW); // Counterclockwise
    steps = -steps;
  }

  float velocity_microseconds = 1000000.0 / (set_speed * stepper.stepsPerRev / 360); // Convert from deg/s to microseconds per step

  unsigned long prevTime_micros = micros();

  for (long i = 0; i < steps; i++) {
    digitalWrite(stepper.STPPin, HIGH);
    delayMicroseconds(1); // Ensure a minimum pulse width
    digitalWrite(stepper.STPPin, LOW);

    // Wait for the remaining time to keep the desired velocity
    unsigned long currentTime_micros = micros();
    unsigned long elapsedTime_micros = currentTime_micros - prevTime_micros;
    if (elapsedTime_micros < velocity_microseconds) {
      delayMicroseconds(velocity_microseconds - elapsedTime_micros);
    }

    prevTime_micros = micros();
  }
}

// Function to move the stepper motor to a specified position in millimeters (for linear motion)
void moveStepper(Stepper& stepper, float distance_mm, float speed_deg_s = -1) {
  float set_speed = (speed_deg_s != -1) ? speed_deg_s : stepper.speed_deg_s;

    
  if (stepper.lead == 0) {
    // For rotary motion, just use rotateStepper as before
    rotateStepper(stepper, distance_mm, speed_deg_s);
  } else {
    // For linear motion, convert distance in millimeters to steps
    long targetSteps = distance_mm * stepper.stepsPerRev / stepper.lead;
    long currentSteps = 0;

    // Set the direction based on the sign of targetSteps
    if (targetSteps >= 0) {
      digitalWrite(stepper.DIRPin, HIGH); // Move forward
    } else {
      digitalWrite(stepper.DIRPin, LOW); // Move backward
      targetSteps = -targetSteps;
    }

    float velocity_microseconds = 1000000.0 / (set_speed * stepper.stepsPerRev / 360); // Convert from deg/s to microseconds per step

    unsigned long prevTime_micros = micros();

    while (currentSteps < targetSteps) {
      digitalWrite(stepper.STPPin, HIGH);
      delayMicroseconds(1); // Ensure a minimum pulse width
      digitalWrite(stepper.STPPin, LOW);

      // Wait for the remaining time to keep the desired velocity
      unsigned long currentTime_micros = micros();
      unsigned long elapsedTime_micros = currentTime_micros - prevTime_micros;
      if (elapsedTime_micros < velocity_microseconds) {
        delayMicroseconds(velocity_microseconds - elapsedTime_micros);
      }

      prevTime_micros = micros();

      currentSteps++;
    }
  }
}

// TMC2209 Simple Movement

// void setup() {
//   pinMode(EN_PIN, OUTPUT);
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
//   digitalWrite(EN_PIN, LOW);      // Enable driver in hardware

//                                   // Enable one according to your setup
// //SPI.begin();                    // SPI drivers
// //SERIAL_PORT.begin(115200);      // HW UART drivers
// driver.beginSerial(115200);     // SW UART drivers

//   driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
//                                   // UART: Init SW UART (if selected) with default 115200 baudrate
//   driver.toff(5);                 // Enables driver in software
//   driver.rms_current(600);        // Set motor RMS current
//   driver.microsteps(16);          // Set microsteps to 1/16th

// //driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
// //driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
//   driver.pwm_autoscale(true);     // Needed for stealthChop
// }

// bool shaft = false;

// void loop() {
//   // Run 5000 steps and switch direction in software
//   digitalWrite(DIR_PIN, HIGH);
//   for (uint16_t i = 5000; i>0; i--) {
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(160);
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(160);
//   }
//   shaft = !shaft;
//   driver.shaft(shaft);
// }

// Stepper Starup
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.print("======================New Start======================= \n");

  driver.begin();
  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(1000);
  driver.microsteps(16);
  driver.shaft(false);
  driver.pwm_autoscale(true);     // Needed for stealthChop
}

int key;
float layerHeight = 0.100;    // layer height for the model (mm)
int stage_speed_fast = 15000;
int stage_speed_med  = 12500;
int stage_speed_slow = 10000;

int speed1 = 10000;
int speed1_5 = 15000;
int speed2 = 20000;
int speed2_5 = 25000;
int speed3 = 30000;
int speed3_5 = 35000;
int speed4 = 40000;
int speed4_5 = 45000;
int speed5 = 50000;
int speed5_5 = 55000;
int speed6 = 60000;
int speed6_5 = 65000;
int speed7 = 70000;
int speed7_3 = 73000;
int speed7_7 = 77000;
int speed8 = 80000;
int speed8_3 = 83000;
int speed8_7 = 87000;
int speed9 = 90000;
int speed9_3 = 93000;
int speed9_7 = 97000;
int speed10 = 100000;
int speed10_25 = 102500;
int speed10_3 = 103000;
int speed10_5 = 105000;
int speed10_7 = 107000;
int speed10_75 = 107500;
int speed11 = 110000;
int speed11_25 = 112500;
int speed11_3 = 113000;
int speed11_5 = 115000;
int speed11_7 = 117000;
int speed11_75 = 117500;
int speed12 = 120000;

void loop() {
  key = Serial.read();
  if (key != -1) {
    switch (key) {
      case '1':                               // test - move fast pos
        moveStepper(testStepper, 30, stage_speed_fast);
        Serial.print("case 1 - fast forward \n");
        break;
      case '2':                               // test - move fast neg
        moveStepper(testStepper, -30, stage_speed_fast);
        Serial.print("case 2 - fast backward \n");
        break;
      case '3':                               // test - move slow pos
        moveStepper(testStepper, 30, stage_speed_slow);
        Serial.print("case 3 - slow forward \n");
        break;
      case '4':                               // test - move slow neg
        moveStepper(testStepper, -30, stage_speed_slow);
        Serial.print("case 4 - slow backward \n");
        break;
      case '5':                               // test - move fast pos
        moveStepper(testStepper, 5, speed1);
        moveStepper(testStepper, 5, speed1_5);
        moveStepper(testStepper, 5, speed2);
        moveStepper(testStepper, 5, speed2_5);
        moveStepper(testStepper, 5, speed3);
        moveStepper(testStepper, 5, speed3_5);
        moveStepper(testStepper, 5, speed4);
        moveStepper(testStepper, 5, speed4_5);
        moveStepper(testStepper, 5, speed5);
        moveStepper(testStepper, 5, speed5_5);
        moveStepper(testStepper, 5, speed6);
        moveStepper(testStepper, 5, speed6_5);
        moveStepper(testStepper, 5, speed7);
        moveStepper(testStepper, 5, speed7_3);
        moveStepper(testStepper, 5, speed7_7);
        moveStepper(testStepper, 5, speed8);
        moveStepper(testStepper, 5, speed8_3);
        moveStepper(testStepper, 5, speed8_7);
        moveStepper(testStepper, 5, speed9);
        moveStepper(testStepper, 5, speed9_3);
        moveStepper(testStepper, 5, speed9_7);
        moveStepper(testStepper, 100, speed10);
        // moveStepper(testStepper, 5, speed10_25);
        // moveStepper(testStepper, 5, speed10_5);
        // moveStepper(testStepper, 5, speed10_75);
        // moveStepper(testStepper, 100, speed11);
        // moveStepper(testStepper, 5, speed11_25);
        // moveStepper(testStepper, 5, speed11_5);
        // moveStepper(testStepper, 5, speed11_75);
        // moveStepper(testStepper, 75, speed12);
        // moveStepper(testStepper, 5, speed11);
        moveStepper(testStepper, 5, speed10);
        moveStepper(testStepper, 5, speed9);
        moveStepper(testStepper, 5, speed8);
        moveStepper(testStepper, 5, speed7);
        moveStepper(testStepper, 5, speed6);
        moveStepper(testStepper, 1, speed5);
        moveStepper(testStepper, 1, speed4);
        moveStepper(testStepper, 1, speed3);
        moveStepper(testStepper, 1, speed2);
        moveStepper(testStepper, 1, speed1);
        Serial.print("case 5 \n");
        break;
      case '6':                               // test - move fast neg
        moveStepper(testStepper, -5, speed1);
        moveStepper(testStepper, -5, speed1_5);
        moveStepper(testStepper, -5, speed2);
        moveStepper(testStepper, -5, speed2_5);
        moveStepper(testStepper, -5, speed3);
        moveStepper(testStepper, -5, speed3_5);
        moveStepper(testStepper, -5, speed4);
        moveStepper(testStepper, -5, speed4_5);
        moveStepper(testStepper, -5, speed5);
        moveStepper(testStepper, -5, speed5_5);
        moveStepper(testStepper, -5, speed6);
        moveStepper(testStepper, -5, speed6_5);
        moveStepper(testStepper, -5, speed7);
        moveStepper(testStepper, -5, speed7_3);
        moveStepper(testStepper, -5, speed7_7);
        moveStepper(testStepper, -5, speed8);
        moveStepper(testStepper, -5, speed8_3);
        moveStepper(testStepper, -5, speed8_7);
        moveStepper(testStepper, -5, speed9);
        moveStepper(testStepper, -5, speed9_3);
        moveStepper(testStepper, -5, speed9_7);
        moveStepper(testStepper, -100, speed10);
        // moveStepper(testStepper, -5, speed10_25);
        // moveStepper(testStepper, -5, speed10_5);
        // moveStepper(testStepper, -5, speed10_75);
        // moveStepper(testStepper, -100, speed11);
        // moveStepper(testStepper, -5, speed11_25);
        // moveStepper(testStepper, -5, speed11_5);
        // moveStepper(testStepper, -5, speed11_75);
        // moveStepper(testStepper, -75, speed12);
        // moveStepper(testStepper, -5, speed11);
        moveStepper(testStepper, -5, speed10);
        moveStepper(testStepper, -5, speed9);
        moveStepper(testStepper, -5, speed8);
        moveStepper(testStepper, -5, speed7);
        moveStepper(testStepper, -5, speed6);
        moveStepper(testStepper, -1, speed5);
        moveStepper(testStepper, -1, speed4);
        moveStepper(testStepper, -1, speed3);
        moveStepper(testStepper, -1, speed2);
        moveStepper(testStepper, -1, speed1);
        Serial.print("case 6 \n");
        break;
      case '0':
        moveStepper(testStepper, 10, speed6);
        moveStepper(testStepper, 100, speed7);
        Serial.print("case 0 \n");
        break;
    }
  }
  delay(10);
}