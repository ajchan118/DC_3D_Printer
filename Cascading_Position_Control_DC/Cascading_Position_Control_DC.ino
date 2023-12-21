#include <ESP32Encoder.h>
#define LIMIT 17
#define BTN 16
#define BIN_1 26
#define BIN_2 25
#define LED_PIN 13
#define POT 14 

ESP32Encoder encoder;

// position values
int thetaPrev = 0;
int theta = 0;
int thetaDes = 0;
int thetaDel = 0;
int thetaMax = 455*19;   // 455
int DTheta = 0;

// velocity values
int omegaSpeedPrev = 0;
int omegaSpeed = 0;
int omegaAccel = 0;
int omegaDes = 0;
int omegaMax = 50;       // 19
int D = 0;
int dir = 1;
int potReading = 0;
float D_scale = 1.0;

// position control values
float KpTheta = 0.25;    // 0.25
float KiTheta = 0.01;    // 0.01
float KdTheta = 2;       // 2
float errorTheta = 0;
float errorThetaSum = 0;

// velocity control values
float KpOmega = 15;      // 15
float KiOmega = 0.01;    // 0.01
float KdOmega = 6;       // 6
float errorOmega = 0;
float errorOmegaSum = 0;

// buttons
volatile bool limitIsPressed = false;
volatile bool buttonIsPressed = false;
int state = 1;

//Setup interrupt variables ----------------------------
volatile int count = 0; // encoder count
volatile bool interruptCounter = false;    // check timer interrupt 1
volatile bool deltaT = false;     // check timer interrupt 2
int totalInterrupts = 0;   // counts the number of triggering of the alarm
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
// hw_timer_t * timer2 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;  // Max = 255
const int NOM_PWM_VOLTAGE = 150;

//Initialization ------------------------------------
// button interrupt
void IRAM_ATTR isr() {
  buttonIsPressed = true;
  timerStart(timer0);  
}

// limit switch interrupt
void IRAM_ATTR isr1() {
  limitIsPressed = true;
}

// void IRAM_ATTR onTime0() {
//   portENTER_CRITICAL_ISR(&timerMux0);
//   interruptCounter = true; // the function to be called when timer interrupt is triggered
//   portEXIT_CRITICAL_ISR(&timerMux0);
// }

// deltaT
void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  count = encoder.getCount( );
  encoder.clearCount ( );
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}

// debounce
void IRAM_ATTR onTime0() { // end timer
  timerStop(timer0);     // stop debounce timer
}

// setup
void setup() {
  // put your setup code here, to run once:
  // onboard setup
  pinMode(POT, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // sets the initial state of LED as turned-off
  pinMode(BTN, INPUT);  // configures the specified pin to behave either as an input or an output
  attachInterrupt(BTN, isr, RISING); // CHANGE -> RISING
  pinMode(LIMIT, INPUT);
  attachInterrupt(LIMIT, isr1, RISING);

  // serial + encoder
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(33, 27); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(BIN_1, ledChannel_1);
  ledcAttachPin(BIN_2, ledChannel_2);

  // initilize timer
  // timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  // timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  // timerAlarmWrite(timer0, 5000000, true); // 5000000 * 1 us = 5 s, autoreload true

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 10000, true); // 10000 * 1 us = 10 ms, autoreload true

  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onTime0, true);
  timerAlarmWrite(timer0, 500000, true);        // 500 ms debounce

  // enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable
  // timerAlarmEnable(timer2);
 
  // stop debounce timer
  timerStop(timer0);
}

void loop() {
    switch (state) {
      case 1:  // off state
        Serial.println("State 1: Off State");
        if (CheckForButtonPress()) {
          // Services
          led_on();
          theta = 0;
          errorThetaSum = 0;
          errorOmegaSum = 0;
          limitIsPressed = false;
          state = 2;
        }
        break;

      case 2:  // init state
        Serial.println("State 2: Initialize (if necessary)");
        runMotorBack();
        if (CheckForButtonPress()) {
          // Services
          led_off();
          limitIsPressed = false;
          state = 3;
        } else if (CheckForLimitPress()) {
          // Services
          led_off();
          limitIsPressed = false;
          state = 3;
        }
        break;

      case 3:  // off state
        Serial.println("State 3: Reset State");
        resetMotor();
        if (CheckForButtonPress()) {
          // Services
          led_on();
          theta = 0;
          errorThetaSum = 0;
          errorOmegaSum = 0;
          limitIsPressed = false;
          state = 4;
        }
        break;

      case 4:  // on state
        Serial.println("State 4: Motor Controlled by Potentiometer");
        runMotor();
        if (CheckForButtonPress()) {
          // Services
          led_off();
          limitIsPressed = false;
          state = 1;
        }
        break;
  }
}


// Other functions
 
// Event Checker Function
bool CheckForButtonPress() {
  // with debouce
  if (timerStarted(timer0)) {
    return false;
  } else {
    if (buttonIsPressed){
      buttonIsPressed = false;
      return true;
    } else {
      return false;
    }
  }
}

// Limit Switch Checker Function
bool CheckForLimitPress() {
  if (limitIsPressed && digitalRead(LIMIT) == 1) {
    limitIsPressed = false;
    return true;
  } else {
    return false;
  }
}

// Main Service Function
void runMotor() {
  if (deltaT) {
      portENTER_CRITICAL(&timerMux1);
      deltaT = false;
      portEXIT_CRITICAL(&timerMux1);

      // Position
      theta += count;
      potReading = analogRead(POT); 
      thetaDes = (map(potReading, 0, 4095, 0, thetaMax));
      thetaDel = thetaPrev - theta;
      thetaPrev = theta;

      errorTheta = thetaDes - theta;
      // Noise Reduction
      if (errorTheta < 0.04*thetaMax && errorTheta > -0.04*thetaMax) {
        errorTheta = 0;
      }
      errorThetaSum += errorTheta;

      // Anti-Windup
      if (errorThetaSum > 300) {
        errorThetaSum = 300;
      } else if (errorThetaSum < -300) {
        errorThetaSum = -300;
      }

      // Position Value (PID)
      DTheta = ((KpTheta * errorTheta) + (KiTheta * errorThetaSum) + (KdTheta * thetaDel));

      // Velocity
      omegaSpeed = count;
      omegaAccel = omegaSpeed - omegaSpeedPrev;
      omegaSpeedPrev = omegaSpeed;
      omegaDes = DTheta;

      errorOmega = omegaDes - omegaSpeed;
      errorOmegaSum += errorOmega;
    
      // Anti-Windup
      if (errorOmegaSum > 400) {
        errorOmegaSum = 400;
      } else if (errorOmegaSum < -400) {
        errorOmegaSum = -400;
      }

      // Velocity Value (PID)
      D = ((KpOmega * errorOmega) + (KiOmega * errorOmegaSum) + (KdOmega * omegaAccel));

      //Ensure that you don't go past the maximum possible command
      if (D > MAX_PWM_VOLTAGE) {
          D = MAX_PWM_VOLTAGE;
      }
      else if (D < -MAX_PWM_VOLTAGE) {
          D = -MAX_PWM_VOLTAGE;
      }

      //Map the D value to motor directionality
      //FLIP ENCODER PINS SO SPEED AND D HAVE SAME SIGN
      if (D > 0) {
          ledcWrite(ledChannel_1, LOW);
          ledcWrite(ledChannel_2, D);
      }
      else if (D < 0) {
          ledcWrite(ledChannel_1, -D);
          ledcWrite(ledChannel_2, LOW);
      }
      else {
          ledcWrite(ledChannel_1, LOW);
          ledcWrite(ledChannel_2, LOW);
      }

      plotControlData();
  }
}

// Initialization Setup
void runMotorBack() {
  if (deltaT) {
      portENTER_CRITICAL(&timerMux1);
      deltaT = false;
      portEXIT_CRITICAL(&timerMux1);

      // Position
      theta += count;
      potReading = analogRead(POT); 
      thetaDes = (map(potReading, 0, 4095, 0, -thetaMax)); // flip dir
      thetaDel = thetaPrev - theta;
      thetaPrev = theta;

      errorTheta = thetaDes - theta;
      // Noise Reduction
      if (errorTheta < 0.04*thetaMax && errorTheta > -0.04*thetaMax) {
        errorTheta = 0;
      }
      errorThetaSum += errorTheta;

      // Anti-Windup
      if (errorThetaSum > 300) {
        errorThetaSum = 300;
      } else if (errorThetaSum < -300) {
        errorThetaSum = -300;
      }

      // Position Value (PID)
      DTheta = ((KpTheta * errorTheta) + (KiTheta * errorThetaSum) + (KdTheta * thetaDel));

      // Velocity
      omegaSpeed = count;
      omegaAccel = omegaSpeed - omegaSpeedPrev;
      omegaSpeedPrev = omegaSpeed;
      omegaDes = DTheta;

      errorOmega = omegaDes - omegaSpeed;
      errorOmegaSum += errorOmega;
    
      // Anti-Windup
      if (errorOmegaSum > 400) {
        errorOmegaSum = 400;
      } else if (errorOmegaSum < -400) {
        errorOmegaSum = -400;
      }

      // Velocity Value (PID)
      D = ((KpOmega * errorOmega) + (KiOmega * errorOmegaSum) + (KdOmega * omegaAccel));

      //Ensure that you don't go past the maximum possible command
      if (D > MAX_PWM_VOLTAGE) {
          D = MAX_PWM_VOLTAGE;
      }
      else if (D < -MAX_PWM_VOLTAGE) {
          D = -MAX_PWM_VOLTAGE;
      }

      //Map the D value to motor directionality
      //FLIP ENCODER PINS SO SPEED AND D HAVE SAME SIGN
      if (D > 0) {
          ledcWrite(ledChannel_1, LOW);
          ledcWrite(ledChannel_2, D);
      }
      else if (D < 0) {
          ledcWrite(ledChannel_1, -D);
          ledcWrite(ledChannel_2, LOW);
      }
      else {
          ledcWrite(ledChannel_1, LOW);
          ledcWrite(ledChannel_2, LOW);
      }

      plotControlData();
  }
}

// Reset Motor to 0
void resetMotor() {
  if (deltaT) {
      portENTER_CRITICAL(&timerMux1);
      deltaT = false;
      portEXIT_CRITICAL(&timerMux1);
      
      // reset values
      thetaPrev = 0;
      theta = 0;
      thetaDes = 0;
      thetaDel = 0;
      thetaMax = 455*20;   // 455
      DTheta = 0;

      // velocity values
      omegaSpeedPrev = 0;
      omegaSpeed = 0;
      omegaAccel = 0;
      omegaDes = 0;
      omegaMax = 50;       // 19
      potReading = 0;

      D = 0;

      //Ensure that you don't go past the maximum possible command
      if (D > MAX_PWM_VOLTAGE) {
          D = MAX_PWM_VOLTAGE;
      }
      else if (D < -MAX_PWM_VOLTAGE) {
          D = -MAX_PWM_VOLTAGE;
      }

      //Map the D value to motor directionality
      //FLIP ENCODER PINS SO SPEED AND D HAVE SAME SIGN
      if (D > 0) {
          ledcWrite(ledChannel_1, LOW);
          ledcWrite(ledChannel_2, D);
      }
      else if (D < 0) {
          ledcWrite(ledChannel_1, -D);
          ledcWrite(ledChannel_2, LOW);
      }
      else {
          ledcWrite(ledChannel_1, LOW);
          ledcWrite(ledChannel_2, LOW);
      }

      plotControlData();
  }
}

// LED on
void led_on() {
  digitalWrite(LED_PIN, HIGH);
}

// LED off
void led_off() {
  digitalWrite(LED_PIN, LOW);
}

void plotControlData() {
  // Serial.print("Test Value:");
  // Serial.print(digitalRead(LIMIT));
  // Serial.print(" ");
  // Serial.println();
  // Serial.print("Test Value:");
  // Serial.print(limitIsPressed);
  // Serial.print(" ");
  // Serial.println();
  // Serial.print("errorOmega/20:");
  // Serial.print(errorOmega/20);
  // Serial.print(" ");
  // Serial.print("errorOmegaSum/20:");
  // Serial.print(errorOmegaSum/20);
  // Serial.print(" ");
  // Serial.print("Speed:");
  // Serial.print(omegaSpeed);
  // Serial.print(" ");
  // Serial.print("Desired_Speed:");
  // Serial.print(omegaDes);
  // Serial.print(" ");
  // Serial.print("errorTheta/20:");
  // Serial.print(errorTheta/20);
  // Serial.print(" ");
  // Serial.print("errorThetaSum/20:");
  // Serial.print(errorThetaSum/20);
  // Serial.print(" ");
  // Serial.print("Position:");
  // Serial.print(theta);
  // Serial.print(" ");
  // Serial.print("Desired_Position:");
  // Serial.print(thetaDes);
  // Serial.print(" ");
  // Serial.print("DTheta:");
  // Serial.print(DTheta);
  // Serial.print(" ");
  // Serial.print("PWM_Duty/10:");
  // Serial.println(D/10);  //PWM is scaled by 1/10 to get more intelligible graph
}
