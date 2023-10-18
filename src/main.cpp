#include <Adafruit_INA260.h>
#include <Servo.h>

Adafruit_INA260 ina260 = Adafruit_INA260(); // Power sensor object
Servo linearActuator;

// Operation states -- add 1 in for power regulation
enum STATES {
  initState,
  normalOp,
  powerReg,
  eStop,
  loadDisconnect,
};

// Pins for relay switching
const int relaySetSignal = 12;
const int relayResetSignal = 13;

// Pin for emergency stop button
const int ESTOP = 8;

// Power Regulation global vars
float power_target = 50000; // Constant to be found in wind tunnel.
float power_threshold = 0.2; // mW. Tweak this to affect sensitivity

// Linear Actuator global vars
const int MAX_POS = 140;
const int MIN_POS = 50;
const int braking_pos = 140;
const int cut_in_pos = 50; // Set to some position for desired cut-in value
const int normal_op_pos = 90; // Set to desired normal pitch (i.e. pitch that maxes power at 11 m/s)
int power_reg_pos = normal_op_pos; // position that will change during power regulation
int direction = 1; // Are we currently increasing (pos) or decreasing blade angle?
int delta = 3;

STATES state;
int relayState; // 0 for closed 1 for open

void closeSwitch();
void openSwitch();

void initialize();
void normalOperation();
void powerRegulation();
void emergencyStop();
void loadDisconnection();
void loadReconnection();

void setup() {
  Serial.begin(9600);

  while (!Serial) {delay(10);}

  ina260.begin();

  linearActuator.attach(9);  // connect linear actuator RC control to pin 9
  
  pinMode(relaySetSignal, OUTPUT);
  pinMode(relayResetSignal, OUTPUT);
  pinMode(ESTOP, INPUT);

  state = initState;

  delay(10);
}

void loop() {
  // Each loop, we check the operating state and execute the appropriate code
  switch (state) {
    case initState:
      initialize();
      break;
    case normalOp:
      normalOperation();
      break;
    case powerReg:
      powerRegulation();
      break;
    case eStop:
      emergencyStop();
      break;
    case loadDisconnect:
      loadDisconnection();
      break;
  }
}

// Helper functions for switching relay
void closeSwitch() {
  digitalWrite(relaySetSignal, HIGH);
  delay(5);
  digitalWrite(relaySetSignal, LOW);
  relayState = 0;
}

void openSwitch() {        
  digitalWrite(relayResetSignal, HIGH);
  delay(5);
  digitalWrite(relayResetSignal, LOW);
  relayState = 1;
}

// Function that should run every time the Arduino boots
// This function makes sure that as the turbine starts spinning,
// the Arduino always has enough power
void initialize() {
  
  // TODO: uncomment this line!
  linearActuator.write(cut_in_pos);
  
  openSwitch();
  float voltage = ina260.readBusVoltage();
  
  while (voltage < 6000)
  { 
    delay(10);
    voltage = ina260.readBusVoltage();
    //Serial.println(voltage);
  }
  
  closeSwitch();
  Serial.print('O');
  state = normalOp;
}

// Normal operation: wind speeds 5-11 m/s and no shutdown conditions
// TODO: Put something in here for power regulation sensing
void normalOperation() {
  //TODO: uncomment this line!!
  linearActuator.write(normal_op_pos);
  
  // Power and voltage readings
  float power = ina260.readPower(); //power in mW
  float voltage = ina260.readBusVoltage(); //voltage in mV

  // Emergency stop button
  if (digitalRead(ESTOP) == LOW) {
    // Begin shutdown operation
    state = eStop;
    Serial.print('E');
    openSwitch();
    linearActuator.write(braking_pos);
  }

  // Load disconnect
  if (Serial.available()) {
    char c;
    c = Serial.read();
    if (c == 'D') {
      state = loadDisconnect;
      linearActuator.write(braking_pos);
      delay(1000); // Test to determine best value to put here
      openSwitch();
    }
  }

  // Power regulation
  if ((power > power_target) && (power - power_target > power_threshold) )
  {
    state = powerReg;
  }
}

// TODO: When to break back into normalOp (sufficiently low voltage reading?)
void powerRegulation() {
  
  // We have a set desired regulated power
  // Read the current power
  float power = ina260.readPower(); //power in mW
  // Calculate the difference
  float power_diff = power - power_target;
  
  // If the current power is within threshold of target power, do nothing
  // If not, we perturb in a direction, observe the change
  if (abs(power_diff) > power_threshold) {
    // Perturb and observe
    if ((power_reg_pos >= MIN_POS+delta && direction == -1) || (power_reg_pos <= MAX_POS-delta && direction == 1)) {
      power_reg_pos = power_reg_pos + direction*delta; 
    }
    linearActuator.write(power_reg_pos); // update position
    delay(1000);

    float power_new = ina260.readPower();
    float power_diff_new = power_new - power_target;
    if (abs(power_diff_new) > abs(power_diff)) {
      direction = -direction;
    }
  }

  // we are back at original pitch
  if (power_reg_pos == normal_op_pos) {
    state = normalOp;
  }

}
// Emergency stop: Emergency stop button has been pressed, we wait for it to release
void emergencyStop() {
  if (digitalRead(ESTOP) == HIGH) {
    Serial.print('N');
    initialize();
  }
}

// Load disconnection: load has been disconnected from turbine
// The load side handles most of the logic for this (and turbine side is off for most of this)
// so turbine side Arduino doesn't need to do anything here
void loadDisconnection() {
  return;
}
