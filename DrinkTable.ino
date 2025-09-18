#include "arduino_secrets.h"
#include "thingProperties.h"

#include <ESP32Servo.h> // ESP32Servo - Version: Latest
#include <SparkFun_APDS9960.h> // SparkFun APDS9960 RGB and Gesture Sensor - Version: Latest
#include <Wire.h> // For IC2 Communication

// Declarations
enum RGBColor {
  Red, Green, Blue, Yellow, Cyan, Purple, White, Black
};

const int red = 13;
const int green = 12;
const int blue = 14;
const int pump1 = 26; // Apple Brandy
const int pump2 = 25; // Cran-Grape Juice
const int pump3 = 33; // UV Blue
const int pump4 = 32; // Lemonade

Servo servo;
const int servoPin = 17;

const int elevatorMotorPin1 = 4;
const int elevatorMotorPin2 = 2;
const int elevatorMotorPWM = 16;
const int elevatorMotorChannel = 2;
const int elevatorMotorFrequency = 5000;
const int elevatorMotorResolution = 8;

const int topLimit = 27;
const int topLimitPower = 15;
const int bottomLimit = 23;
const int bottomLimitPower = 5;

SparkFun_APDS9960 prox = SparkFun_APDS9960();
const int proxPower = 19;
const int proxGround = 18;

boolean firstRun;

boolean errorState = false;
int errorBlinkCode = 0;

boolean recentPour = false;

double shotPumpTime = 6.0;
double mixerPumpTime = 8.5;

/// @brief Function called once on boot of program
void setup() {
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(pump3, OUTPUT);
  pinMode(pump4, OUTPUT);

  pinMode(elevatorMotorPin1, OUTPUT);
  pinMode(elevatorMotorPin2, OUTPUT);

  pinMode(topLimit, INPUT_PULLDOWN);
  pinMode(topLimitPower, OUTPUT);
  pinMode(bottomLimit, INPUT_PULLDOWN);
  pinMode(bottomLimitPower, OUTPUT);

  pinMode(proxPower, OUTPUT);
  pinMode(proxGround, OUTPUT);

  Serial.begin(9600); // Initialize serial and wait for port to open:
  delay(1500); // Delay briefly to allow a Serial Monitor to connect without blocking if none found
  initProperties(); // Defined in thingProperties.h
  ArduinoCloud.begin(ArduinoIoTPreferredConnection); // Connect to Arduino IoT Cloud

  // Gets network/IoT Cloud connection status and errors (0 = errors only, 4 = maximum detail).
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  servo.attach(servoPin);
  servoSetPosition(0.0);
  
  ledcSetup(elevatorMotorChannel, elevatorMotorFrequency, elevatorMotorResolution);
  ledcAttachPin(elevatorMotorPWM, elevatorMotorChannel);
  digitalWrite(elevatorMotorPin1, HIGH);
  digitalWrite(elevatorMotorPin2, HIGH);
  ledcWrite(elevatorMotorChannel, 0);
  
  digitalWrite(topLimitPower, HIGH);
  digitalWrite(bottomLimitPower, HIGH);
  
  digitalWrite(proxPower, HIGH);
  digitalWrite(proxGround, LOW);
  delay(500); // Delay for proximity sensor to power on

  prox.init(); // Initialize APDS-9960 (configure I2C and initial values)
  prox.setProximityGain(0); // Adjust the Proximity sensor gain. 0: 1x, 1: 2x, 2: 4x, 3: 8x
  prox.enableProximitySensor(false); // Start running the prox-9960 proximity sensor (no interrupts)
}

/// @brief Function called repeatedly during execution of program
void loop() {
  // Check if connected to Arduino IoT Cloud
  if (!ArduinoCloud.connected()) {
    Serial.println("Connecting to Arduino IoT Cloud...");
    firstRun = true;
    connectionBlink(false);
  }

  // Check that resets recentPour
  if (recentPour && !isGlassPresent()) {
    recentPour = false;
  }

  ArduinoCloud.update();
}

/// @brief Function called when a change in Drink Selection is detected
void onDrinkSelectionChange()  {
  // Check if the change is due to connecting to the internet
  if (firstRun) {
    firstRun = false;
    connectionBlink(true);
    return;
  }
  
  // Retrieve the table code and reset it back to 0
  int tableCode = drinkSelection.getBrightness();
  drinkSelection.setBrightness(0);
  Serial.println("-------");
  Serial.print("Table Code: ");
  Serial.println(tableCode);

  // Clear Error State of Table
  if (tableCode == 99) {
    errorState = false;
    errorBlinkCode = 0;
    Serial.println("Error State Cleared");
    errorBlink(1, false);
    return;
  }

  // Checks if Table is in Error State
  if (errorState) {
    Serial.println("Error: Table in Error State, Please Correct and Clear the Error");
    errorBlink(errorBlinkCode, true);
    return;
  }

  // Homes Table to Top Limit
  if (tableCode == 97) {
    Serial.println("Homing to Top Limit...");
    if (!elevatorMove(true)) { // Move Up until Top Limit
      Serial.println("Error: Elevator Movement Timed Out");
      errorBlink(2, true);
    }

    return;
  }

  // Homes Table to Bottom Limit
  if (tableCode == 98) {
    Serial.println("Homing to Bottom Limit...");
    if (!elevatorMove(false)) { // Move Up until Top Limit
      Serial.println("Error: Elevator Movement Timed Out");
      errorBlink(2, true);
    }

    return;
  }

  // Checks if the Glass has been Replaced
  if (recentPour) {
    Serial.println("Error: Table has Detected a Recent Pour, Replace the Glass with a New One");
    errorBlink(4, false);
    return;
  }

  // Checks if a Glass is Present
  if (!isGlassPresent()) {
    Serial.println("Error: Glass is not Present");
    errorBlink(2, false);
    return;
  }

  // Detects a Non-Valid Drink is Selected
  double pump1Time, pump2Time, pump3Time, pump4Time = 0.0;
  if (!getPumpTimes(tableCode, pump1Time, pump2Time, pump3Time, pump4Time)) {
    Serial.println("Error: Invalid Drink Selection");
    errorBlink(3, false);
    return;
  }
  Serial.println("-Pump Times-");
  Serial.println(pump1Time);
  Serial.println(pump2Time);
  Serial.println(pump3Time);
  Serial.println(pump4Time);

  // Drink Routine Starts Here
  if (!elevatorMove(false)) { // Move Elevator Down until Bottom Limit
    Serial.println("Error: Elevator Movement Timed Out");
    errorBlink(2, true);
    return;
  }

  // Move Servo to Pouring Position
  servoSetPosition(1.0);
  delay(500);

  // Pump Liquid
  if (!pumpLiquid(pump1Time, pump2Time, pump3Time, pump4Time)) {
    Serial.println("Error: Pump Liquid Failed");
    servoSetPosition(0.0);
    errorBlink(3, true);
    return;
  }
  delay(1500);
  
  // Move Servo to Stored Position
  servoSetPosition(0.0);
  delay(500);

  // Move Elevator Up until Top Limit
  if (!elevatorMove(true)) {
    Serial.println("Error: Elevator Movement Timed Out");
    errorBlink(2, true);
    return;
  }

  setColor(Black);
}

/// @brief Function that determines if a glass is present
/// @return True if a glass is present, false if not present or if there is an error reading the
/// value
boolean isGlassPresent() {
  uint8_t proxData;
  if (!prox.readProximity(proxData)) {
    Serial.println("Error: Could not Read Proximity Value");
    return false;
  } else {
    Serial.print("Proximity Value: ");
    Serial.println(proxData);
    return proxData > 230;
  }
}

/// @brief Function that sets the servo's position given a percent of rotation. To reduce noise from
/// constant minor corrections, the target position is adjusted by ±5 units after a short delay.
/// @param percent A value between 0.0 and 1.0 with 0.0 being stored position and 1.0 being pouring
/// position
void servoSetPosition(double percent) {
  if (percent < 0.0) {
    percent = 0.0;
  } else if (percent > 1.0) {
    percent = 1.0;
  }

  // Map the values 0.0-1.0 to 5-110
  int value = 105 * percent + 5; 
  int prevValue = servo.read();
  servo.write(value);

  // Determine direction of travel
  if (abs(value - prevValue) > 5) {
    int adjustment = (value < prevValue) ? 5 : -5;
    
    // Let the servo move to the requested set position
    delay(1000);
    servo.write(value + adjustment);
  }
}

/// @brief Function that moves the elevator until the correct limit switch is pressed with a timeout
/// @param up Whether the desired movement of the elevator is up or down
/// @return True if the movement did not timeout, else false
boolean elevatorMove(boolean up) {
  unsigned long startTime = millis();
  const unsigned long timeout = 8000;

  // Select direction and limit switch depending on up
  double direction = up ? 1.0 : -1.0;
  int limitPin = up ? topLimit : bottomLimit;

  while (!digitalRead(limitPin) && (millis() - startTime) < timeout) {
    motorSetSpeed(elevatorMotorChannel, elevatorMotorPin1, elevatorMotorPin2, direction);
  }
  motorSetSpeed(elevatorMotorChannel, elevatorMotorPin1, elevatorMotorPin2, 0.0);

  return (millis() - startTime) < timeout;
}

/// @brief Function that sets a motor's speed and direction
/// @param motorChannel The channel that the motor controller is running on
/// @param pin1 One of the pins that the motor controller is plugged into
/// @param pin2 One of the pins that the motor controller is plugged into
/// @param percentage A value between -1.0 and 1.0 that represents the percentage and direction
/// for the motor to run
void motorSetSpeed(int motorChannel, int pin1, int pin2, double percentage) {
  if (percentage < -1.0) {
    percentage = -1.0;
  } else if (percentage > 1.0) {
    percentage = 1.0;
  }

  int value = round(abs(255 * percentage));
  if (percentage > 0.0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    ledcWrite(motorChannel, value); 
  } else if (percentage < 0.0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    ledcWrite(motorChannel, value);
  } else {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, HIGH);
    ledcWrite(motorChannel, value);
  }
}

/// @brief Function that pumps liquid given the desired duration
/// @param pump1Time The desired pump time in seconds
/// @param pump2Time The desired pump time in seconds
/// @param pump3Time The desired pump time in seconds
/// @param pump4Time The desired pump time in seconds
/// @return True if the glass stays present for the duration of the function, else false
boolean pumpLiquid(double pump1Time, double pump2Time, double pump3Time, double pump4Time) {
  unsigned long pump1Dur = (unsigned long)(pump1Time * 1000.0);
  unsigned long pump2Dur = (unsigned long)(pump2Time * 1000.0);
  unsigned long pump3Dur = (unsigned long)(pump3Time * 1000.0);
  unsigned long pump4Dur = (unsigned long)(pump4Time * 1000.0);

  unsigned long maxPumpTime = max(max(pump1Dur, pump2Dur), max(pump3Dur, pump4Dur));
  unsigned long elapsedTime = 0;
  unsigned long startTime = millis();

  while (maxPumpTime > elapsedTime && isGlassPresent()) {
    elapsedTime = millis() - startTime;
    digitalWrite(pump1, elapsedTime < pump1Dur ? HIGH : LOW);
    digitalWrite(pump2, elapsedTime < pump2Dur ? HIGH : LOW);
    digitalWrite(pump3, elapsedTime < pump3Dur ? HIGH : LOW);
    digitalWrite(pump4, elapsedTime < pump4Dur ? HIGH : LOW);

    recentPour = true;
  }

  digitalWrite(pump1, LOW);
  digitalWrite(pump2, LOW);
  digitalWrite(pump3, LOW);
  digitalWrite(pump4, LOW);

  return isGlassPresent();
}

/// @brief Function that sets the blink code and state of table and blinks LEDs
/// @param times Number of times for the LEDs to blink
/// @param state True if fatal error, false if warning error 
void errorBlink(int times, boolean state) {
  errorBlinkCode = times;
  errorState = state;
  RGBColor color;

  if (state) {
    color = Red;
  } else {
    color = White;
  }

  for (int i = 0; i < times; i++) {
    setColor(color);
    delay(250);
    setColor(Black);
    delay(250);
  }
}

/// @brief Function that blinks the LEDs to show the Arduino IoT Cloud connection state
/// @param connected True if the table is connected, false if not connected
void connectionBlink(boolean connected) {
  RGBColor color;
  int delayTime;
  
  if (connected) {
    color = Green;
    delayTime = 250;
  } else {
    color = Blue;
    delayTime = 1000;
  }
  
  setColor(color);
  delay(delayTime);
  setColor(Black);
  delay(delayTime);
}

/// @brief Function that sets the color of the LEDs
/// @param color The desired color for the LEDs to be set to
void setColor(RGBColor color) {
  switch (color) {
    case Red:
      digitalWrite(red, HIGH);
      digitalWrite(green, LOW);
      digitalWrite(blue, LOW);
      break;
    case Green:
      digitalWrite(red, LOW);
      digitalWrite(green, HIGH);
      digitalWrite(blue, LOW);
      break;
    case Blue:
      digitalWrite(red, LOW);
      digitalWrite(green, LOW);
      digitalWrite(blue, HIGH);
      break;
    case Yellow:
      digitalWrite(red, HIGH);
      digitalWrite(green, HIGH);
      digitalWrite(blue, LOW);
      break;
    case Cyan:
      digitalWrite(red, LOW);
      digitalWrite(green, HIGH);
      digitalWrite(blue, HIGH);
      break;
    case Purple:
      digitalWrite(red, HIGH);
      digitalWrite(green, LOW);
      digitalWrite(blue, HIGH);
      break;
    case White:
      digitalWrite(red, HIGH);
      digitalWrite(green, HIGH);
      digitalWrite(blue, HIGH);
      break;
    case Black:
      digitalWrite(red, LOW);
      digitalWrite(green, LOW);
      digitalWrite(blue, LOW);
      break;
    default:
      digitalWrite(red, LOW);
      digitalWrite(green, LOW);
      digitalWrite(blue, LOW);
      break;
  }
}

/// @brief Function that given the drink code will set the pump times and LED color
/// @param drinkCode The numeric code of the drink
/// @param pump1Time The time to be set for pump1
/// @param pump2Time The time to be set for pump2
/// @param pump3Time The time to be set for pump3
/// @param pump4Time The time to be set for pump4
/// @return True if a valid drink's pump times were calculated, else false 
boolean getPumpTimes(int drinkCode, double &pump1Time, double &pump2Time, double &pump3Time,
                     double &pump4Time) {
  boolean validDrink = false;

  switch (drinkCode) {
    case 1: // Apple Brandy Straight
      pump1Time = shotPumpTime; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Green);
      validDrink = true;
      break;
    case 2: // Apple Brandy Straight Tall
      pump1Time = shotPumpTime * 2; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Green);
      validDrink = true;
      break;
    case 3: // UV Blue Straight
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = shotPumpTime; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Blue);
      validDrink = true;
      break;
    case 4: // UV Blue Straight Tall
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = shotPumpTime * 2; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Blue);
      validDrink = true;
      break;
    case 5: // UV Blue Cran-Grape
      pump1Time = 0.0; // Apple Brandy
      pump2Time = mixerPumpTime; // Cran-Grape
      pump3Time = shotPumpTime; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Purple);
      validDrink = true;
      break;
    case 6: // UV Blue Cran-Grape Tall
      pump1Time = 0.0; // Apple Brandy
      pump2Time = mixerPumpTime; // Cran-Grape
      pump3Time = shotPumpTime * 2; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Purple);
      validDrink = true;
      break;
    case 7: // UV Blue Lemonade
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = shotPumpTime; // UV Blue
      pump4Time = mixerPumpTime; // Lemonade
      setColor(Cyan);
      validDrink = true;
      break;
    case 8: // UV Blue Lemonade Tall
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = shotPumpTime; // UV Blue
      pump4Time = mixerPumpTime * 2; // Lemonade
      setColor(Cyan);
      validDrink = true;
      break;
    case 9: // Cran-Grape
      pump1Time = 0.0; // Apple Brandy
      pump2Time = mixerPumpTime; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Red);
      validDrink = true;
      break;
    case 10: // Cran-Grape Tall
      pump1Time = 0.0; // Apple Brandy
      pump2Time = mixerPumpTime * 2; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Red);
      validDrink = true;
      break;
    case 11: // Lemonade
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = mixerPumpTime; // Lemonade
      setColor(Yellow);
      validDrink = true;
      break;
    case 12: // Lemonade Tall
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = mixerPumpTime * 2; // Lemonade
      setColor(Yellow);
      validDrink = true;
      break;
    default:
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Black);
      validDrink = false;
      break;
  }

  return validDrink;
}
