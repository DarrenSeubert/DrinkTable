#include "arduino_secrets.h"
#include <SparkFun_APDS9960.h> // SparkFun APDS9960 RGB and Gesture Sensor - Version: Latest
#include <Wire.h> // Enable IC2 Communication
#include <ESP32Servo.h> // ESP32Servo - Version: Latest
#include "thingProperties.h"

enum RGBColor {
  Red, Green, Blue, Yellow, Cyan, Purple, White, Black
};

int red = 13;
int green = 12;
int blue = 14;
int pump1 = 26; // Apple Brandy
int pump2 = 25; // Cran-Grape Juice
int pump3 = 33; // UV Blue
int pump4 = 32; // Lemonade

SparkFun_APDS9960 prox = SparkFun_APDS9960();
int proxPower = 19;
int proxGround = 18;

int topLimit = 34;
int topLimitPower = 23;
int bottomLimit = 35;
int bottomLimitPower = 5;

int elevatorMotorPWM = 16;
int elevatorMotorPin1 = 4;
int elevatorMotorPin2 = 2;

Servo servo;
int servoPin = 17;

boolean errorState = false;
int errorBlinkCode = 0;

boolean recentPour = false;
unsigned long lastPourTime = 0;

double shotPumpTime = 6.0;
double mixerPumpTime = 8.5;

/**

*/
void setup() {
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(pump3, OUTPUT);
  pinMode(pump4, OUTPUT);

  pinMode(proxPower, OUTPUT);
  pinMode(proxGround, OUTPUT);

  pinMode(topLimit, INPUT);
  pinMode(topLimitPower, OUTPUT);
  pinMode(bottomLimit, INPUT);
  pinMode(bottomLimitPower, OUTPUT);

  pinMode(elevatorMotorPWM, OUTPUT);
  pinMode(elevatorMotorPin1, OUTPUT);
  pinMode(elevatorMotorPin2, OUTPUT);

  servo.attach(servoPin);

  Serial.begin(9600); // Initialize serial and wait for port to open:
  delay(1500); // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  initProperties(); // Defined in thingProperties.h
  ArduinoCloud.begin(ArduinoIoTPreferredConnection); // Connect to Arduino IoT Cloud

  /*
     The following function allows you to obtain more information related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get. The default is 0 (only errors), maximum is 4.
  */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  digitalWrite(topLimitPower, HIGH);
  digitalWrite(bottomLimitPower, HIGH);
  digitalWrite(proxPower, HIGH);
  digitalWrite(proxGround, LOW);
  delay(500); // Delay for proximity sensor to power on

  prox.init(); // Initialize APDS-9960 (configure I2C and initial values)
  prox.setProximityGain(1); // Adjust the Proximity sensor gain. 0: 1x, 1: 2x, 2: 4x, 3: 8x
  prox.enableProximitySensor(false); // Start running the prox-9960 proximity sensor (no interrupts)
}

/**

*/
void loop() {
  // Check that resets recentPour
  if (recentPour && (millis() - lastPourTime > 3000 || millis() - lastPourTime < 0) && !isGlassPresent()) {
    recentPour = false;
  }

  ArduinoCloud.update();
}

/**

*/
void onDrinkSelectionChange()  {
  if (errorState) {
    Serial.println("Error: Table in Error State, Please Correct Error and Power Cycle Table");
    drinkSelection.setBrightness(0);
    errorBlink(errorBlinkCode, true);
    return;
  }

  if (recentPour) {
    Serial.println("Error: Table has Detected a Recent Pour, Replace the Glass with a New One");
    drinkSelection.setBrightness(0);
    errorBlink(2, false);
    return;
  }

  if (!isGlassPresent()) { // Glass is not Present
    Serial.println("Error: Glass is not Present");
    drinkSelection.setBrightness(0);
    errorBlink(3, false);
    return;
  }

  int drinkCode = drinkSelection.getBrightness();
  drinkSelection.setBrightness(0);
  double pump1Time, pump2Time, pump3Time, pump4Time = 0.0;
  if (!getPumpTimes(drinkCode, pump1Time, pump2Time, pump3Time, pump4Time)) { // Detects a Non-Valid Drink is Selected
    Serial.println("Error: Invalid Drink Selection");
    errorBlink(2, true);
    return;
  }

  Serial.println(pump1Time);
  Serial.println(pump2Time);
  Serial.println(pump3Time);
  Serial.println(pump4Time);
  Serial.println("-------");

  // Drink Routine Starts Here:
  if (!elevatorMove(false)) { // Move Down until Bottom Limit
    Serial.println("Error: Elevator Movement Timed Out");
    errorBlink(3, true);
    return;
  }

  servoSetPosition(1.0);
  delay(2000);

  if (!pumpLiquid(pump1Time, pump2Time, pump3Time, pump4Time)) { // Pump Liquid
    Serial.println("Error: Pump Liquid Failed");
    errorBlink(4, true);
    return;
  }

  delay(1000);
  servoSetPosition(1.0);
  delay(2000);

  if (!elevatorMove(true)) { // Move Up until Top Limit
    Serial.println("Error: Elevator Movement Timed Out");
    errorBlink(3, true);
    return;
  }

  setColor(Black);
  recentPour = true;
  lastPourTime = millis();
}

/**
   Returns True if a glass is present, else false
*/
boolean isGlassPresent() {
  uint8_t proxData;
  if (!prox.readProximity(proxData)) {
    Serial.println("Error: Could not Read Proximity Value");
    return false;
  } else {
    return proxData > 240;
  }
}

/**

*/
void servoSetPosition(double percent) {
  if (percent < 0.0) {
    percent = 0.0;
  } else if (percent > 1.0) {
    percent = 1.0;
  }

  servo.write(180 * percent);
}

/**
   Returns True if timeout or wrap around did not occur
*/
boolean elevatorMove(boolean up) {
  unsigned long startTime = millis();
  if (up) { // Move Up
    while (!digitalRead(topLimit) && (millis() - startTime) < 3000 && (millis() - startTime) > 0) {
      motorSetSpeed(elevatorMotorPWM, elevatorMotorPin1, elevatorMotorPin2, 1.0);
    }
  } else { // Move Down
    while (!digitalRead(bottomLimit) && (millis() - startTime) < 3000 && (millis() - startTime) > 0) {
      motorSetSpeed(elevatorMotorPWM, elevatorMotorPin1, elevatorMotorPin2, -1.0);
    }
  }
  motorSetSpeed(elevatorMotorPWM, elevatorMotorPin1, elevatorMotorPin2, 0.0);

  return ((millis() - startTime) < 3000 && (millis() - startTime) > 0);
}

/**

*/
void motorSetSpeed(int pwmPin, int pin1, int pin2, double speed) {
  if (speed < -1.0) {
    speed = -1.0;
  } else if (speed > 1.0) {
    speed = 1.0;
  }

  int value = round(abs(255 * speed));
  if (speed >= 0) {
    analogWrite(pwmPin, value);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else {
    analogWrite(pwmPin, value);
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
}

/**
   Returns false if pump fails (glass is not present at the begining or end of pumping)
*/
boolean pumpLiquid(double pump1Time, double pump2Time, double pump3Time, double pump4Time) {
  if (!isGlassPresent()) {
    return false;
  }

  double maxPumpTime = max(max(pump1Time, pump2Time), max(pump3Time, pump4Time));
  double elapsedTime = 0.0;
  double startTime = millis() / 1000.0;

  while (maxPumpTime > elapsedTime) {
    elapsedTime = (millis() / 1000.0) - startTime;
    if (elapsedTime < pump1Time) {
      digitalWrite(pump1, HIGH);
    } else {
      digitalWrite(pump1, LOW);
    }

    if (elapsedTime < pump2Time) {
      digitalWrite(pump2, HIGH);
    } else {
      digitalWrite(pump2, LOW);
    }

    if (elapsedTime < pump3Time) {
      digitalWrite(pump3, HIGH);
    } else {
      digitalWrite(pump3, LOW);
    }

    if (elapsedTime < pump4Time) {
      digitalWrite(pump4, HIGH);
    } else {
      digitalWrite(pump4, LOW);
    }
  }

  return isGlassPresent();
}

/**

*/
void errorBlink(int times, boolean state) {
  errorBlinkCode = times;
  errorState = state;
  RGBColor color = White;

  if (errorState) {
    color = Red;
  }

  for (int i = 0; i < times; i++) {
    setColor(color);
    delay(250);
    setColor(Black);
    delay(250);
  }
}

/**

*/
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

/**
   Given drink code, set pump times and color

   pump1 = Apple Brandy
   pump2 = Cran-Grape Juice
   pump3 = UV Blue
   pump4 = Lemonade
*/
boolean getPumpTimes(int drinkCode, double &pump1Time, double &pump2Time, double &pump3Time, double &pump4Time) {
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
    case 2:  // Apple Brandy Straight Tall
      pump1Time = shotPumpTime * 2; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Green);
      validDrink = true;
      break;
    case 3:  // UV Blue Straight
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = shotPumpTime; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Blue);
      validDrink = true;
      break;
    case 4:  // UV Blue Straight Tall
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = shotPumpTime * 2; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Blue);
      validDrink = true;
      break;
    case 5:  // UV Blue Cran-Grape
      pump1Time = 0.0; // Apple Brandy
      pump2Time = mixerPumpTime; // Cran-Grape
      pump3Time = shotPumpTime; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Purple);
      validDrink = true;
      break;
    case 6:  // UV Blue Cran-Grape Tall
      pump1Time = 0.0; // Apple Brandy
      pump2Time = mixerPumpTime; // Cran-Grape
      pump3Time = shotPumpTime * 2; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Purple);
      validDrink = true;
      break;
    case 7:  // UV Blue Lemonade
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = shotPumpTime; // UV Blue
      pump4Time = mixerPumpTime; // Lemonade
      setColor(Cyan);
      validDrink = true;
      break;
    case 8:  // UV Blue Lemonade Tall
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = shotPumpTime; // UV Blue
      pump4Time = mixerPumpTime * 2; // Lemonade
      setColor(Cyan);
      validDrink = true;
      break;
    case 9:  // Cran-Grape
      pump1Time = 0.0; // Apple Brandy
      pump2Time = mixerPumpTime; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Red);
      validDrink = true;
      break;
    case 10:  // Cran-Grape Tall
      pump1Time = 0.0; // Apple Brandy
      pump2Time = mixerPumpTime * 2; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = 0.0; // Lemonade
      setColor(Red);
      validDrink = true;
      break;
    case 11:  // Lemonade
      pump1Time = 0.0; // Apple Brandy
      pump2Time = 0.0; // Cran-Grape
      pump3Time = 0.0; // UV Blue
      pump4Time = mixerPumpTime; // Lemonade
      setColor(Yellow);
      validDrink = true;
      break;
    case 12:  // Lemonade Tall
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