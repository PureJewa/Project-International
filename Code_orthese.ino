#include <Wire.h>
#include <MPU6050.h>
#include <TomIBT2.h>
#include <LiquidCrystal_I2C.h>  // Gebruik LiquidCrystal_I2C bibliotheek voor I2C LCD-scherm
// Arduino pinnen
#define MOTOR_PIN_R_EN 7
#define MOTOR_PIN_L_EN 8
#define MOTOR_PIN_RPWM 10  // PWM 490.20Hz
#define MOTOR_PIN_LPWM 9   // PWM 490.20Hz
#define knopPin 52         // Pin voor de modusknop
#define selectButton 53    // Pin voor selecteren

//Constante
#define THRESHOLD 5000    // Drempelwaarde voor activering van de motor
#define RAMP_DURATION 1   // Duur van de ramp (in milliseconden)
#define MOTORSPEED 30000  // Snelheid van de motor
#define HOEK1 120
#define HOEK2 -120
#define STAANHOEK 20
#define ZITHOEK 90
#define calibrationSamples 1000

// LCD-scherm instellingen
#define LCD_ADDRESS 0x27  // I2C-adres van het LCD-scherm (kan variëren)
#define LCD_COLUMNS 16    // Aantal kolommen
#define LCD_ROWS 2        // Aantal rijen

// Libraries
MPU6050 mpu;
TomIBT2 motor(MOTOR_PIN_R_EN, MOTOR_PIN_L_EN, MOTOR_PIN_RPWM, MOTOR_PIN_LPWM);
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);  // Instantie van het I2C LCD-scherm

// Constanten
int BUTTONSTATE = 0;
int BEGIN_ROTATIE = -180;  // Begin positie knie
int EIND_ROTATIE = 180;    // eind positie knie
int16_t acclX_cal[calibrationSamples];
int16_t acclY_cal[calibrationSamples];
int16_t acclZ_cal[calibrationSamples];
int16_t gyroX_cal[calibrationSamples];
int16_t gyroY_cal[calibrationSamples];
int16_t gyroZ_cal[calibrationSamples];
int16_t gyroX, gyroY, gyroZ;
int16_t acclX, acclY, acclZ;
int16_t rotx, roty, rotz;
float Temp;
int acclY_normalized;
int sitAngle = 0;
int standAngle = 0;
int threshhold = 5000;

// Modusvariabele
enum Mode {
  STIL,
  WALK,
  STAND,
  SIT,
  TRAIN,
  CALIBRATE,
};

enum Rehab{
  SITANGLE,
  STANDANGLE,
  THRESHOLDINC,
  THRESHOLDDEC,
  BACK
};
Mode currentMode = STIL;  // Standaardmodus is Stil
Rehab currentCalibrationMode = SITANGLE; // Variabele om de huidige kalibratiemodus bij te houden

void setup() {
  Serial.begin(9600);
  // Initialiseer het I2C LCD-scherm
  lcd.init();
  lcd.backlight();      // Inschakelen van de LCD-achtergrondverlichting
  lcd.setCursor(0, 0);  // Zet cursor naar positie (0,0)
  lcd.print("Starting up");
  motor.begin();
  
  Wire.begin();
  mpu.initialize();
  currentModeToString();
  checkModeChange();
  //calibrateSensor();
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  //mpu.CalibrateGyro();
  // Configureer de knoppenpin als een invoer
  pinMode(knopPin, OUTPUT);
  pinMode(selectButton, OUTPUT);
  lcd.setCursor(0, 0);  // Zet cursor naar positie (0,0)
  lcd.print("Ready!");
  delay(1000);
  lcd.setCursor(0, 0);  // Zet cursor naar positie (0,0)
  lcd.print("Rotatie: ");

  // Weergave van de huidige modus op het LCD-scherm
  lcd.setCursor(0, 1);
  lcd.print("M ");
  lcd.print(currentModeToString());


}
void checkModeChange() {
  BUTTONSTATE = digitalRead(knopPin);
  if (BUTTONSTATE == HIGH) {
    // Verhoog de huidige modus met 1
    currentMode = static_cast<Mode>((currentMode + 1) % 6);  // Zorg ervoor dat de modus binnen het bereik blijft (0-6)
    delay(10);
    // Weergave van de huidige modus op het LCD-scherm
    lcd.setCursor(2, 1);
    lcd.print("             ");  // Wis vorige modus
    lcd.setCursor(2, 1);
    lcd.print(currentModeToString());  // Print de nieuwe modus op het scherm
  }
}
//Functie om de orthese te calibreren
void calibrateSensor() {
  Serial.println("Calibration mode...");
  delay(5);  // Just to ensure stability

  for (int i = 0; i < calibrationSamples; i++) {
    int16_t acclX, acclY, acclZ;
    int16_t gyroX, gyroY, gyroZ;

    mpu.getAcceleration(&acclX, &acclY, &acclZ);
    mpu.getRotation(&gyroX, &gyroY, &gyroZ);

    // Store accelerometer and gyroscope data in arrays
    acclX_cal[i] = acclX;
    acclY_cal[i] = acclY;
    acclZ_cal[i] = acclZ;
    gyroX_cal[i] = gyroX;
    gyroY_cal[i] = gyroY;
    gyroZ_cal[i] = gyroZ;

    delay(10);  // Adjust the delay according to your needs
  }

  Serial.println("Calibration completed.");
}
// Functie om de motor naar de gewenste snelheid te brengen met een lineaire ramp
void setMotorSpeed(int targetSpeed, TomIBT2::Direction direction) {
  motor.rotate(MOTORSPEED, direction);
}
// Functie om de waardes van de MPU omtezetten naar overzichtelijke waardes
int mapAcceleration(int acceleration) {
  // Map de inputwaarde van -16000 tot 16000 naar het bereik van -180 tot 180
  return map(acceleration, -16200, 16200, BEGIN_ROTATIE, EIND_ROTATIE);
}
// Functie om de motor geleidelijk tot stilstand te brengen
void stopMotor() {
  motor.stop();
}
// Controleer of de gyroscoopwaarde hoger is dan de drempelwaarde of lager dan de negatieve drempelwaarde
void AansturenMotor() {
  switch (currentMode) {
    case STIL:
      stopMotor();
    break;
    case WALK:
      stopMotor();
      int acclY_normalized = mapAcceleration(acclY);

        if (abs(acclY_normalized) < HOEK1 | acclY_normalized > HOEK2) {
            Serial.print("acclY_normalized");
            Serial.print(acclY_normalized);

              if (gyroZ > THRESHOLD) {
                int motorSpeed = map(abs(gyroZ), 0, 32767, 0, MOTORSPEED);
                setMotorSpeed(motorSpeed, TomIBT2::CW);
              }
              else if (gyroZ < -THRESHOLD) {
                int motorSpeed = map(abs(gyroZ), 0, 32767, 0, MOTORSPEED);
                setMotorSpeed(motorSpeed, TomIBT2::CCW);
              } 
         else {
            stopMotor();
          }
        }
      break;
    case STAND:
      stopMotor();

    if (abs(acclY_normalized) < STAANHOEK) {

      if (gyroZ > 0 && abs(gyroZ) > THRESHOLD) {
        setMotorSpeed(MOTORSPEED, TomIBT2::CW);
      } else {
        stopMotor();
      }
    }
      break;
    case SIT:
      stopMotor();
    if (abs(acclY_normalized) < ZITHOEK) {
        if (gyroZ > 0 && abs(gyroZ) > THRESHOLD) {
          setMotorSpeed(MOTORSPEED, TomIBT2::CCW);
        } else {
          stopMotor();
        }
    }
      break;
    case TRAIN:
      // Handle TRAIN mode
      break;
    case CALIBRATE:
      
      break;
  }
};

// Data voor tijdens het programmeren
void DataPrinten() {
  int acclY_normalized = mapAcceleration(acclY);
  Serial.print("gyroX: ");
  Serial.print(gyroX);
  Serial.print(", gyroY: ");
  Serial.print(gyroY);
  Serial.print(", gyroZ: ");
  Serial.println(gyroZ);  //belangrijk
  Serial.print(", acclX: ");
  Serial.println(acclX);
  Serial.print(", acclY: ");  // belangrijk
  Serial.println(acclY);
  Serial.print(", acclZ: ");
  Serial.println(acclZ);
  Serial.println(acclY_normalized);
  Serial.print(", rotx: ");
  Serial.println(rotx);
  Serial.print(", roty: ");
  Serial.println(roty);
  Serial.print(", rotz: ");
  Serial.println(rotz);
  Serial.print(", temp: ");
  Serial.println(Temp);
};
// Zorg dat vooruit/acheruit op het scherm komt te staan
void LCDscherm() {
  lcd.setCursor(9, 0);  // Zet cursor naar positie (9,0)
  if (gyroZ > 1000) {
    lcd.print("Vooruit");  // Print de gyroZ-waarde op het scherm
  } else if (gyroZ < -1000) {
    lcd.print("Achteruit");
  } else if (500 < gyroZ > -500) {
    lcd.print("Stil     ");
  }
};
void changeCalibrationMode() {
    if (digitalRead(knopPin) == HIGH) {
        // Schakel tussen de kalibratiemodi
        currentCalibrationMode = static_cast<Rehab>((currentCalibrationMode + 1) % 5);
        delay(100); // Debounce-delay
    }
}

void handleCalibrationMode() {
    switch (currentCalibrationMode) {
        case BACK:
            if(digitalRead(selectButton) == HIGH)
            currentMode = STIL;
            break;
        case SITANGLE:
            // Code voor het instellen van de zithoek
            if (digitalRead(selectButton) == HIGH){
              sitAngle = acclY_normalized;
            };
            break;
        case STANDANGLE:
            // Code voor het instellen van de standhoek
            if (digitalRead(selectButton) == HIGH){
              standAngle = acclY_normalized;
            };
            break;
        case THRESHOLDINC:
            // Code voor het verhogen van de kalibratiedrempel
            if (digitalRead(selectButton) == HIGH){
              threshhold =+ 50;
            }
            break;
        case THRESHOLDDEC:
            // Code voor het verlagen van de kalibratiedrempel
            if (digitalRead(selectButton) == HIGH){
              threshhold =- 50;
            }
            break;
    }
}


String currentModeToString() {
  switch (currentMode) {
    case STIL:
      return "Stil";
      break;

    case WALK:
      return "Walk";
      break;

    case STAND:
      return "Stand";
      break;

    case SIT:
      return "Sit";
      break;

    case TRAIN:
      return "Train";
      break;

    case CALIBRATE:
      return "Calibrate";
      break;
    }
  }

void readTemperature() {
  Wire.beginTransmission(0x68);  // MPU6050 address
  Wire.write(0x41);              // Temperature register address
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 2);  // Request 2 bytes from MPU6050
  int16_t rawTemp = (Wire.read() << 8 | Wire.read());

  Temp = (rawTemp / 340.0) + 36.53;  // Convert raw temperature to Celsius
}
void loop() {
  // Controleer of de modusknop is ingedrukt
  checkModeChange();
  if (currentMode == CALIBRATE && digitalRead(selectButton == HIGH)){
    changeCalibrationMode();
    handleCalibrationMode();
  }
  else {
  checkModeChange();
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  mpu.getAcceleration(&acclX, &acclY, &acclZ);
  mpu.getRotation(&rotx, &roty, &rotz);
  readTemperature();
  //  mpu.PID();
  DataPrinten();
  // Weergave van gyroscoopwaarde op het LCD-scherm
  LCDscherm();
  AansturenMotor();
  // Voeg een kleine vertraging toe tussen iteraties
  millis();
  //delay(1000);
};
}