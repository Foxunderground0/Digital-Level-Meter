#include "TinyWireM.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

float accelX, accelY, accelZ;
char mpu = 0x68;
long ofsetX, ofsetY;
const int ledPositive = 1;
const int ledNegative = 4;
const float threshold = 2.5;
boolean slowPolling = false;
float accelAngleX, accelAngleXOfset;
long powerDownDelay = 0;
const int powerDownDelayThreshold = 1200;
float lastVal = 0;
float seccondLastVal = 0;


void setup() {
  power_adc_disable();
  //ADCSRA &= ~_BV(ADEN);                   // ADC off

  //pinMode(3, INPUT);
  pinMode(ledNegative, OUTPUT);
  pinMode(ledPositive, OUTPUT);
  digitalWrite(ledNegative, LOW);
  digitalWrite(ledPositive, LOW);

  TinyWireM.begin();

  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x6B); //  Power setting 1 address
  TinyWireM.write(0b10000000); //  Acceleration data register
  TinyWireM.endTransmission();

  delay(2000);

  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x75); //  Whoami data register
  TinyWireM.endTransmission();

  TinyWireM.requestFrom(mpu, 1);

  int test = TinyWireM.read() ;

  if ((test xor 0b01101000) != 0) {
    while (1) {
      digitalWrite(ledPositive, HIGH);
      delay(100);
      digitalWrite(ledPositive, LOW);
      delay(100);

      TinyWireM.beginTransmission(mpu); //I2C address of the MPU
      TinyWireM.write(0x75); //  Whoami data register
      TinyWireM.endTransmission();

      TinyWireM.requestFrom(mpu, 1);
      test = TinyWireM.read() ;
    }
  }

  TinyWireM.beginTransmission(mpu);
  TinyWireM.write(0x6B); //  Power setting 1 address
  TinyWireM.write(0b00000000);
  TinyWireM.endTransmission();

  TinyWireM.beginTransmission(mpu);
  TinyWireM.write(0x6C); //  Power setting 2 address
  TinyWireM.write(0b00000000);
  TinyWireM.endTransmission();

  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x1C); // Accelerometer config register
  TinyWireM.write(0b00001000); // 4g range +/- (default)
  TinyWireM.endTransmission();

  digitalWrite(ledPositive, HIGH);
  delay(1000);
  digitalWrite(ledPositive, LOW);
  callibrateAccel();
  delay(100);
  digitalWrite(ledPositive, HIGH);
  delay(100);
  digitalWrite(ledPositive, LOW);
  callibrateAngle();
  delay(100);
  digitalWrite(ledPositive, HIGH);
  delay(100);
  digitalWrite(ledPositive, LOW);

  //Enable Delayed Polling with sleep
  TinyWireM.beginTransmission(mpu);
  TinyWireM.write(0x6B); //  Power setting 1 address
  TinyWireM.write(0b00101000);
  TinyWireM.endTransmission();

  TinyWireM.beginTransmission(mpu);
  TinyWireM.write(0x6C); //  Power setting 2 address
  TinyWireM.write(0b11000111);
  TinyWireM.endTransmission();


  TinyWireM.beginTransmission(mpu);
  TinyWireM.write(0x37); //  Intterupt CFG
  TinyWireM.write(0b00010000);
  TinyWireM.endTransmission();

  TinyWireM.beginTransmission(mpu);
  TinyWireM.write(0x38); //  Intterup CFG
  TinyWireM.write(0b00000001);
  TinyWireM.endTransmission();

}

void sleepIdle() {
  if (slowPolling == true) {
    TinyWireM.beginTransmission(mpu);
    TinyWireM.write(0x6C); //  Power setting 2 address
    TinyWireM.write(0b11000111);
    TinyWireM.endTransmission();
    slowPolling = false;
  }

  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT3);                   // Use PB3 as interrupt pin
  set_sleep_mode(SLEEP_MODE_IDLE);    // replaces above statement

  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                  // Enable interrupts
  sleep_cpu();                            // sleep

  cli();                                  // Disable interrupts
  PCMSK &= ~_BV(PCINT3);                  // Turn off PB3 as interrupt pin
  sleep_disable();                        // Clear SE bit

  sei();                                  // Enable interrupts
}

void sleepPowerDown() {
  powerDownDelay++;
  if (powerDownDelay > powerDownDelayThreshold) {
    powerDownDelay = powerDownDelayThreshold + 2;
    if (slowPolling == false) {
      TinyWireM.beginTransmission(mpu);
      TinyWireM.write(0x6C); //  Power setting 2 address
      TinyWireM.write(0b00000111);
      TinyWireM.endTransmission();
      slowPolling = true;
    }

    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    PCMSK |= _BV(PCINT3);                   // Use PB3 as interrupt pin
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep
    sleep_cpu();                            // sleep
    sleep_cpu();                            // sleep
    sleep_cpu();                            // sleep
    sleep_cpu();                            // sleep
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts
    PCMSK &= ~_BV(PCINT3);                  // Turn off PB3 as interrupt pin
    sleep_disable();                        // Clear SE bit

    sei();                                  // Enable interrupts
  } else {
    sleepIdle();
  }
}

ISR(PCINT0_vect) {
}

ISR(WDT_vect) {
}

void loop() {
  getAccelCalibrated();

  accelAngleX = (atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI) - accelAngleXOfset; // AccErrorX ~(0.58)
  //int accelAngleY = (atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

  float tempVal = accelAngleX;

  accelAngleX = (seccondLastVal+lastVal+ tempVal)/3.0;
  
  seccondLastVal = lastVal;
  lastVal = tempVal;
  
  if (accelAngleX > threshold) {
    powerDownDelay = 0;
    analogWrite(ledPositive, min(int(pow(2.0, (abs(accelAngleX) / 10.0)) - 1.0), 255));
    analogWrite(ledNegative, 0);
    sleepIdle();
  } else if (accelAngleX < -threshold) {
    powerDownDelay = 0;
    analogWrite(ledPositive, 0);
    analogWrite(ledNegative, min(int(pow(2.0, (abs(accelAngleX) / 10.0)) - 1.0), 255));
    sleepIdle();
  } else {
    analogWrite(ledNegative, 0);
    analogWrite(ledPositive, 0);
    sleepPowerDown();
  }
}

void callibrateAngle() {
  for (int i = 0; i < 100; i++) {
    getAccelCalibrated();
    accelAngleXOfset += (atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI);
  }
  accelAngleXOfset = accelAngleXOfset / 100;
}


void callibrateAccel() {
  for (int i = 0; i < 100; i++) {
    getAccelUncalibrated();
    ofsetX += accelX;
    ofsetY += accelY;
  }
  ofsetX = ofsetX / 100;
  ofsetY = ofsetY / 100;
}

void getAccelCalibrated() {
  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x3B); //  Acceleration data register
  TinyWireM.endTransmission();

  TinyWireM.requestFrom(mpu, 6); // Get 6 bytes, 2 for each DoF
  accelX = (TinyWireM.read() << 8 | TinyWireM.read()) / 8192.0;
  accelY = (TinyWireM.read() << 8 | TinyWireM.read()) / 8192.0;
  accelZ = (TinyWireM.read() << 8 | TinyWireM.read()) / 8192.0;

  accelX -= ofsetX;
  accelY -= ofsetY;
}

void getAccelUncalibrated() {
  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x3B); //  Acceleration data register
  TinyWireM.endTransmission();

  TinyWireM.requestFrom(mpu, 6); // Get 6 bytes, 2 for each DoF
  accelX = (TinyWireM.read() << 8 | TinyWireM.read()) / 8192.0;
  accelY = (TinyWireM.read() << 8 | TinyWireM.read()) / 8192.0;
  accelZ = (TinyWireM.read() << 8 | TinyWireM.read()) / 8192.0;
}
