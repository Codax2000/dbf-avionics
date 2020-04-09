#include <EnableInterrupt.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

File data;

#define RC_NUM_CHANNELS  5

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3
#define RC_CH5_INPUT  9

#define AILERON_PIN 7
#define ELEVATOR_PIN 6
#define THROTTLE_PIN 5
#define THROTTLE_PIN_2 10
#define RUDDER_PIN 3
#define BANNER_PIN 8
#define AIRSPEED_PIN A6

// values for banner release
#define REST 0
#define DEPLOY 50
#define RELEASE 150
uint16_t banner_microseconds;

// percent change for differential thrust
#define PERCENT_CHANGE .1

// Servos for controls
Servo servoAileron;
Servo servoElevator;
Servo servoThrottle;
Servo ServoThrottleRight;
Servo servoRudder;
Servo servoBanner;

// values for interrupts
uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

// values for airspeed
#define V_0 5.0
#define rho 1.204

// parameter for time management of airspeed reading
int mod_count = 0;

// parameters for averaging
int offset = 0; 
#define offset_size 10
#define veloc_mean_size 10
#define zero_span 2

// values for MPU
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;

// values for dif. thrust
static int thrust;
static int rudder;
static float percent_thrust_change;
static int thrust_left;
static int thrust_right;


void setup() {
  // initialize SD card reader
  SD.begin(4);
  data = SD.open("data.csv", FILE_WRITE);
  if (data) {
    //Serial.println("Connected Successfully");
    data.println("time,ax,ay,az,gx,gy,gz,airspeed");
  }
  data.close();

  // initialize MPU
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  accelgyro.initialize();
  
  // set pins
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);

  // establish interrupts
  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);

  // attach servos
  servoAileron.attach(AILERON_PIN);
  servoElevator.attach(ELEVATOR_PIN);
  servoThrottle.attach(THROTTLE_PIN);
  ServoThrottleRight.attach(THROTTLE_PIN_2);
  servoRudder.attach(RUDDER_PIN);
  servoBanner.attach(BANNER_PIN);

  // set up airspeed
  for (int ii = 0; ii < offset_size; ii++) {
    offset += analogRead(AIRSPEED_PIN) - (1023 / 2);
    
  }
  offset /= offset_size;
}

void loop() {
  rc_read_values();

  // implement differential thrust
  thrust = (int)(rc_values[RC_CH3]);
  rudder = (int)(rc_values[RC_CH4]);
  percent_thrust_change = (rudder - 1492) / (500 / PERCENT_CHANGE);
  thrust_left = (int)(thrust * (1 + percent_thrust_change));
  thrust_right = (int)(thrust * (1 - percent_thrust_change));
  
  // implement data collection from MPU
  servoAileron.writeMicroseconds(rc_values[RC_CH1]);
  servoElevator.writeMicroseconds(rc_values[RC_CH2]);
  servoThrottle.writeMicroseconds(thrust_left);
  ServoThrottleRight.writeMicroseconds(thrust_right);
  servoRudder.writeMicroseconds(rc_values[RC_CH4]);
  banner_microseconds = rc_values[RC_CH5];

  if (banner_microseconds > 1700) {
    servoBanner.write(REST);
  } else if (banner_microseconds > 1200) {
    servoBanner.write(DEPLOY);
  } else {
    servoBanner.write(RELEASE);
  }
  // records data every half second, change to 1000 for every second
  if (millis() > mod_count * 500) {
    read_data_and_record();
    mod_count++;
  }
  
}

// updates rc values
void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void read_data_and_record() {
  read_mpu_and_record();
  read_airspeed_and_record();
}

void read_mpu_and_record() {
  accelgyro.getAcceleration(&ax, &ay, &az);
  accelgyro.getRotation(&gx, &gy, &gz);
  data = SD.open("data.csv", FILE_WRITE);
  if (data) {
    data.print(mod_count); data.print(",");
    data.print(ax); data.print(",");
    data.print(ay); data.print(",");
    data.print(az); data.print(",");
    data.print(gx); data.print(",");
    data.print(gy); data.print(",");
    data.print(gz); data.print(",");
  }
  data.close();
}

void read_airspeed_and_record() {
  float adc_avg = 0;
  float veloc = 0.0;

  //average a few for stability
  for (int ii = 0; ii < veloc_mean_size; ii++) {
    adc_avg += analogRead(AIRSPEED_PIN) - offset;
  }
  adc_avg /= veloc_mean_size;
  
  // make sure if the ADC reads below 512, then we equate it to a negative velocity
  if (adc_avg > 512 - zero_span and adc_avg < 512 + zero_span){
  } else {
    if (adc_avg < 512){
      veloc = -sqrt((-10000.0 * ((adc_avg / 1023.0) - 0.5)) / rho);
    } else{
      veloc = sqrt((10000.0 * ((adc_avg / 1023.0) - 0.5)) / rho);
    }
  }
  data = SD.open("data.csv", FILE_WRITE);
  if (data) {
    //Serial.print(mod_count);
    //Serial.print(",");
    
    if (veloc < 0) {
      //Serial.println(0.00);
      data.println(0.00);
    } else {
      //Serial.println(veloc);
      data.println(veloc * 2.237);
    }
  }
  data.close();
}

// ISRs
void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }

// generic Interrupt Service Routine
void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}
