#include "ArduinoMotorShieldR3.h"
#include <SPI.h>

#define ARRAY_SIZE 2500   // empirical max = 7500 with 3 arrays(unsigned long, long, and float)
#define TS 10000          // [us] sampling period
#define RECORD_TIME 4e6   // [us] duration to record data
#define PWM_MAX 400
#define REF_INPUT 80.0F   // [rad/s]
#define KP -0.10F         // proportional gain

// encoder chip select pin
int chipSelectPin1 = 10;
int chipSelectPin2 = 9;
int chipSelectPin3 = 8;

// instance of motor shield class
ArduinoMotorShieldR3 md;

// setup routine
void setup() {

  // initialize serial comms
  Serial.begin(115200);
  Serial.println("ENGS 147 Lab 2");

  // set up motor shield
  md.init();

  // set up encoder shield
  pinMode(chipSelectPin3, OUTPUT);
  digitalWrite(chipSelectPin3, HIGH);
  LS7366_Init();

  // pause while things finish
  delay(100);
}

void loop() {  // treating loop() like main() here, and letting it repeat... our actual sampling loop is nested within...

  // array variables
  // TODO: this is getting ridiculous... use a 2D array
  unsigned long   arr_tim[ARRAY_SIZE]   = {};
  long            arr_pos[ARRAY_SIZE]   = {};
  float           arr_vel[ARRAY_SIZE]   = {};
  float           arr_err[ARRAY_SIZE]   = {};
  float           arr_vlt[ARRAY_SIZE]   = {};
  int16_t         arr_pwm[ARRAY_SIZE]   = {};
  unsigned long   *ptr_tim              = nullptr;
  long            *ptr_pos              = nullptr;
  float           *ptr_vel              = nullptr;
  float           *ptr_err              = nullptr;
  float           *ptr_vlt              = nullptr;
  int16_t         *ptr_pwm              = nullptr;  
  size_t          array_size            = 0;

  // other variables
  unsigned long progstart, loopstart, curtime, dt;
  long encoder3Value, dcount;
  bool array_overrun_flag = false;
  bool TS_overrun_flag = false;
  float err_sig;
  long pwm_val;

  // make sure we're starting from rest
  md.setM1Speed(0);
  delay(1000);

  // reset all pointers and array size
  ptr_tim = arr_tim;
  ptr_pos = arr_pos;
  ptr_vel = arr_vel;
  ptr_vlt = arr_vlt;
  ptr_err = arr_err;
  ptr_pwm = arr_pwm;  
  array_size = 0;

  // collect data for two seconds
  curtime = micros();
  progstart = curtime;
  while (curtime < (progstart + RECORD_TIME)) {

    // TIME CRITICAL OPERATIONS

    // read encoder
    encoder3Value = getEncoderValue(3);  // as close to micros() as possible

    // compute velocity
    if (ptr_tim == arr_tim) {
      *ptr_vel = 0;  // report no speed on first iteration
    } else {
      dcount = encoder3Value - *(ptr_pos - 1);
      dt = (curtime - progstart) - *(ptr_tim - 1);
      *ptr_vel = (dcount * 1.0F / dt) * ((float)1e6) * (6.28F / 1440.0F);  // simple first backward difference approximation to derivative
    }

    // compute error signal
    *ptr_err = ((float)REF_INPUT) - *ptr_vel;

    // compute pwm value
    *ptr_vlt = ((float)KP)*(*ptr_err);
    *ptr_pwm = (int16_t)(((float)(*ptr_vlt))*(50.0F));

    // saturate pwm value at limits
    if((*ptr_pwm) > PWM_MAX){
      *ptr_pwm = PWM_MAX;
    } else if((*ptr_pwm) < -1*PWM_MAX) {
      *ptr_pwm = -1*PWM_MAX;  
    }

    // write pwm value to motor
    md.setM1Speed(*ptr_pwm);

    // END OF TIME-CRITICAL OPERATIONS

    // store time and position
    *ptr_tim = curtime - progstart;  // time stored is [us] since this sampling period began
    *ptr_pos = encoder3Value;

    // increment pointers and array size
    ++ptr_tim;
    ++ptr_pos;
    ++ptr_vel;
    ++ptr_err;
    ++ptr_vlt;
    ++ptr_pwm;
    ++array_size;

    // save time that loop started
    loopstart = curtime;  // absolute count of when this sampling period started

    // check for array overrun
    // and reset pointers to avoid
    // actually overrunning allocations
    if (array_size >= ARRAY_SIZE) {
      array_size = 0;
      ptr_tim = arr_tim;
      ptr_pos = arr_pos;
      ptr_vel = arr_vel;
      ptr_err = arr_err;
      ptr_vlt = arr_vlt;
      ptr_pwm = arr_pwm;
      array_overrun_flag = true;
      Serial.println("************ ARRAY OVERRUN ************");
    }

    // check whether we overran sample time
    curtime = micros();
    if ((curtime - loopstart) > TS) {
      TS_overrun_flag = true;
    }

    // enforce loop timing
    curtime = micros();
    while ((curtime - loopstart) < TS) {
      curtime = micros();
    }
  }

  // stop motor
  md.setM1Speed(0);

  // print all stored data in serial stream
  ptr_tim = arr_tim;
  ptr_pos = arr_pos;
  ptr_vel = arr_vel;
  ptr_err = arr_err;
  ptr_vlt = arr_vlt;
  ptr_pwm = arr_pwm;
  while (ptr_tim < (arr_tim + array_size)) {
    Serial.print(*ptr_tim);
    Serial.print(",");
    Serial.print(*ptr_pos);
    Serial.print(",");
    Serial.print(*ptr_vel, 8);
        Serial.print(",");
    Serial.print(*ptr_err, 8);
        Serial.print(",");
    Serial.print(*ptr_vlt, 8);
        Serial.print(",");
    Serial.println(*ptr_pwm);
    ++ptr_tim;
    ++ptr_pos;
    ++ptr_vel;
    ++ptr_err;
    ++ptr_vlt;
    ++ptr_pwm;
  }

  // print warning if we overran TS at any point
  if (TS_overrun_flag) {
    Serial.println("************ SAMPLE TIME OVERRUN ************");
  }

  // wait before doing it all again
  delay(3000);
}


//*****************************************************
long getEncoderValue(int encoder)
//*****************************************************
{
  unsigned int count1Value, count2Value, count3Value, count4Value;
  long result;

  selectEncoder(encoder);

  SPI.transfer(0x60);                // Request count
  count1Value = SPI.transfer(0x00);  // Read highest order byte
  count2Value = SPI.transfer(0x00);
  count3Value = SPI.transfer(0x00);
  count4Value = SPI.transfer(0x00);  // Read lowest order byte

  deselectEncoder(encoder);

  result = ((long)count1Value << 24) + ((long)count2Value << 16) + ((long)count3Value << 8) + (long)count4Value;

  return result;
}  //end func

//*************************************************
void  selectEncoder(int encoder)
//*************************************************
{
  switch (encoder) {
    case 1:
      digitalWrite(chipSelectPin1, LOW);
      break;
    case 2:
      digitalWrite(chipSelectPin2, LOW);
      break;
    case 3:
      digitalWrite(chipSelectPin3, LOW);
      break;
  }  //end switch

}  //end func

//*************************************************
void deselectEncoder(int encoder)
//*************************************************
{
  switch (encoder) {
    case 1:
      digitalWrite(chipSelectPin1, HIGH);
      break;
    case 2:
      digitalWrite(chipSelectPin2, HIGH);
      break;
    case 3:
      digitalWrite(chipSelectPin3, HIGH);
      break;
  }  //end switch

}  //end func



// LS7366 Initialization and configuration
//*************************************************
void LS7366_Init(void)
//*************************************************
{


  // SPI initialization
  SPI.begin();
  //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
  delay(10);

  digitalWrite(chipSelectPin1, LOW);
  SPI.transfer(0x88);
  SPI.transfer(0x03);
  digitalWrite(chipSelectPin1, HIGH);


  digitalWrite(chipSelectPin2, LOW);
  SPI.transfer(0x88);
  SPI.transfer(0x03);
  digitalWrite(chipSelectPin2, HIGH);


  digitalWrite(chipSelectPin3, LOW);
  SPI.transfer(0x88);
  SPI.transfer(0x03);
  digitalWrite(chipSelectPin3, HIGH);

}  //end func
