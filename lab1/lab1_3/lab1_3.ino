#include "ArduinoMotorShieldR3.h"
#include <SPI.h>

#define ARRAY_SIZE    7500   // empirical max = 7500 with 3 arrays(unsigned long, long, and float)
#define TS            5000   // [us] sampling period
#define RECORD_TIME   2e6    // [us] duration to record data

// encoder chip select pins
int chipSelectPin1 = 10;
int chipSelectPin2 = 9;
int chipSelectPin3 = 8;

// motor shield object
ArduinoMotorShieldR3 md;

// setup routine
void setup() {

  // initialize serial comms
  Serial.begin(115200);
  Serial.println("ENGS 147 Lab 1");

  // set up motor shield
  md.init();

  // set up encoder shield
  pinMode(chipSelectPin1, OUTPUT);
  pinMode(chipSelectPin2, OUTPUT);
  pinMode(chipSelectPin3, OUTPUT);
  digitalWrite(chipSelectPin1, HIGH);
  digitalWrite(chipSelectPin2, HIGH);
  digitalWrite(chipSelectPin3, HIGH);
  LS7366_Init();

  // pause while things finish
  delay(100);
}

void loop() {  // treating loop() like main() here, and letting it repeat... our actual sampling loop is nested within...

  // variables
  unsigned long arr_time[ARRAY_SIZE] = {};
  long arr_pos[ARRAY_SIZE] = {};
  float arr_vel[ARRAY_SIZE] = {};
  unsigned long *ptr_time = nullptr;
  long *ptr_pos = nullptr;
  float *ptr_vel = nullptr;
  size_t array_size = 0;
  unsigned long progstart, loopstart, curtime, dt;
  long encoder3Value, dcount;
  bool array_overrun_flag = false;
  bool TS_overrun_flag = false;

  // make sure we're starting from rest
  md.setM1Speed(0);
  delay(1000);

  // turn motor on to 50% speed
  // NEGATIVE SIGN TO ACCOUNT FOR DIRECTION OF EXTERNAL ENCODER COUNT
  md.setM1Speed(-200);

  // reset all pointers and array size
  ptr_time = arr_time;
  ptr_pos = arr_pos;
  ptr_vel = arr_vel;
  array_size = 0;

  // collect data for two seconds
  curtime = micros();
  progstart = curtime;
  while (curtime < (progstart + RECORD_TIME)) {

    // read encoder
    encoder3Value = getEncoderValue(3);  // as close to micros() as possible
    loopstart = curtime;                 // absolute count of when this sampling period started
    
    // store time, position, and velocity
    *ptr_time = loopstart - progstart;  // time stored is [us] since this sampling period began
    *ptr_pos = encoder3Value;
    if (ptr_time == arr_time) {
      *ptr_vel = 0;                     // report no speed on first iteration
    } else {
      dcount = encoder3Value - *(ptr_pos - 1);  
      dt = *ptr_time - *(ptr_time - 1);
      *ptr_vel = (dcount * 1.0F / dt) * ((float)1e6);   // simple first backward difference approximation to derivative
    }

    // increment pointers and array size
    ++ptr_time;
    ++ptr_pos;
    ++ptr_vel;
    ++array_size;

    // check for array overrun
    // and reset pointers to avoid
    // actually overrunning allocations
    if (array_size >= ARRAY_SIZE) {
      array_size = 0;
      ptr_time = arr_time;
      ptr_pos = arr_pos;
      ptr_vel = arr_vel;
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
  ptr_time = arr_time;
  ptr_pos = arr_pos;
  ptr_vel = arr_vel;
  while (ptr_time < (arr_time + array_size)) {
    Serial.print(*ptr_time);
    Serial.print(",");
    Serial.print(*ptr_pos);
    Serial.print(",");
    Serial.println(*ptr_vel, 8);
    ++ptr_time;
    ++ptr_pos;
    ++ptr_vel;
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
void selectEncoder(int encoder)
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
