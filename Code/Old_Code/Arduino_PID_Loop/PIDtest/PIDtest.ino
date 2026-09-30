#include <Wire.h>

const uint8_t MT6701_ADDR = 0x06;  //default 7-bit I2C address
const uint8_t ANGLE_MSB = 0x03;
const uint8_t ANGLE_LSB = 0x04;

const uint16_t SAMPLE_PERIOD_MS = 1;  //100 Hz

//for stepper driving
int32_t pulses = 0;
int32_t pulseFinal = 0;
int16_t direction = 0;
int16_t position = 0;

//velocity/frequency calculation
float u = 0;
float v = 0;
float f = 0;

float fLast = 0;
float filter = 1250; //max Hz/second
float filter2 = 0;

static uint32_t tlast = 0; //setup variable for time storage

int16_t raw = 0;
int16_t rawlast = 5850;
//5724 up
uint16_t downVal = 14388; //value when straight down
uint16_t correction = downVal - 8192;//value for straight up
float angleInt = 0;
float angleDiff = 0;
float angleLast = 0;
float targetAngle = 0;
float targetAngleCap = 0;
float targetAngleDiff = 0;
float targetAngleLast = 0;
float error = 0;

// uint16_t readAngleRaw() {
//   Wire.beginTransmission(MT6701_ADDR);
//   Wire.write(ANGLE_MSB);
//   Wire.endTransmission(false);
  
//   Serial.println("bR");
//   Wire.requestFrom(MT6701_ADDR, (uint8_t)2);
//   Serial.println("aR");

//   if (Wire.available() < 2) return 0;
//   uint8_t msb = Wire.read();
//   uint8_t lsb = Wire.read();
//   return ((uint16_t)msb << 6) | (lsb & 0x3F);  //14-bit value
// }

uint16_t readAngleRaw() {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(ANGLE_MSB);
  uint8_t response = Wire.endTransmission(true);

  if(response != 0){ //if no response
    return rawlast; //return previous reading
  }

  // Serial.println("bR");
  // noInterrupts();
  Wire.requestFrom(MT6701_ADDR, (uint8_t)2);
  // interrupts();
  // Serial.println("aR");

  if (Wire.available() < 2) return 0;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t)msb << 6) | (lsb & 0x3F);  //14-bit value
}

void setup() {
  delay(2000); //wait 2 seconds before starting initialization

  pinMode(22,INPUT_PULLUP); //limit switch inputs
  pinMode(27,INPUT_PULLUP);

  motorInit();
  interruptInit();
  Serial.begin(115200);
  while (!Serial) {}
  Wire.setClock(400000);
  Wire.setTimeout(5); //set timeout of 1ms
  Wire.begin();
  
  tlast = millis(); //store initial time
  delay(1);
}

void loop() {
  // static uint32_t t0 = millis();
  // static uint32_t next = 0;

  // uint32_t now = millis();
  // if ((now - t0) >= next) {
  //   next += SAMPLE_PERIOD_MS;

    static uint32_t tcurrent = millis(); //get current time
    static uint32_t tstep = tcurrent - tlast; //calculate time step
    tlast = tcurrent; //store previous time

    // Serial.println("bR");
    // noInterrupts();
    raw = readAngleRaw();
    // interrupts();
    if(raw-rawlast > 2000 || raw-rawlast < -2000){
      raw = rawlast;
    }
    rawlast = raw; //store previous value
    Serial.println(raw);

    int32_t rawshift = raw;
    if(rawshift<0){
      rawshift = rawshift + 16384;
    }
    else if(rawshift>16383){
      rawshift = rawshift - 16384;
    }

    float angleRad = (rawshift / 16384.0f) * 2.0f * PI - (correction / 16384.0f) * 2.0f * PI; //get adjusted angle in radians
    // Serial.println("aA");

    targetAngle = (float)position / 50000;
    targetAngleCap = constrain(targetAngle, -0.03, 0.03);

    error = angleRad - targetAngleCap;
    angleInt = angleInt + error * tstep; //approximate integral
    angleInt = constrain(angleInt, -200, 200); //cap integral value
    angleDiff = (error - angleLast)/tstep; //calculate derivative of angle
    angleLast = error;
    targetAngleDiff = (targetAngle - targetAngleLast)/tstep;
    targetAngleLast = targetAngle;


    if(angleRad>1 || angleRad<-1){ //if angle is too big, or the limit switch is triggered
      digitalWrite(9,HIGH); //disable motor
      Serial.println("angle");
      Serial.println(raw);
      Serial.println(angleRad,3);
      while(1){}
    }
    if(digitalRead(22) == 0 || digitalRead(27) == 0){ //if angle is too big, or the limit switch is triggered
      digitalWrite(9,HIGH); //disable motor
      Serial.println("limit");
      while(1){}
    }

    //calculate control force
    u = 25*error + 7*angleInt + 13*angleDiff - 25*targetAngle - 50*targetAngleDiff;

    f = u * 30;
    f = f * 4;

    // filter2 = filter*tstep;

    // if((f-fLast) > filter2){
    //   f = fLast + filter2;
    // } else if((f-fLast) < -filter2){
    //   f = fLast - filter2;
    // }

    // Serial.println("aC");
    
    motorControl((int16_t)f); //control the motor

    //Print one CSV row
    // Serial.println(position);
    // Serial.print(',');
    // Serial.println(raw);
  // }
}

void motorInit(){
  TCCR3A = 0x00;
	TCCR3B = 0x00;
	TCNT3H = 0x00;
	TCNT3L = 0x00;
	OCR3A = 0x0000;
	OCR3B = 0x0000;
	TIMSK3 = 0x00;
	TIFR3 = 0x00;

  TCCR3A = (1<<COM3A0); //set toggle on compare match
	TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS31) | (1<<CS30); //set the prescaler to 64, set CTC and start timer

  pinMode(9,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(5,OUTPUT);

  digitalWrite(9,LOW); //set enable high (active low)
  digitalWrite(7,LOW); //set direction high
}

void motorControl(int16_t frequency){
  //convert frequency to max timer value
  if(frequency > 2 || frequency < -2){
    float conversion = (0.5/frequency)/(1.0/250000.0);
    pulses = conversion;
  } else if(frequency >= 0){
    pulses = 65535;
  } else if(frequency < 0){
    pulses = -65535;
  }

  if(digitalRead(9) == 1) pulses = 65535;
  
  //configure motor ouputs
  if(pulses>0){
    //digitalWrite(9,HIGH); //set enable high
    digitalWrite(7,HIGH); //set direction high
    direction = -1;
    pulseFinal = pulses;
  } else if(pulses<0){
    //digitalWrite(9,HIGH); //set enable high
    digitalWrite(7,LOW); //set direction low
    direction = 1;
    pulseFinal = pulses * -1; //make value positive
  }
 
  //set timer max value
  ICR3 = pulseFinal; //set max timer value
  if(TCNT3>ICR3){ //if the timer value is too big
    TCNT3 = ICR3 - 1;//set TCNT just under ICR3 so it will toggle the output and reset
  }
}

void interruptInit(){
  EICRA = (1<<ISC31);// | (1<<ISC30); //set interrupt 3 to trigger on falling edge
	EIMSK = (1<<INT3); //enable external interrupt 3

  sei(); //enable interrupt
  // interrupts();
}

ISR(INT3_vect) //interrupt from OC3A
{
  //increment position by direction
  position = position + direction;

  //check direction, and increment the position
	// if(direction == 1){
  //   position = position + 1;
  // } else{
  //   position = position - 1;
  // }
}