static uint32_t lastPrint = 0;
// static uint32_t TotalTime = 0;

uint16_t lastRaw = 0;
int counter = 0;

//for stepper driving
int32_t pulses = 0;
int32_t pulseFinal = 0;
int16_t direction = 0;
int16_t position = 0;
int16_t positionTrue = 0;

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
uint16_t downVal = 12152; //value when straight down
uint16_t correction = downVal - 8192;//value for straight up
float angleInt = 0;
float angleDiff = 0;
float angleLast = 0;
float targetAngle = 0;
float targetAngleCap = 0;
float targetAngleDiff = 0;
float targetAngleLast = 0;
float error = 0;

int32_t message = 65535;
int32_t messageLast = 65535;
int32_t Test = 123456;
//----------------------------------------------------

//function prototypes
void motorInit();
void motorControl(int32_t pulses);
void interruptInit();

void setup() {
  motorInit();
  interruptInit();
  motorControl(65535);

  pinMode(22,INPUT_PULLUP); //limit switch inputs
  pinMode(27,INPUT_PULLUP);
  pinMode(23,INPUT_PULLUP); //Button input

  digitalWrite(9,HIGH); //disable motor  
  
  Serial.begin(115200); 
  while(!Serial){}

 while (Serial.available() <= 3){ //wait for a serial communication
        //delay(0.00001);
      }
      //message = Serial.read();
      //Serial.readBytes((char*)&message, 4);

  digitalWrite(9,LOW); //enable motor
}

void loop() {

      //check limit switches
      if(digitalRead(22) == 0 || digitalRead(27) == 0 || digitalRead(23) == 0){ //if limit switch or button is triggered
        digitalWrite(9,HIGH); //disable motor
        TCCR3B = 0x00; //disable timer
        while(1){} //catch program
      }

      messageLast = message; //store last message

      //receive serial data
      if(Serial.available() >= 4){   //if data is availible
        //message = Serial.read();
        Serial.readBytes((char*)&message, 4); //read the data

      
        if(message == 0xFFFFFFFF){ //if termination message is sent
          TCCR3B = 0x00;
          digitalWrite(9,HIGH); //disable motor
        }
        else{ //otherwise
          
          //filter recieved value
//        if(abs(abs(message) - abs(messageLast)) > 10000){
//          message = messageLast;
//        }

          motorControl(message); //forward the motor value
        }

        positionTrue = position; //copy position
        
        Serial.write(positionTrue>>8); //send position to the raspberry pi
        Serial.write(positionTrue);
        //Serial.write('\n'); 
      }
      
      
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

void motorControl(int32_t pulses){
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
}
