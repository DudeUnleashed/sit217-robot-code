#include <IRremote.h>
IRrecv irrecv(12);
decode_results results;

//initialise Ultrasonic sensor pins
#define TRIG_PIN A1
#define ECHO_PIN A0
int Ult_distance=0;

//IR values for buttons on remote
#define IR_Forward 0x00ff629d
#define IR_Backward 0x00ffa857
#define IR_Left 0x00ff22dd
#define IR_Right 0x00ffc23d
#define IR_Stop 0x00ff02fd

#define Lpwm_pin 5 //speed pin for left motor
#define Rpwm_pin 6 //speed pin for right motor

int pinLB=2; //pin for left back
int pinLF=4; //pin for left front
int pinRB=7; //pin for right rear
int pinRF=8; //pin for right front

float checkdistance() 
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  float distance = pulseIn(ECHO_PIN, HIGH) / 58.00;
  delay(10);
  return distance;
}

void forward(unsigned char speed) //speed 0-255
{
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,LOW );  
     digitalWrite(pinLB,HIGH); 
     digitalWrite(pinLF,LOW);
     analogWrite(Lpwm_pin,speed);
     analogWrite(Rpwm_pin,speed);
}

void backward(unsigned char speed)  //speed 0-255
{
     digitalWrite(pinRB,LOW);  
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  
     digitalWrite(pinLF,HIGH);
     analogWrite(Lpwm_pin,speed);
     analogWrite(Rpwm_pin,speed);
}
    
void left(unsigned char speed)  //speed 0-255
{
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,LOW );  
     digitalWrite(pinLB,LOW); 
     digitalWrite(pinLF,HIGH);
     analogWrite(Lpwm_pin,speed);
     analogWrite(Rpwm_pin,speed);   
}

void right(unsigned char speed) //speed 0-255
{
     digitalWrite(pinRB,LOW);  
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,LOW);  
     analogWrite(Lpwm_pin,speed);
     analogWrite(Rpwm_pin,speed);
}    

void stopp()  //stop
{
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
}

void IR_Control(void)
{
   unsigned long Key;
   if(irrecv.decode(&results)) //judging if serial port receives data   
 {
     Key = results.value;
    switch(Key)
     {
       case IR_Forward:forward(50);
       Serial.println("forward"); //Make rover go forwards
       break;
       case IR_Backward:backward(50);
       Serial.println("backward");  //Make rover go backwards
       break;
       case IR_Left:left(25);   
       Serial.println("left"); //Turn rover Left    
       break;
       case IR_Right:right(25); 
       Serial.println("right");  //Turn rover Right
       break;
       case IR_Stop:stopp();
       Serial.println("stop"); //Stop rover
       break;
       default: 
       break;      
     } 
     irrecv.resume(); // Receive the next value
    } 
}

void setup() 
{
  //initialise all pins
   pinMode(pinLB,OUTPUT);
   pinMode(pinLF,OUTPUT);
   pinMode(pinRB,OUTPUT);
   pinMode(pinRF,OUTPUT);
   pinMode(Lpwm_pin,OUTPUT);
   pinMode(Rpwm_pin,OUTPUT);  

   pinMode(TRIG_PIN, OUTPUT);
   pinMode(ECHO_PIN, INPUT);

   //enable IR reciever
   irrecv.enableIRIn();

   //initialising serial port at 9600
   Serial.begin(9600);

   //make sure all motors are stopped
   stopp(); 
}

void loop() 
{
  //Ult_distance = checkdistance();
  //Serial.print("Distance:");
  //Serial.print(Ult_distance);
  //Serial.println("CM");
  
  IR_Control();
}
