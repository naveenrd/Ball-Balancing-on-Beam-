int last_reading;
#include<Servo.h>
#include<PID_v1.h>

const int servoPin = 5;                                               
const int TrigPin = 6;
const int EchoPin = 7;

float Kp = 0.6;                                                      
float Ki = 0.15;                                                      
float Kd = 0.38;                                                          
int i,j,s=21.5;  
float Input                               

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);                                                                                  
Servo myServo;                                                       

void setup() { 
  Serial.begin(9600);                                                //Begin Serial.
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  
  myServo.attach(servoPin);                                          //Attach Servo.
  Input = readPosition();                                            
                                                                     //  position as the input to the PID algorithm.
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC.
  myPID.SetOutputLimits(-15,15);                                     //Set Output limits to -15 and 15 degrees. 
}

void loop()
{
   if(Serial.available()>0)
   {     
      String data= Serial.readString(); // reading the data received from the bluetooth module
      i=(data[0]-'0')*10; 
      j=data[1]-'0';
      s=i+j;
   }   
  Setpoint =s;
  Input = readPosition();                                            


 if(Input<45)
 {
  myPID.Compute();                                                  
  ServoOutput=85+Output;                                            
  myServo.write(ServoOutput);                                        
 }
  
}
      
float readPosition() {
  //delay(20);                                                            
  
  double duration, cm;
  double now = millis();
  //pinMode(TrigPin, OUTPUT);
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);


  //pinMode(EchoPin, INPUT);
  duration = pulseIn(EchoPin, HIGH);

  cm = duration/(29*2);
  
  Serial.println(Setpoint-cm);
  
  return cm;                                         
}
