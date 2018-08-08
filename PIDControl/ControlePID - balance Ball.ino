int ultima_leitura;
#include<Servo.h>
#include<PID_v1.h>


const int servoPin = 9;                                             
const int TrigPin = 4;
const int EchoPin = 3;

float Kp = 5;                                                      
float Ki = 2;                                                      
float Kd = 2;                                                   
double Setpoint, entrada, saida, ServoOutput;                                       



PID myPID(&entrada, &saida, &Setpoint, Kp, Ki, Kd, DIRECT);           
                                                                      
                                                                     
                                                                     
                                                                     
Servo myServo;                                                       

void setup() {
  ultima_leitura=5;
  Serial.begin(9600);                                                
  pinMode(EchoPin, INPUT);
  
  myServo.attach(servoPin);                                          
  entrada = readPosition();                                            
                                                                     
 
  myPID.SetMode(AUTOMATIC);                                          
  myPID.SetOutputLimits(-30,30);                              
}

void loop()
{
 
  Setpoint = 15;
  entrada = readPosition();                                            

 
  myPID.Compute();                                                   
 ultima_leitura= entrada; 
  ServoOutput= 60 + saida;                      // defimos a horizontal com base no sergo sg90 
  myServo.write(ServoOutput);                                        
  
  
}
      
      
      

float readPosition() {
  delay(40);                                                            
  
long duration, cm;
unsigned long now = millis();
  //pinMode(TrigPin, OUTPUT);
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);


  //pinMode(EchoPin, INPUT);
  duration = pulseIn(EchoPin, HIGH);

  cm = duration/(29*2);
  
  
 if(cm > 30)     // 30 cm maxima distancia da base
  {cm=30;}

  if(cm<1)
  {cm=1;}
  //cm=30-cm;
 /* if (cm <7 ) {cm=6; Serial.println("Mazen");}*/
  Serial.println(cm);
  
  return cm;                                          //Retorna a distancia em cm.
}



