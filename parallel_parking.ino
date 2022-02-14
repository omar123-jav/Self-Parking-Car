#include <Arduino_FreeRTOS.h>
#include <Servo.h>

//Ulra Sonic Pins (3 ultra sonic sensors)
//Back sensor
#define trigPin1 A3
#define echoPin1 A2
//LLeft sensor
#define trigPin2 A4
#define echoPin2 A5
//Front sensor
#define trigPin3 A0 
#define echoPin3 A1 
// Motors Speed 
#define S 70

// Motor A (FR)

#define enA 4
#define in1  48
#define in2  46

// Motor B (FL)

#define enB 5
#define in3  52
#define in4 50

// Motor A (RR)

#define enC 2
#define in5  28
#define in6  26

// Motor A (RL)

#define enD 6
#define in7  24
#define in8  22

#define A 30
#define B 31
#define C 32
#define D 33
#define E 34
#define F 35
#define G 36

#define common_cathode 0
#define common_anode 1
bool segMode = common_anode; // set this to your segment type, my segment is common_cathode
int seg[] {A,B,C,D,E,F,G}; // segment pins
byte chars = 3; // max value in the array "Chars"

byte Chars[3][9] { 
            {'0',1,1,1,1,1,1,0,0},//0
            {'1',0,1,1,0,0,0,0,0},//1
            {'2',1,1,0,1,1,0,1,0},//2
            };

//parking space
int space=0;
//Move Functions Prototype
void Stop();
void MoveForward();
void MoveLeft();
void MoveBack();
void MoveRight();
//ultra Sonic Function prototype
int UFun(int triger,int echo);
//functions prototypes
void Park();
int Check();
void buzzer();

void parkingTask(void *pvParameters);

boolean parked;
const int sensorMin = 0;     // sensor minimum
const int sensorMax = 1024;  // sensor maximum
int pos = 0;
int rainRange = 0;
int servoPin = 9;
int rainPin = A9;
int fuelLevelPin = A6;

Servo myservo;

void setup() {
//ultra sonic pins
  pinMode(trigPin1,OUTPUT);
  pinMode(echoPin1,INPUT);
  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);
  pinMode(trigPin3,OUTPUT);
  pinMode(echoPin3,INPUT);
  
// Set all the motor control pins to outputs

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(seg[0],OUTPUT);
  pinMode(seg[1],OUTPUT);
  pinMode(seg[2],OUTPUT);
  pinMode(seg[3],OUTPUT);
  pinMode(seg[4],OUTPUT);
  pinMode(seg[5],OUTPUT);
  pinMode(seg[6],OUTPUT);
  pinMode(seg[7],OUTPUT);

  myservo.attach(servoPin);

  parked = false;

  xTaskCreate(buzzerTask, "Buzzer Task", 128, NULL, 1, NULL);
  xTaskCreate(parkingTask, "Parking Task", 128, NULL, 1, NULL);
  xTaskCreate(rainTask, "Rain Task", 128, NULL, 1, NULL);
  xTaskCreate(servoTask, "Servo Task", 128, NULL, 1, NULL);
  xTaskCreate(fuelLevelTask, "Fuel Level Task", 128, NULL, 1, NULL);

  Serial.begin(9600);
}

void setState(bool mode) //sets the hole segment state to "mode"
{ for(int i = 0;i<=6;i++)
{
  digitalWrite(seg[i],mode);
}
}

void Print(int num) // print any number on the segment
{ 
  setState(segMode);//turn off the segment
  

 
if(num > chars || num < 0 )// if the number is not declared
{
  for(int i = 0;i <= 6;i++)
  {
  digitalWrite(seg[i],HIGH);
  delay(100);
  digitalWrite(seg[i],LOW);
  }
  for(int i = 0;i <= 2;i++)
{
  delay(100);
  setState(HIGH);
  delay(100);
  setState(LOW); 
}
}else // else if the number declared, print it
 {
  if(segMode == 0){ //for segment mode
for(int i = 0;i<8;i++)
    {digitalWrite(seg[i],Chars[num][i+1]);
    }
 }
 else{
  for(int i = 0;i<8;i++)
    {digitalWrite(seg[i],!Chars[num][i+1]);
    }
 }
 }
}


void loop() {
//  delay(2000);
//  if (Check()==1&& space>3){//space is measured by trial and error to check the right value ,it's varied by the speed
//    space=0;
//    Stop();
//    delay(1000);
//    Park();
//    exit(0);
//  }
//  else {
//    if(UFun(trigPin1,echoPin1)>4&&UFun(trigPin3,echoPin3)>4)
//    {
//      MoveForward();
//    }
//    else{
//      Stop();
//      }
//    }
}

void parkingTask(void *pvParameters) {
  const TickType_t xDelay100ms = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1 && !parked) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (Check()==1&& space>3){//space is measured by trial and error to check the right value ,it's varied by the speed
      space=0;
      Stop();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      Park();
      vTaskDelete(NULL);
//      exit(0);
    }
    else {
      if(UFun(trigPin1,echoPin1)>10&&UFun(trigPin3,echoPin3)>10)
      {
        MoveForward();
      }
      else{
        Stop();
        }
      }
      vTaskDelayUntil( &xLastWakeTime, xDelay100ms );// DelayUntil
  }
}

void Park(){//Parking algorithm
  MoveBack();
  vTaskDelay(1500 / portTICK_PERIOD_MS);
  Stop();
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  MoveRight();
  vTaskDelay(450 / portTICK_PERIOD_MS);
  int c =3;
  while(c-- > 0){
    MoveBack();
//    delay(8000);
  }
  Stop();
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  MoveLeft();
  vTaskDelay(500 / portTICK_PERIOD_MS);
  Stop();
  parked = true;
  }
int Check(){//Checking the suitable parking space for the car width
  int Flag =0;
  
  const TickType_t xDelay100ms = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while(UFun(trigPin2,echoPin2)>20){
    MoveForward();
    space++;
    Flag =1;
//    vTaskDelayUntil( &xLastWakeTime, xDelay100ms );
    }
  return Flag;
}

void buzzerTask(void *pvParameters){
  const TickType_t xDelay100ms = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    int buzzerDelay;
//    Serial.println(UFun(trigPin1,echoPin1));
//    Serial.println(UFun(trigPin3,echoPin3));
//    Serial.println("---");
  
  
    if(UFun(trigPin1,echoPin1) <= 5 || UFun(trigPin3,echoPin3) <= 5){
      buzzerDelay = 100;
      tone(13, 1500);
      delay(buzzerDelay);
      noTone(13);
    }else if(UFun(trigPin1,echoPin1) <= 15 || UFun(trigPin3,echoPin3) <= 15){
      buzzerDelay =500;
      tone(13, 1500);
      delay(buzzerDelay);
      noTone(13);
    }else{
      buzzerDelay = 1500;
      noTone(13);
    }
    
     vTaskDelayUntil( &xLastWakeTime, xDelay100ms );// DelayUntil
  }
}

int UFun(int triger , int echo){//ultrasonic reading 
//  Serial.println("Test");
   long duration ;
   int distance;
        digitalWrite(triger,LOW);
        delayMicroseconds(2);
        digitalWrite(triger, HIGH);
        delayMicroseconds(10);
        digitalWrite(triger,LOW);
    
        duration = pulseIn(echo,HIGH);
        distance = duration/2 /29.1 ;
//        if (triger == trigPin1)
//          Serial.println(triger);
//          Serial.println(distance);
        // 331.5 m/s ===> 0 C.....331.5 +0.6*temp....343.5*100/1000000=.03435cm/us
  return distance; 
  }
void MoveBack(){
  // Turn on motor A
      
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
      
        // Set speed to 200 out of possible range 0~255
      
        analogWrite(enA, S);
      
        // Turn on motor B
      
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
      
        // Set speed to 200 out of possible range 0~255
      
        analogWrite(enB, S);

        digitalWrite(in5, LOW);
        digitalWrite(in6, HIGH);
      
        // Set speed to 200 out of possible range 0~255
      
        analogWrite(enC, S);

        digitalWrite(in7, HIGH);
        digitalWrite(in8, LOW);
      
        // Set speed to 200 out of possible range 0~255
      
        analogWrite(enD, S);

        delay(250);

        Stop();

        delay(500);
  
  
  }
void MoveLeft(){

  // Turn on motor A

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enA, S);

  // Turn on motor B

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enB, S);

  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enC, S);

  // Turn on motor B

  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enD, S);
  

  }
void MoveRight(){
      
    // Turn on motor A

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enA, S);

  // Turn on motor B

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enB, S);

  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enC, S);

  // Turn on motor B

  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enD, S);


     }
void Stop(){
  // Now turn off motors
 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);

  digitalWrite(in5, HIGH);
  digitalWrite(in6, HIGH);  
  digitalWrite(in7, HIGH);
  digitalWrite(in8, HIGH);
  
  }
void MoveForward() {
    // Turn on motor A

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enA, S);

  // Turn on motor B

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enB, S);

  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enC, S);

  // Turn on motor B

  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);

  // Set speed to 200 out of possible range 0~255

  analogWrite(enD, S);

  delay(250);

  Stop();

  delay(500);

}

void rainTask(void *pvParameters) {
  const TickType_t xDelay100ms = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    int sensorReading = analogRead(rainPin);
    rainRange = map(sensorReading, sensorMin, sensorMax, 0, 3);
    vTaskDelayUntil( &xLastWakeTime, xDelay100ms );
  }

}

void servoTask( void *pvParameters ) {
  const TickType_t xDelay100ms = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
//    Serial.println(rainRange);
    if (rainRange != 2) {
      for (pos = 0; pos <= 180; pos += 2 - rainRange) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      for (pos = 180; pos >= 0; pos -= (2 - rainRange)) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      vTaskDelayUntil( &xLastWakeTime, xDelay100ms );
    }
  }
}

void fuelLevelTask( void *pvParameters ) {
  const TickType_t xDelay100ms = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    int value = analogRead(fuelLevelPin);
    Serial.println(value);
    if (value < 300){
      Print(0);
    } else if (value < 640){
      Print(1);
    } else {
      Print(2);
    }
    vTaskDelayUntil( &xLastWakeTime, xDelay100ms );
  }

}
