#include <Servo.h>
#include "CytronMotorDriver.h"
#include <AccelStepper.h>
//Servo Declarations
servo clawR;
Servo clawL;
Servo Bin;
Servo camera;
int StepperPos = 0;
bool Biodegradble = false;
//Stepper Pins
const int motorPin1 = 14;  // IN1
const int motorPin2 = 27;  // IN2
const int motorPin3 = 26;  // IN3
const int motorPin4 = 25;  // IN4
AccelStepper stepper(4, motorPin1, motorPin2, motorPin3, motorPin4);
void openClaw(){
  for(int i =0 ; j = 180 ; i<=180 , j >=0 ; i++ ,j-- ){
    clawL.write(j);
    clawR.write(i);

  }

}

void closeClaw(){
  for(int i =180 ; j = 0 ; i>=0 , j <=180 ; i-- ,j++ ){
    clawL.write(j);
    clawR.write(i);
    
  }

void CameraDown(){
  camera.write(60);
}

void CameraUp(){
  camera.write(0);
}
  
void LidOpen(){
  for(i=0 ; i < 6 ; i++ ){
    moveOneRevoluiton(true);

  }

}
void LidDown(){
  for(int i = 0 ; i < 6 ; i++){
  moveOneRevolution(false);

}
void moveOneRevolution(bool Flag) {
   
  if(bool){
    stepper.moveTo(2048);
  }
  else{
    stepper.moveTo(-2048);
  }
  
  stepper.runToPosition();   
}

void setup() {
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  clawR.setPeriodHertz(50);
  clawL.setPeriodHertz(50);
  camera.setPeriodHertz(50);
  Bin.setPeriodHertz(50);  
  clawR.attach(16);
  clawL.attach(4);
  Bin.attach(15);
  camera.attach();
  CytronMD motor1(PWM_DIR, , );
  CytronMD motor2(PWM_DIR, , );
  stepper.setMaxSpeed(1000);  // steps per second
  stepper.setAcceleration(500);  // steps per second^2
}



  
  
   

}

void loop() {
  if(Serial.available()>0){
    char = Serial.read();
    if(ch=='a'){
      straight();

    }
    else if(ch=='b'){
      CameraDown();
      
      

    }
    else if(ch=='c'){
      LidDown();
      While(true){
        if(StepperPos == 50){
          ClawOpen();
        }
    
      }
      closeClaw();
      LidOpen();


      

    }
    else{
      move_straight();
    }
   
}
