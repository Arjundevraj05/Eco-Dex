#include "CytronMotorDriver.h"
#include <ESP32Servo.h>

Servo clawL;
Servo clawR;
Servo bin;
Servo camera;

int leftMotorPWM = 19;
int leftMotorDIR = 18;
int rightMotorPWM = 5;
int rightMotorDIR = 17;
int servoclawL = 4;
int servoclawR = 16;
int binservo = 13;
int posclawR = 0.00;
int posclawL = 180.00;
int camerapin = 15;

 
int posClosingclawL = 0;
int boolbin = 0;
int cameraPos = 0;

CytronMD motor1(PWM_DIR, leftMotorPWM, leftMotorDIR );  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR,rightMotorPWM , rightMotorDIR); // PWM 2 = Pin 9, DIR 2 = Pin 10.
void straight(){
  motor1.setSpeed(50);
  motor2.setSpeed(50);


}
void reverse(){
  motor1.setSpeed(-100);
  motor2.setSpeed(-100);


}
void stop(){
  motor1.setSpeed(0);
  motor2.setSpeed(0);

}
void left(){
  motor1.setSpeed(-50);
  motor2.setSpeed(50);

}
void right(){
  motor1.setSpeed(50);
  motor2.setSpeed(-50);

}
void setup() {
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  clawR.setPeriodHertz(50); 
  clawL.setPeriodHertz(50); 
  bin.setPeriodHertz(50);
  camera.setPeriodHertz(50);

  clawL.attach(servoclawL,500,2400);
  clawR.attach(servoclawR,500,2400);
  bin.attach(binservo,500,2400);
  camera.attach(camerapin,500,2400);
  clawL.write(180);
  clawR.write(0);
  bin.write(0);

  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){
    char ch = Serial.read();
    if( ch == 'a'){
      straight();

    }
    else if( ch =='s'){
      left();

    }
    else if( ch=='w'){
      right();
    }
    else if( ch == 'd'){
    reverse();
      
    }
    else if( ch == 'c'){
      /*for(int i = posclawR , j = posclawL ; i<=180 && j >=90 ; i++ , j--){
        clawL.write(j);
        clawR.write(i);

        posclawR = i;
        posclawL = j;
      }*/
      //clawR.write(180);
      clawR.write(100);
      delay(300);
      clawL.write(100);
      delay(300);
      clawR.write(180);
      delay(300);
      clawL.write(0);
      delay(300);


    }
    else if( ch == 'o'){
      for(int i = posclawR , j = posclawL ; i>=0 && j <=180 ; i-- , j++  ){
        clawL.write(j);
        clawR.write(i);

        posclawR = i;
        posclawL = j;
      }


    }

    else if( ch == 'b'){
      if(boolbin== 1){
        bin.write(0);
        boolbin = 0;
      }
      else{
        bin.write(110);
        boolbin = 1;
      }


    }

    else if(ch == 'l'){

    }
    else if(ch == 'k'){

    }
    else if(ch == 'i'){
      for(int i = cameraPos ; i <=60 ; i ++){
        camera.write(i);
        cameraPos= i;
      }

    }
    else if(ch == 'u'){
      for(int i = cameraPos ; i >=0 ; i --){
        camera.write(i);
        cameraPos= i;
      }

    }
    else{
      stop();
    }

  }
   

}