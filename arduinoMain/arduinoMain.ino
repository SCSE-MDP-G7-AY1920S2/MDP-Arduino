#include <ZSharpIR.h>

#include "mdp_motor.h"

String toSend = "";
String command = "";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(50);
  startEncoder();
  setupPID();
  setupSensorsCalibration();
}

void loop() {
  // put your main code here, to run repeatedly:

  //Serial.print(getLeft());
  //Serial.print(", ");
  //Serial.print(getRightBack());
  //Serial.print(", ");
  //Serial.print(getFrontRight());
  //Serial.print("\n");
  //delay(1000);

  calibrateAll();
  Serial.println("Done");
  delay(5000);

  String message = "";
  String com = "";
  if(Serial.available() > 0){
    message = Serial.readString();

    //send sensor values
    if (message.charAt(0) == 'K'){
      sendSensor();
    }

    //exploration move front
    else if (message.charAt(0) == 'W'){
      goForward(10);
      sendSensor();
    }

    //exploration move back
    else if (message.charAt(0) == 'S'){
      goBackward(10);
      sendSensor();
    }

    //exploration move lefft
    else if (message.charAt(0) == 'A'){
      turnLeft(90);
      sendSensor();
    }

    else if (message.charAt(0) == 'D'){
      turnRight(90);
      sendSensor();
    }

    //exploration end
    else if (message.charAt(0) == 'E'){
      sendFin();
    }

    //calibrate front
    else if (message.charAt(0) == 'F'){
      calibrateFront();
      sendFin();
    }

    //parallelwall
    else if (message.charAt(0) == 'R'){
      parallelWall();
      sendFin();
    }

    //turns robot back to "North" position
    //robot facing "South"
    else if (message.charAt(0) == 'G'){
      delay(15000);
      turnLeft(90);
      parallelWall();
      distanceFront();
      allignFront();
      turnRight(90);
      distanceFront();
      allignFront();
      turnRight(90);
    }

    //robot facing "East"
    else if (message.charAt(0) == 'H'){
      delay(15000);
      parallelWall();
      distanceFront();
      allignFront();
      turnRight(90);
      distanceFront();
      allignFront();
      turnRight(90);
    }

    //allign robot using front and side walls
    else if (message.charAt(0) == 'C'){
      calibrateAll();
      sendFin();
    }

    //finish
    else if (message.charAt(0) == 'F'){
      calibrateFront();
      sendFin();
    }
  }

  Serial.flush();
}

//aligns the robot against the wall
void parallelWall(){
  int rf = getRightFrontRaw();
  int rb = getRightBackRaw();
  int diff = rf-rb;
  startMotor();

  while(abs(diff) != 0){
    if(rf < rb){ //facing right
      //turnLeft(1);
      turnLeft(1);
    }
    else if(rb < rf){ //facing left
      //turnRight(1);
      turnRight(1);
    }
    rf = getRightFrontRaw();
    rb = getRightBackRaw();
    diff = rf-rb;
  }
  endMotor();
}

//align robot to the front (angle)
void allignFront(){

  int fr = getFrontRightRaw();
  int fl = getFrontLeftRaw();
  int diff = fr-fl;
  
  startMotor();

  while(abs(diff) != 0){

    if(fl > fr){ //facing right
      turnLeft(1);
    }
    else if(fr > fl){ //facing left
      turnRight(1);
    }

    fr = getFrontRightRaw();
    fl = getFrontLeftRaw();
    diff = fr-fl;
  }
  endMotor();
}

//align robot to the wall (distance)
void distanceFront()
{
  int diff;
  startMotor();
  while (getFrontMiddleRaw() != 10)
  {
    diff = abs(getFrontMiddleRaw() - 10);
    if (getFrontRightRaw() < 10)
      goBackward(diff);

    if (getFrontRightRaw() > 10)
      goForward(diff);
  }
  endMotor();
}

//combines allignFront and distanceFront function
void calibrateFront()
{
    allignFront();
    delay(100);
    distanceFront();
    delay(100);
}

//allign against front and side wall
void calibrateAll()
{
    parallelWall();
    delay(250);
    turnRight(90);
    delay(250);
    allignFront();
    delay(250);
    distanceFront();
    delay(250);
    turnLeft(90);
    parallelWall();
}

//send sensor data
void sendSensor(){
  //delay(1000);
  toSend = ";{\"from\":\"Arduino\",\"com\":\"SD\",\"fr\":";
  toSend.concat(getFrontRight());
  
  toSend.concat(",\"fl\":");
  toSend.concat(getFrontLeft());
  
  toSend.concat(",\"fm\":");
  toSend.concat(getFrontMiddle());
  
  toSend.concat(",\"left\":");
  toSend.concat(getLeft());
  //toSend.concat("-1");
  
  toSend.concat(",\"rf\":");
  toSend.concat(getRightFront());
  //toSend.concat("-1");
  
  toSend.concat(",\"rb\":");
  toSend.concat(getRightBack());
  //toSend.concat("-1");
  toSend.concat("}");

  Serial.println(toSend);
  Serial.flush();
}

void sendFin()
{
  toSend = ";{\"from\":\"Arduino\",\"com\":\"C\"}";
  
  Serial.println(toSend);
  Serial.flush();
}
