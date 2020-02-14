#include <ZSharpIR.h>
#include <DualVNH5019MotorShield.h>

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
  //calibrateSensors();
  
  String message = "";
  String com = "";
  if(Serial.available() > 0){
    message = Serial.readString();

    //send sensor values
    if (message.charAt(0) == 'K'){
      sendSensor();
    }

    //exploration move front
    else if (message.charAt(0) == 'W')
    {
      goForward(10);
      sendSensor();
    }

    //exploration move back
    else if (message.charAt(0) == 'S')
    {
      goBackward(10);
      sendSensor();
    }

    //exploration move lefft
    else if (message.charAt(0) == 'A')
    {
      turnLeft(90);
      sendSensor();
    }

    else if (message.charAt(0) == 'D')
    {
      turnRight(90);
      sendSensor();
    }

    //exploration end
    else if (message.charAt(0) == 'E')
    {
      sendFin();
    }

    //calibrate front
    else if (message.charAt(0) == 'F')
    {
      //calibrateFront();
      sendFin();
    }

    //parallelwall
    else if (message.charAt(0) == 'R')
    {
      //parallelWall();
      sendFin();
    }
  }
  Serial.flush();
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