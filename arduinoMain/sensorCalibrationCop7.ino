/*
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  //actual values
  int tablesr0c[]={4,673,484,344,281,223,190,143,139,106,26,45,53,97,42,100,52,32,56,0};
  int tablesr1c[]={2,663,489,341,268,227,136,197,123,65,93,144,53,73,112,43,27,54,35,0};
  int tablesr2c[]={4,664,481,325,254,206,191,167,127,122,107,92,67,92,92,75,59,69,51,0};
  int table3rc[]={3,215,583,596,514,467,404,346,264,155,80,51,44,42,37,43,40,39,40,0};
  int table4rc[]={5,672,513,360,278,216,189,123,131,173,130,135,106,84,105,86,76,84,45,0};
  int table5rc[]={5,672,523,355,285,231,195,165,154,137,124,121,95,100,78,84,71,67,55,0};

  //excel values
  int tablesr0c[]={4,673,484,344,281,223,190,143,139,106,96,85,75,57,42,49,41,32,27,0};
  int tablesr1c[]={2,663,489,341,268,227,197,163,145,129,114,102,90,73,70,61,53,38,35,0};
  int tablesr2c[]={4,664,481,325,254,206,191,167,127,122,107,83,67,92,70,65,59,47,40,0};

  //3 cm interval for front sensors
  int tablesr0c[]={3,558,623,500,391,322,286,245,258,192,166,187,149,102,155,122,114,135,49,0};
  int tablesr1c[]={3,540,620,499,389,320,271,227,207,213,149,155,144,171,90,47,160,173,60,0};
  int tablesr2c[]={3,548,622,490,379,311,265,227,195,189,166,155,142,147,107,117,111,99,105,0};
*/
#include <ZSharpIR.h>

#define SRmodel 1080
#define LRmodel 20150

#define s0 A0 //front right
#define s1 A1 //front middle
#define s2 A2 //front left
#define s3 A3 //left long range
#define s4 A4 //right bottom
#define s5 A5 //right top

//short IR sensor
//c - calibrated
ZSharpIR sr0c(s0,SRmodel);
ZSharpIR sr1c(s1,SRmodel);
ZSharpIR sr2c(s2,SRmodel);

ZSharpIR sr4c(s4,SRmodel);
ZSharpIR sr5c(s5,SRmodel);

//long IR sensor
ZSharpIR sr3c = ZSharpIR(s3,LRmodel);

void setupSensorsCalibration(){
  int tablesr0c[]={3,623,437,313,249,176,160,140,130,129,117,86,70,50,38,30,20,15,10,0};
  int tablesr1c[]={3,619,431,308,250,200,180,165,139,119,53,56,84,89,25,21,60,80,48,0};
  int tablesr2c[]={3,621,410,295,230,190,168,145,122,118,95,82,91,95,78,59,55,54,71,0};
  
  int tablesr3c[]={2,275,486,528,482,433,382,335,293,258,235,217,197,182,170,156,148,139,139,0};
  
  int tablesr4c[]={3,621,491,342,262,228,216,158,112,132,128,92,108,79,76,72,60,48,64,0};
  int tablesr5c[]={3,605,494,338,260,217,185,155,132,126,112,97,132,84,89,80,68,72,60,0};
  
  sr0c.ApplyCalibration(tablesr0c);
  sr1c.ApplyCalibration(tablesr1c);
  sr2c.ApplyCalibration(tablesr2c);
  sr3c.ApplyCalibration(tablesr3c);
  sr4c.ApplyCalibration(tablesr4c);
  sr5c.ApplyCalibration(tablesr5c);
}

int getFrontRight(){
  ZSharpIR sr0(s0, SRmodel);
  return getDistance(sr0, sr0c);
}

int getFrontRightRaw(){
  ZSharpIR sr0(s0, SRmodel);
  return getDistanceRaw(sr0, sr0c);
}

int getFrontMiddle(){
  ZSharpIR sr1(s1, SRmodel);
  return getDistance(sr1, sr1c);
}

int getFrontMiddleRaw(){
  ZSharpIR sr1(s1, SRmodel);
  return getDistanceRaw(sr1, sr1c);
}

int getFrontLeft(){
  ZSharpIR sr2(s2, SRmodel);
  return getDistance(sr2, sr2c);
}

int getFrontLeftRaw(){
  ZSharpIR sr2(s2, SRmodel);
  return getDistanceRaw(sr2, sr2c);
}

int getLeft(){
  ZSharpIR sr3(s3, LRmodel);
  return getDistance(sr3, sr3c);
}

int getRightBack(){
  ZSharpIR sr4(s4, SRmodel);
  return getDistance(sr4, sr4c);
}

int getRightBackRaw(){
  ZSharpIR sr4(s4, SRmodel);
  return getDistanceRaw(sr4, sr4c)+1;
}

int getRightFront(){
  ZSharpIR sr5(s5, SRmodel);
  return getDistance(sr5, sr5c);
}

int getRightFrontRaw(){
  ZSharpIR sr5(s5, SRmodel);
  return getDistanceRaw(sr5, sr5c);
}

//return distance from sensors (cm)
int getDistance(ZSharpIR sensor, ZSharpIR sensorc){
  int dist;
  //multipler to set ratio for calibrated and non-calibrated value
  double multipler = 1;
  int lrMax = 60;
  int srMax = 30;

  boolean sr = true;
  if(sensor.getMax() != 800){
    sr = false;
  }

  dist = sensorc.distance();

  //if reading above/ below a certain range
  if(sr==false && dist > lrMax){
    dist = sensor.distance()/10;
  }
  
  else if(sr==true && dist > srMax){
    dist = sensor.distance()/10;
  }
  
  //otherwise use calibrated value
  else{
    dist = multipler*dist + (1-multipler)*(sensor.distance()/10);
  }
  dist = dist/10;
  
  if(sr==false){
    return dist;
  }
  if(sensorc.getIrPin() == sr4c.getIrPin()){
    return (dist+1)/10 + 1;
  }
  return dist+1;
}

int getDistanceRaw(ZSharpIR sensor, ZSharpIR sensorc){
  int dist;
  //multipler to set ratio for calibrated and non-calibrated value
  double multipler = 1;
  int lrMax = 60;
  int srMax = 30;
  boolean sr = true;
  if(sensor.getMax() != 800){
    sr = false;
  }
  dist = sensorc.distance();

  //if reading above/ below a certain range
  if(sr==false && dist > lrMax){
    dist = sensor.distance()/10;
  }
  else if(sr==true && dist > srMax){
    dist = sensor.distance()/10;
  }
  else{
    dist = multipler*dist + (1-multipler)*(sensor.distance()/10);
  }

  return dist;
}

//calibrate sensors
//calibration table of 20 value,
//1st is for 0cm
//next is for 5cm, next for 10,.... up to 95cm
void calibrateSensors(){

    //callibrate front
    Serial.println("Front calibration, send g to begin");
    while(Serial.read() != 'g'){}
    
    sr0c.CalibrateStart();
    sr1c.CalibrateStart();
    sr2c.CalibrateStart();
    
    for(int i=0; i<19;i++){
      sr0c.CalibrateNextStep();
      sr1c.CalibrateNextStep();
      sr2c.CalibrateNextStep();
      
      Serial.println("sr0 ");
      sr0c.DisplayCalibration(Serial);
      Serial.println("sr1 ");
      sr1c.DisplayCalibration(Serial);
      Serial.println("sr2");
      sr2c.DisplayCalibration(Serial);
      
      Serial.print("Move to ");
      Serial.print((i+1)*5);
      Serial.print(", and send g\n");
      while(Serial.read() != 'g'){}
      delay(1000);
    }
    
    Serial.println("End of calibration");
    
    sr0c.DisplayCalibration(Serial);
    sr1c.DisplayCalibration(Serial);
    sr2c.DisplayCalibration(Serial);


    //calibrate left side
    Serial.println("Left calibration, send g to begin");
    while(Serial.read() != 'g'){}
    
    sr3c.CalibrateStart();
    
    for(int i=0; i<19;i++){
      sr3c.CalibrateNextStep();
      sr3c.DisplayCalibration(Serial);
      Serial.print("Move to ");
      Serial.print((i+1)*5);
      Serial.print(", and send g\n");
      while(Serial.read() != 'g'){}
      delay(1000);
    }
    
    Serial.println("End of calibration");
    sr3c.DisplayCalibration(Serial);

    //calibrate right side
    Serial.println("Right calibration, send g to begin");
    while(Serial.read() != 'g'){}
    
    sr4c.CalibrateStart();
    sr5c.CalibrateStart();
    
    for(int i=0; i<19;i++){
      sr4c.CalibrateNextStep();
      sr4c.DisplayCalibration(Serial);
      sr5c.CalibrateNextStep();
      sr5c.DisplayCalibration(Serial);
      Serial.print("Move to ");
      Serial.print((i+1)*5);
      Serial.print(", and send g\n");
      while(Serial.read() != 'g'){}
      delay(1000);
    }
    
    Serial.println("End of calibration");
    sr4c.DisplayCalibration(Serial);
    sr5c.DisplayCalibration(Serial);

}
