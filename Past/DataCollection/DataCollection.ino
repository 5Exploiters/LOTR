
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu1;
MPU6050 mpu2(0x69); //if ADO high --> MPU6050 mpu(0x69)

/* =========================================================================
 NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
 depends on the MPU-6050's INT pin being connected to the Arduino's
 external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
 digital I/O pin 2.
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus1;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus2;
uint8_t devStatus1;     // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize2;
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint16_t fifoCount2; 
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t fifoBuffer2[64]; // FIFO storage buffer

int count=0;
// orientation/motion vars
Quaternion q1;           // [w, x, y, z]         quaternion container
Quaternion q2;
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int arr [50][3];
int arr1 [50][3];
int num;
int num1;
int big;
int big1;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

//* byte start_address = 0;
//* byte end_address = 127;
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  //*byte rc;
  Wire.begin();
  
  for (int i=0; i<50; i++){
    arr[i][0]=1000;
    arr[i][1]=1000;
    arr[i][2]=1000;
  }
  for (int i=0; i<50; i++){
    arr1[i][0]=1000;
    arr1[i][1]=1000;
    arr1[i][2]=1000;
  }
  // initialize serial communication
  //*Serial.begin(9600);
  //*Serial.println("\nI2C Scanner");
  Serial.begin(115200);
  while (!Serial);

  /* /=========================================================================================================  
   // I2C address CONTROL
   
   //Serial.print("Scanning I2C bus from ");
   //Serial.print(start_address,DEC);  Serial.print(" to ");  Serial.print(end_address,DEC);
   //Serial.println("...");
   
   for( byte addr  = start_address;
   addr <= end_address;
   addr++ ) {
   Wire.beginTransmission(addr);
   rc = Wire.endTransmission();
   
   if (addr<16) Serial.print("0");
   Serial.print(addr,HEX);
   if (rc==0) {
   Serial.print(" found!");
   } else {
   Serial.print(" "); Serial.print(rc); Serial.print("     ");
   }
   Serial.print( (addr%8)==7 ? "\n":" ");
   }
   
   Serial.println("\n-------------------------------\nPossible devices:");
   
   for( byte addr  = start_address;
   addr <= end_address;
   addr++ ) {
   Wire.beginTransmission(addr);
   rc = Wire.endTransmission();
   if (rc == 0) {
   Serial.print(addr,HEX); Serial.print(" = ");
   switch (addr) {
   case 0x50: Serial.println("AT24C32/AT24C64 - EEPROM"); break;
   case 0x68: Serial.println("DS1307"); break;
   default: Serial.println("Unknown"); break;
   }
   }
   }
   
   Serial.println("\ndone");
   } 
   ========================================================================================================= */


  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu1.initialize();
  mpu2.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu1.testConnection() ? F("MPU6050 1 connection successful") : F("MPU6050 1 connection failed"));
  Serial.println(mpu2.testConnection() ? F("MPU6050 2 connection successful") : F("MPU6050 2 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus1 = mpu1.dmpInitialize();
  devStatus2 = mpu2.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus1 == 0 || devStatus2 == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu1.setDMPEnabled(true);
    mpu2.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus1 = mpu1.getIntStatus();
    mpuIntStatus2 = mpu2.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize1 = mpu1.dmpGetFIFOPacketSize();
    packetSize2 = mpu2.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus1);
    Serial.print(" ");
    Serial.print(devStatus2);
    Serial.println(F(")"));
  }
} 


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  //if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt){ // && fifoCount1 < packetSize1 && fifoCount2 < packetSize2) {
    //Serial.print("\nwaiting...\n");
  // other program behavior stuff here
  // .
  // .
  // .
  // if you are really paranoid you can frequently test in between other
  // stuff to see if mpuInterrupt is true, and if so, "break;" from the
  // while() loop to immediately process the MPU data
  // .
  // .
  // .
 // }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus1 = mpu1.getIntStatus();
  mpuIntStatus2 = mpu2.getIntStatus();

  // get current FIFO count
  fifoCount1 = mpu1.getFIFOCount();
  fifoCount2 = mpu2.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus1 & 0x10) || (mpuIntStatus2 & 0x10) || fifoCount1 == 1024 || fifoCount2 == 1024) {
    // reset so we can continue cleanly
    mpu1.resetFIFO();
    mpu2.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if ((mpuIntStatus1 & 0x02) || (mpuIntStatus2 & 0x02)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount1 < packetSize1 && fifoCount2 < packetSize2)
    {
      fifoCount1 = mpu1.getFIFOCount();
      fifoCount2 = mpu2.getFIFOCount();
    }

    // read a packet from FIFO
    mpu1.getFIFOBytes(fifoBuffer1, packetSize1);
    mpu2.getFIFOBytes(fifoBuffer2, packetSize2);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount1 -= packetSize1;
    fifoCount2 -= packetSize2;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
    mpu1.dmpGetGravity(&gravity, &q1);
    mpu1.dmpGetYawPitchRoll(ypr, &q1, &gravity);
    num++;
    /*Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");*/
    Serial.println(ypr[2] * 180/M_PI);
    int count1=(count+25)%50;
    if (1.0*arr[count1][2]-20>ypr[2]*180/M_PI){
      int x=arr[count1][2];
      
      if (num-20>big){
        
        //if (ypr[2]*180/M_PI+10<x){
        bool yoy=true;
        for (int i=0;i<49;i++){
          //Serial.println(arr[i][2]);
          if (abs(arr[i+1][2]-arr[i][2])>5){
            yoy=false;
          }
        }
        if (yoy){
          Serial.println(arr[count][2]);
          Serial.println(x);
          Serial.println(ypr[2]*180/M_PI);
          for (int kek=0;kek<50;kek++){
            Serial.println(arr[kek][2]);
          }
          if (arr[count][2]+20<ypr[2]*180/M_PI){
            Serial.write("EEEEEEEEE");
          }
          //else if (ypr[2]*180/M_PI-10>x){
          else if (arr[count][2]-20>ypr[2]*180/M_PI){
            Serial.write("CCCCCCCC");
          }
          else{
            Serial.write("DDDDDDDD");
          }
        big=num;
        }
      }
    }
    arr[count][0]=1.0*ypr[0]*180/M_PI;
    
    arr[count][1]=1.0*ypr[1]*180/M_PI;
    arr[count][2]=1.0*ypr[2]*180/M_PI;
    
    /*Serial.print(arr[0][2]);
    Serial.print(arr[1][2]);
    Serial.print(arr[2][2]);
    Serial.print(arr[3][2]);
    Serial.print(arr[4][2]);
    Serial.print(ypr[0]*180/M_PI);
    Serial.print(ypr[1]*180/M_PI);
    Serial.println(ypr[2]*180/M_PI);*/
    
    count+=1;
    if (count==50){
      count=0;
    }
    mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
    mpu2.dmpGetGravity(&gravity, &q2);
    mpu2.dmpGetYawPitchRoll(ypr, &q2, &gravity);
    /*Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);*/
    /*mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
    Serial.print("a2 1 0 0 0 ");
    Serial.print(q1.w);
    Serial.print(" ");
    Serial.print(q1.x);
    Serial.print(" ");
    Serial.print(q1.y);
    Serial.print(" ");
    Serial.print(q1.z);
    Serial.print("\n");
    mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
    Serial.print("2 0 0 0 ");
    Serial.print(q2.w);
    Serial.print(" ");
    Serial.print(q2.x);
    Serial.print(" ");
    Serial.print(q2.y);
    Serial.print(" ");
    Serial.print(q2.z);
    Serial.println("b");
    Serial.print("\n");*/
#endif
  }
}

