#include <ros.h>
#include <std_msgs/Int16.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

ros::NodeHandle nh;
std_msgs::Int16 read_msg;
ros::Publisher readings("topic", &read_msg);

HardwareSerial Serial3(PB11, PB10);

// class default I2C address is 0x68
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


#define LED_PIN PC13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

//Kalman filter variables
float x[3]; //state variable (estimation)
float p; //uncertainty (error in estimation)
float x_hat[3] = {0}; //predicted state (previous estimation)
float p_hat = 2; //prediction uncertainty (previous error in estimation)
float k; //kalman gain
float Q = 4; //process variance
float R = 4; //sensor variance(error in measurement)


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  nh.initNode();
  nh.advertise(readings);

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize serial communication
  Serial3.begin(115200);

  // initialize device
  mpu.initialize();


  // verify connection
  Serial3.println(F("Testing device connections..."));
  Serial3.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  // load and configure the DMP
  Serial3.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  // /if (devStatus == 0) {
  // Calibration Time: generate offsets and calibrate our MPU6050
  // mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  // turn on the DMP, now that it's ready
  Serial3.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);


  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  Serial3.println(F("DMP ready!"));
  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  ///    }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet


    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    int i;
    for (i = 0; i < 3; i++)
    {
      k = p_hat / (p_hat + R);
      x[i] = x_hat[i] + k * (ypr[i] - x_hat[i]);
      p = (1 - k) * p_hat;
    }
    for (i = 0; i < 3; i++) {
      x_hat[i] = x[i];
    }
    p_hat = p + Q;


    Serial3.print("ypr\t");
    Serial3.print(x[0] * 180 / M_PI);
    Serial3.print("\t");
    Serial3.print(x[1] * 180 / M_PI);
    Serial3.print("\t");
    Serial3.println(x[2] * 180 / M_PI);


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
