#include <ros.h>

#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include<std_msgs/Float32.h>

#include <I2Cdev.h>

ros::NodeHandle nh;
std_msgs:: Float32 ogYaw;
std_msgs:: Float32 filtYaw;
ros::Publisher OG("OG", &ogYaw);
ros::Publisher Filt("Filt", &filtYaw);

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 gy;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

float X = 0, Q = 2.0, Un = 7, R = 1.0;
float predX, predUn, K, Y;
int prevTime = 0;


void setup() {
  nh.initNode();
  nh.advertise(OG);
  nh.advertise(Filt);
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial1.begin(115200);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();



}

void loop() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial1.print("ypr\t");
    Serial1.print(ypr[0] * 180 / M_PI);
    Serial1.print("\t");
    Serial1.print(ypr[1] * 180 / M_PI);
    Serial1.print("\t");
    Serial1.println(ypr[2] * 180 / M_PI);
  }
  if (millis() - prevTime >= 5000)
    X = ypr[0];
  predX = X;
  predUn = Un + Q;
  Y = ypr[0] - predX;
  K = predUn / (predUn + R);
  X = predX + K * Y;
  Un = (1 - K) * predUn;
  ogYaw.data = ypr[0];
  filtYaw.data = X;
  OG.publish(&ogYaw);
  Filt.publish(&filtYaw);
  nh.spinOnce();
  (nh.getHardware())->setPort(&Serial1);
  (nh.getHardware())->setBaud(11520);



}
