#include <ros.h>
#include <std_msgs/Int16.h>


int r = 0.1;
int cpr = 4 * 512;
int count = 0;
int prevState = 0, currState = 0;
int prevTime, currTime;
int totalDis,prevDis =0;
float kp = 0.2, ki = 0.3, kd = 0.3;
float prevError = 0;
ros::NodeHandle nh;
std_msgs::Int16 Vel;
ros::Publisher chatter("chatter", &Vel);


void ISRA() {
  if (digitalRead(PB15) != digitalRead(PB14))
    count++;
  else
    count--;
}

void ISRB() {
  if (digitalRead(PB15) == digitalRead(PB14))
    count --;
  else
    count ++;
}

int disCalc(int count) {
  float distance = 2 * 3.14 * r * count / cpr;
  return distance;
}

void motorSet(int dir, int PWM, int pin, int CW, int CCW) {

}


void setup() {
  nh.initNode();
  nh.advertise(chatter);

  
  pinMode(PB15, INPUT_PULLUP);
  pinMode(PB14, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PB15), ISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PB14), ISRB, CHANGE);
  prevState = digitalRead(PB15);
  prevTime = millis();
}

void loop() {
  currState = digitalRead(PB15);
  currTime = millis();
  if (prevState != currState) {
    float error = (float)(totalDis - disCalc(count));
    float dt = currTime - prevTime;
    float errInteg;
    errInteg += error * dt;
    float errDrev = (error - prevError) / dt;
    float u = kp * error + ki * errInteg + kd * errDrev;
    prevError = error;
    int dir;
    if (u > 0)
      dir = 1;
    else
      dir = -1;

    int PWM = (int)fabs(u);
    if (PWM > 255)
      PWM = 255;
    motorSet(dir,PWM,PA0,PA1,PA2);

    int velocity = (int)((disCalc(count) - prevDis));
    Vel.data = velocity;
    chatter.publish(&Vel);
    nh.spinOnce();
    (nh.getHardware())->setPort(&Serial1);
    (nh.getHardware())->setBaud(115200);



    prevDis = disCalc(count);


  }


  prevState = currState;
  prevTime = currTime;
}
