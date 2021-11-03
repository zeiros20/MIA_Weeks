#include <ArduinoTcpHardware.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int16.h>
int count = 0;
ros::NodeHandle nh;
std_msgs::Int16 counter;
ros::Publisher chatter("chatter", &counter);

int currState;
int prevState;
int prevTime, currTime;

void setup() {
  pinMode(PB_14, INPUT);
  pinMode(PB_15, INPUT);
  prevState = digitalRead(PB_15);
  prevTime = millis();


}

void loop() {
  currState = digitalRead(PB_15);

  if (currState != prevState) {
    if (digitalRead(PB_14) != currState) {
      count--;
    }
    else {
      count++;
    }
  }
  currTime = millis();
  if (currTime - prevTime == 3000) {
    counter.data = count;
    chatter.publish(&counter);
    nh.spinOnce();
    prevState = currState;
    count =0;
    prevTime = currTime;
  }

}
