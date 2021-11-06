#include <ros.h>
#include <std_msgs/Int16.h>

int count = 0;
int prevState=0,currState=0;
ros::NodeHandle nh;
std_msgs::Int16 counter;
ros::Publisher chatter("chatter", &counter);


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


void setup() {
  pinMode(PB15, INPUT_PULLUP);
  pinMode(PB14, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PB15),ISRA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PB14),ISRB,CHANGE);
  prevState = digitalRead(PB15);
}

void loop() {
  currState = digitalRead(PB15);

  if(currState != prevState && currState == 1){
    counter.data = count;
    chatter.publish(&counter);
    nh.spinOnce();
  }
  prevState = currState;
}
