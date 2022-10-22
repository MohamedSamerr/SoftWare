#include <ros.h>
#include <std_msgs/Int64.h>
#define outputA PA6
#define outputB PA7
long long counter=0;

ros::NodeHandle node;

std_msgs::Int64 msg;

ros::Publisher encoder("encoder", &msg);
HardwareSerial Serial3(PB11,PB10);
void setup() 
{
  Serial3.begin(115200);
  (node.getHardware())->setPort(&Serial3);
  (node.getHardware())->setBaud(115200);

  
  node.initNode();
  node.advertise(encoder);
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(outputA), interrupt_A , CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), interrupt_B , CHANGE);


}

void loop() 
{
  msg.data = counter ;
  encoder.publish( &msg );
  node.spinOnce();
  delay(10);


}
void interrupt_A (void)
{
  if(digitalRead(outputA)!=digitalRead(outputB))
  {
    counter++;
  }
  else
  {
    counter--;
  }
}
void interrupt_B (void)
{
  if(digitalRead(outputA)==digitalRead(outputB))
  {
    counter++;
  }
  else
  {
    counter--;
  }
}
