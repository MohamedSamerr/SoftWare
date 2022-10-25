




#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>





//ENCODER PINS
#define outputA 2
#define outputB 3
//CYTRON 
#define motor_pwm 5
#define motor_dir 6
//=====================
int Time_between_publishes = 10;
long long counter=0;


ros::NodeHandle node;
std_msgs::Int64 ENCODER;
ros::Publisher encoder("encoder", &ENCODER);






//=========================================================================================================
void pwm_input( const std_msgs::Int16& pwm_value)
{
  int pwm = 0;
  pwm = pwm_value.data;
  if ( pwm > 0 )  //posotive
  {
    analogWrite(motor_pwm,pwm);
    digitalWrite(motor_dir,HIGH);
    
  }
  else if (pwm < 0)           //negative
  {
    
    analogWrite(motor_pwm,fabs(pwm));
    digitalWrite(motor_dir,HIGH); 
  }
 
}
//========================================================================================================
ros::Subscriber<std_msgs::Int16> pwm("PWM_Values", &pwm_input);







//TTL PINS
//HardwareSerial Serial3(PB11,PB10);
//=====================================================================================
void setup() 
{
  //Serial3.begin(115200);
  //(node.getHardware())->setPort(&Serial3);
  //(node.getHardware())->setBaud(115200);

  
  node.initNode();
  node.advertise(encoder);
  node.subscribe(pwm);

  pinMode(motor_pwm,OUTPUT);
  pinMode(motor_dir,OUTPUT);
  
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(outputA), interrupt_A , CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), interrupt_B , CHANGE);
}
//=======================================================================================================
void loop() 
{
 
  ENCODER.data = counter ;
  encoder.publish( &ENCODER );
  node.spinOnce();
  delay(Time_between_publishes);
}
//========================================================================================================
//========================================================================================================
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
//=========================================================================================================
