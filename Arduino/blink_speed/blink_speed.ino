#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;

float vx = 0;
float vy = 0;
float vz = 0;

uint16_t time_delay = 5000;
uint16_t tick_count = 0;
bool start_blink = false;

void speedCb( const std_msgs::UInt16& speed_msg ){
  uint16_t speed_input = speed_msg.data;
  if (time_delay != speed_input){
    tick_count = 0;
    if(speed_input > 0){
      start_blink = true;
    }else{
      start_blink = false;
    }
    time_delay = speed_input;
    if (time_delay < 10)
    {
      time_delay = 10;
    }  
  }
}

void droneInputCb( const geometry_msgs::Vector3& drone_input ){
  vx = drone_input.x;
  vy = drone_input.y;
  vz = drone_input.z;
}

ros::Subscriber<std_msgs::UInt16> speedSub("toggle_speed", speedCb );
ros::Subscriber<geometry_msgs::Vector3> droneInputSub("control/drone_input", droneInputCb );

geometry_msgs::Vector3 drone_speed_msg;
ros::Publisher droneSpeed("drone_speed", &drone_speed_msg);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(speedSub);
  nh.subscribe(droneInputSub);
}

void loop()
{
  if(start_blink)
  {
    tick_count += 100;
    if (tick_count >= time_delay){
      digitalWrite(13, HIGH-digitalRead(13));   // blink the led
      tick_count = 0;  
    }
  }
  else
  {
    digitalWrite(13, LOW);  //turn off led
  }
  drone_speed_msg.x = vx;
  drone_speed_msg.y = vy;
  drone_speed_msg.z = vz;
  droneSpeed.publish( &drone_speed_msg );
  nh.spinOnce();
  delay(100);
}
