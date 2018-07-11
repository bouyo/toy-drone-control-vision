#include <ros.h>
#include <geometry_msgs/Vector3.h>

//    VARIABLES
// 0 - 255 values for clock time
uint8_t sample_time = 50; // Ts = 0.05s

float vx = 92.0;
float vy = 92.0;
float vyaw = 92.0;
float vz = 179.0;

// digital pins for drone control
uint8_t pin_thrust = 3;
uint8_t pin_yaw = 9;
uint8_t pin_x = 10;
uint8_t pin_y = 11;

geometry_msgs::Vector3 drone_speed_msg;

ros::NodeHandle  nh;

// Subscriber callback function
void droneInputCb( const geometry_msgs::Vector3& control_msg ){
  vx = control_msg.x;
  vy = control_msg.y;
  vz = control_msg.z;
}

// pubs and subs
ros::Publisher droneSpeed("drone_speed", &drone_speed_msg);
ros::Subscriber<geometry_msgs::Vector3> droneInputSub("control/drone_input", droneInputCb );

void setup() {

  pinMode(pin_thrust, OUTPUT);
  pinMode(pin_yaw, OUTPUT);
  pinMode(pin_x, OUTPUT);
  pinMode(pin_y, OUTPUT);
  TCCR1B = (TCCR1B & 0b11111000) | 0x02;
  TCCR2B = (TCCR2B & 0b11111000) | 0x02;
  nh.initNode();
  nh.advertise(droneSpeed);
  nh.subscribe(droneInputSub);
}

void loop() {

  drone_speed_msg.x = vx;
  drone_speed_msg.y = vy;
  drone_speed_msg.z = vz;
  droneSpeed.publish( &drone_speed_msg );
  nh.spinOnce();
  analogWrite(pin_thrust, (int)vz);
  analogWrite(pin_x, (int)vx);
  analogWrite(pin_y, (int)vy);
  analogWrite(pin_yaw, (int)vyaw);
  delay(sample_time);  
}
