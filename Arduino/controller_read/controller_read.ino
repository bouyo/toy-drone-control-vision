int read_thrust = 0;
int read_yaw = 1;
int read_x = 2;
int read_y = 3;

int thrust = 0;
int yaw = 0;
int x = 0;
int y = 0;
// digital pins for drone control
uint8_t pin_thrust = 3;
uint8_t pin_yaw = 9;
uint8_t pin_x = 10;
uint8_t pin_y = 11;
int controlbuffer[4] = {255,89,89,89}; //starting control values
void setup() {
  // put your setup code here, to run once:
  pinMode(pin_thrust, OUTPUT);
  pinMode(pin_yaw, OUTPUT);
  pinMode(pin_x, OUTPUT);
  pinMode(pin_y, OUTPUT);
  
  pinMode(read_thrust, INPUT);
  pinMode(read_yaw, INPUT);
  pinMode(read_x, INPUT);
  pinMode(read_y, INPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  controlbuffer[0] = analogRead(read_thrust);
  controlbuffer[1] = analogRead(read_yaw);
  controlbuffer[2] = analogRead(read_x);
  controlbuffer[3] = analogRead(read_y);

  thrust = map(controlbuffer[0]-357,0,1023,0,255) + 89;
  x = map(controlbuffer[2]-357,0,1023,0,255);
  y = map(controlbuffer[3]-357,0,1023,0,255);
  analogWrite(pin_thrust,thrust);
  analogWrite(pin_yaw, 89);
  analogWrite(pin_x,x);
  analogWrite(pin_y,y);

  Serial.println("Read: ");
  Serial.println("  thrust: " + String(thrust));
  Serial.println("  yaw: " + String(yaw));
  Serial.println("  x: " + String(x));
  Serial.println("  y: " + String(y));
}
