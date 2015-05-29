#include <AFMotor.h>

//#define DEBUG

AF_DCMotor motor_right(4);
AF_DCMotor motor_left(3);

int motor_speed = 140;

int max_control = 110;
int sensor_prop = 3;  // Константа для нормализации значения отклонения от центра (отклонение / константа)

int sensor_left;
int sensor_left_center;
int sensor_right_center;
int sensor_right;

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  pinMode(A4, OUTPUT);
  digitalWrite(A4, LOW);
  
  motor_left.setSpeed(motor_speed);
  motor_right.setSpeed(motor_speed);
  motor_left.run(RELEASE);
  motor_right.run(RELEASE);
}

void readSensors() {
  digitalWrite(A4, HIGH);
  delay(3);
  sensor_left = analogRead(A0);
  sensor_left_center = analogRead(A1);
  sensor_right_center = analogRead(A2);
  sensor_right = analogRead(A3);
  digitalWrite(A4, LOW);
}

void printSensors() {
#ifdef DEBUG
  Serial.print(sensor_left);
  Serial.print(',');
  Serial.print(sensor_left_center);
  Serial.print(',');
  Serial.print(sensor_right_center);
  Serial.print(',');
  Serial.println(sensor_right);
#endif
}

void runMotors(int left, int right) {
  motor_left.setSpeed(abs(left));
  motor_right.setSpeed(abs(right));

  if (left > 0)
    motor_left.run(BACKWARD);
  else if (left < 0)
    motor_left.run(FORWARD);
  else
    motor_left.run(RELEASE);

  if (right > 0)
    motor_right.run(FORWARD);
  else if (right < 0)
    motor_right.run(BACKWARD);
  else
    motor_right.run(RELEASE);
}

void turn(int center_error, int last_error, int &motor_speed_left, int &motor_speed_right)
{
    if (abs(last_error) > 50)
    {
      motor_speed_left = motor_speed + last_error;
      motor_speed_right = motor_speed - last_error;
    }
    else
    {
      motor_speed_left = motor_speed + center_error;
      motor_speed_right = motor_speed - center_error;
    }
}

void correct(int &center_error, int &last_error, int sensor_prop)
{
    center_error /= sensor_prop;
    last_error /= sensor_prop;
    
    center_error = center_error > max_control ? max_control : center_error;
    center_error = center_error < (-max_control) ? (-max_control) : center_error;
    
    last_error = last_error > max_control ? max_control : last_error;
    last_error = last_error < (-max_control) ? (-max_control) : last_error;
}

void loop() {
  readSensors();
  printSensors();  

  int center_error = 0;      // Отклонение внутренних датчиков от центра
  int last_error = 0;        // Отклонение внешних датчиков от центра
  int motor_speed_left = 0;  // Скорость левого колеса
  int motor_speed_right = 0; // Скорость правого колеса
  
  if (sensor_left < 100 and sensor_right < 100)
  {   
    center_error = sensor_left_center - sensor_right_center;
    last_error = sensor_left - sensor_right;
    
    correct(center_error, last_error, sensor_prop)
    
    turn(center_error, last_error, motor_speed_left, motor_speed_right);
  }
  else
  {
    center_error = sensor_right_center - sensor_left_center;
    last_error = sensor_right - sensor_left;
    
    correct(center_error, last_error, sensor_prop)
    
    turn(center_error, last_error, motor_speed_left, motor_speed_right);
  }
  
  runMotors(motor_speed_left, motor_speed_right);

  delay(50);
}
