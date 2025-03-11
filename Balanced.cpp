b// add library to control led lights
#include <Adafruit_NeoPixel.h>

#include "Balanced.h"
#include "Wire.h"
#include "Motor.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h"

MPU6050 MPU6050;
Mpu6050 Mpu6050;
Balanced Balanced;
KalmanFilter kalmanfilter;
Motor Motor;

// add PID controller for distance 
PID movePID(.05,0,0.15); 
//add light controller
Adafruit_NeoPixel lights = Adafruit_NeoPixel(4, 3, NEO_GRB + NEO_KHZ800);

// add boolean flag for motor status
bool motor_busy = false;
bool reverseFlag = false;

void Balanced::initLED()
{
  lights.begin();  // initialize the strip
  lights.show();   // make sure it is visible
  lights.clear();  // Initialize all pixels to 'off' 
}

void Timer2::init(int time)
{
  MsTimer2::set(time,interrupt);
  MsTimer2::start();
}

static void Timer2::interrupt()
{ 
  sei();//enable the global interrupt
  Balanced.Get_EncoderSpeed();
  Mpu6050.DataProcessing();
  Balanced.PD_VerticalRing();
  Balanced.interrupt_cnt++;
  if(Balanced.interrupt_cnt > 8)
    {
     Balanced.interrupt_cnt=0;
     Balanced.SpeedRing();
     Balanced.SteeringRing();
  }
  Balanced.Total_Control();
}

Balanced::Balanced()
{
  // set parameter values for Elegoo PID code
  kp_balance = 50, kd_balance = 0.75; //kp_balance = 55, kd_balance = 0.75;
  kp_speed = 10, ki_speed = 0.26;
  kp_turn = 2.5, kd_turn = 1.0; //0.5;
}

void Balanced::Total_Control()
{
  pwm_left = balance_control_output - speed_control_output - rotation_control_output;//Superposition of Vertical Velocity Steering Ring
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;//Superposition of Vertical Velocity Steering Ring

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

   while(EXCESSIVE_ANGLE_TILT || PICKED_UP)
  { 
    Mpu6050.DataProcessing();
    Motor.Stop();
  }
  
  (pwm_left < 0) ?  (Motor.Control(AIN1,1,PWMA_LEFT,-pwm_left)):
                    (Motor.Control(AIN1,0,PWMA_LEFT,pwm_left));
  
  (pwm_right < 0) ? (Motor.Control(BIN1,1,PWMB_RIGHT,-pwm_right)): 
                    (Motor.Control(BIN1,0,PWMB_RIGHT,pwm_right));
}

void Balanced::Get_EncoderSpeed()
{
  encoder_left_pulse_num_speed += pwm_left < 0 ? (-Motor::encoder_count_left_a) : 
                                                  Motor::encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? (-Motor::encoder_count_right_a) :
                                                  Motor::encoder_count_right_a;
  // add new counts to left and right totals for controlled motion
  encoder_wheel_left += pwm_left < 0 ? (-Motor::encoder_count_left_a) : 
                                                  Motor::encoder_count_left_a;
  encoder_wheel_right += pwm_right < 0 ? (-Motor::encoder_count_right_a) :
                                                  Motor::encoder_count_right_a;;
  // reset encoders for next time increment
  Motor::encoder_count_left_a=0;
  Motor::encoder_count_right_a=0;
}

void Balanced::Motion_Control(Direction direction)
{
  switch(direction)
  {
    case STOP:
                  Stop();break;
    case FORWARD:
                  Forward(40);break;
    case BACK:
                  Back(40);break;
    case LEFT:
                  Left(50);break;
    case RIGHT:
                  Right(50);break;
    default:      
                  Stop();break;
  }
}

void Balanced::Stop()
{
  while (getBusy()){delay(100);}
  delay(3000); // wait for 3 seconds after last controlled motion completed (for stability)
  setting_car_speed = 0;
  setting_turn_speed = 0;
}

void Balanced::Forward(int speed)
{
  setting_car_speed = speed;
  setting_turn_speed = 0;
}

void Balanced::Back(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = 0;
}

void Balanced::Left(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = speed/2;
}

void Balanced::Right(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = -speed/2;
}

void Balanced::SpeedRing()
{ 
  float ctrlSpeed;
  float dist,oldDist;
  if (move_count_max > 0) // check to see if controlled motion is in progress
    {
      move_count =  abs(encoder_wheel_left + encoder_wheel_right)*0.5;
      dist = (move_count_max - move_count);
      if ((0.5*(abs(dist)+abs(oldDist))) < 5) // check to see if distanced moved is within tolerance
        {
          // motion is complete
          ctrlSpeed = 0;
          reverseFlag = false;
          setBusy(false);
        }
      else 
        {        
        ctrlSpeed = movePID.computePID(dist); // use new distance PID to control speed
        if (reverseFlag) {ctrlSpeed *= -1;}
        ctrlSpeed = constrain(ctrlSpeed,-setting_car_speed,+setting_car_speed); // control top speed
        oldDist = dist;
        } 
    }
    else {ctrlSpeed = setting_car_speed;} 
    
  // perform normal calculations next
    double car_speed=(encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    encoder_left_pulse_num_speed = 0;
    encoder_right_pulse_num_speed = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -ctrlSpeed; 
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    speed_control_output =  -kp_speed * speed_filter - ki_speed * car_speed_integeral; 
}

void Balanced::PD_VerticalRing()
{
  balance_control_output= kp_balance * (kalmanfilter.angle - 0) + kd_balance * (kalmanfilter.Gyro_x - 0);
}

void Balanced::SteeringRing()
{ 
  float turnTotal,turnSpeed;
  
   
  if (turn_count_max > 0) // check to see if controlled turn is in progress
    {
    turn_count =  encoder_wheel_left - encoder_wheel_right;
    if (abs(turn_count) > turn_count_max) // check to see if turn limit is reached
      {
      setting_turn_speed = 0; //stop turning
      turn_count_max = 0; // reset turn limit
      setBusy(false); // signal turn is done
      encoder_wheel_left = 0;
      encoder_wheel_right = 0;
      } 
    rotation_control_output = setting_turn_speed;
    
  }
  else
  {
    // set normal value for rotation_control_output 
    rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;////control with Z-axis gyroscope   
  }
}

void Mpu6050::init()
{
   Wire.begin();         
   MPU6050.initialize();    
 }

Mpu6050::Mpu6050()
{
    dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
}

void Mpu6050::DataProcessing()
{  
  MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);// Data acquisition of MPU6050 gyroscope and accelerometer
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter
}

void Balanced::LeftAngle(int angle)
{
  while (getBusy()){delay(100);}
  delay(3000); // wait for 3 seconds after last controlled motion completed (for stability)
  setBusy(true);
  int speed = 100;
  setting_car_speed = 0;
  setting_turn_speed = +speed/2;
  // reset encoder wheel counts and calculate new counter turn limit 
  encoder_wheel_left = 0;
  encoder_wheel_right = 0;
  turn_count_max = angle * PI*TRACK_WIDTH/180; //distance travelled by each wheel
  turn_count_max *= (GEAR_RATIO * ENCODER_PULSE_COUNT) / (WHEEL_DIA * PI); // encoder pulse count
}

void Balanced::RightAngle(int angle)
{
  while (getBusy()){delay(100);}
  delay(3000); // wait for 3 seconds after last controlled motion completed (for stability)
  setBusy(true);
  int speed = 100;
  setting_car_speed = 0;
  setting_turn_speed = -speed/2;
  // reset encoder wheel counts and calculate new counter turn limit 
  encoder_wheel_left = 0;
  encoder_wheel_right = 0;
  turn_count_max = angle * PI*TRACK_WIDTH/180; //distance travelled by each wheel
  turn_count_max *= (GEAR_RATIO * ENCODER_PULSE_COUNT) / (WHEEL_DIA * PI); // encoder pulse count
}

void Balanced::ForwardDist(float dist)
{
  while (getBusy()){delay(100);}
  delay(3000); // wait for 3 seconds after last controlled motion completed (for stability)
  setBusy(true);
  setting_car_speed = 50;
  setting_turn_speed = 0;
  // reset encoder wheel counts and calculate new motion turn limit 
  encoder_wheel_left = 0;
  encoder_wheel_right = 0;
  move_count_max = dist;  //distance travelled by each wheel(inches)
  move_count_max *= (GEAR_RATIO * ENCODER_PULSE_COUNT) / (WHEEL_DIA * PI); // encoder pulse count
}

void Balanced::ReverseDist(float dist)
{
  while (getBusy()){delay(100);}
  delay(3000); // wait for 3 seconds after last controlled motion completed (for stability)
  setBusy(true);
  reverseFlag = true;
  setting_car_speed = 50;
  setting_turn_speed = 0;
  // reset encoder wheel counts and calculate new motion turn limit 
  encoder_wheel_left = 0;
  encoder_wheel_right = 0;
  move_count_max = dist;  //distance travelled by each wheel(inches)
  move_count_max *= (GEAR_RATIO * ENCODER_PULSE_COUNT) / (WHEEL_DIA * PI); // encoder pulse count
}

void Balanced::setBusy(bool state)
  {
    motor_busy = state;
    if (state) // code below turns LEDs on green
    {
      for( int i = 0; i < 4; i++ )
      {
        lights.setPixelColor(i, 0, 100, 0);
      }
      lights.show();
      delay(10);
       
    }
    else // code below tunrs LEDs off
    {
      for( int i = 0; i < 4; i++ )
      {
        lights.setPixelColor(i, 0, 0, 0);
      }
      lights.show();
      delay(10);
       
    }


  }

bool Balanced::getBusy()
  {
    return motor_busy;
  }

// add definition for PID object
PID::PID(float kp, float ki, float kd)
  {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _cum_error = 0;
    _last_error = 0;

  }

// add function to update PID value
float PID::computePID(float error)
  { 
    _cum_error += error;                // compute integral
    float rate_error = (error - _last_error);  // compute derivative
    float out = _kp*error + _ki*_cum_error + _kd*rate_error;                //PID output               
    _last_error = error;    //remember current error           
    return out;   //have function return the PID output
  }