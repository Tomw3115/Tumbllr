#ifndef _BALANCED_h
#define _BALANCED_h

#include "MsTimer2.h"
#include "KalmanFilter.h"



enum Direction


{
  FORWARD,
  BACK,
  LEFT,
  RIGHT,
  STOP,
};

class Balanced
{
  public:
          Balanced();
          void Get_EncoderSpeed();
          void PD_VerticalRing();
          void SpeedRing();
          void SteeringRing();
          void Total_Control();
          void LeftAngle(int angle);
          void RightAngle(int angle);
          void ForwardDist(float dist);
          void ReverseDist(float dist);
          void BackDist(float Dist);
          void Motion_Control(Direction direction);
          void Stop();
          void Forward(int speed);
          void Back(int speed);
          void Left(int speed);
          void Right(int speed);
          void setBusy(bool state);
          bool getBusy();
          void initLED();


/*Speed value*/
          double pwm_left;
          double pwm_right;
          int encoder_left_pulse_num_speed;
          int encoder_right_pulse_num_speed;
          
/*Controlled motion */

          long turn_count_max = 0;
          long move_count_max = 0;
          long encoder_wheel_left;
          long encoder_wheel_right;
          long turn_count;
          long move_count;
          bool motor_busy;

/*Cnt*/
          int interrupt_cnt;

/*PID parameter*/
         /*PD_VerticalRing*/
          double kp_balance, kd_balance;
         /*PI_SpeedRing*/
          double kp_speed, ki_speed;
         /*PI_SteeringRing*/
          double kp_turn, kd_turn;

          double speed_filter;
          double speed_filter_old;
          double car_speed_integeral;
          double balance_control_output;
          double speed_control_output;
          double rotation_control_output;
          int setting_turn_speed;
          int setting_car_speed;
          
   private:
   #define ANGLE_MIN -27
   #define ANGLE_MAX 27
   #define EXCESSIVE_ANGLE_TILT (kalmanfilter.angle < ANGLE_MIN || ANGLE_MAX < kalmanfilter.angle)
   #define PICKED_UP (kalmanfilter.angle6 < -10 || 22 < kalmanfilter.angle6)

};

class Timer2
{
  public:
          void init(int time);
          static void interrupt();
  private:       
          #define TIMER 5
};


class Mpu6050
{
  public:
          void init();
          void DataProcessing();
          Mpu6050();

  public:
         int ax, ay, az, gx, gy, gz;
         float dt, Q_angle, Q_gyro, R_angle, C_0, K1;
};

class PID
{ public:
    PID(float kp,float ki, float kd);
    float computePID(float error);
  private:
    float _kp,_ki,_kd, _cum_error, _last_error;
};

#endif
