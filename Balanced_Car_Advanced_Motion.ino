#include "Motor.h"
#include "Balanced.h"
#include <Adafruit_NeoPixel.h>

Timer2 Timer2;
extern Mpu6050 Mpu6050;
extern Motor Motor;
extern Balanced Balanced;


void setup() 
{
  Motor.Pin_init();
  Motor.Encoder_init();
  Timer2.init(TIMER);
  Mpu6050.init();
  Balanced.initLED();
  Serial.begin(9600);

 
float tile = 17.0; //floor tile length

  

  Balanced.Stop(); 
  delay(5000);

  // Balanced.ForwardDist(20);
  // Balanced.RightAngle(180);
  // Balanced.ForwardDist(20);
  // Balanced.Stop();
  // Balanced.RightAngle(90);
  // Balanced.ForwardDist(20);
  // Balanced.RightAngle(90);
  // Balanced.ForwardDist(20);
  // Balanced.RightAngle(90);
  // Balanced.ForwardDist(20);
  // Balanced.RightAngle(90);
  // Balanced.ForwardDist(20);
  // Balanced.RightAngle(90);
  // Balanced.Stop();
  // Balanced.ForwardDist(20);
  // Balanced.ReverseDist(20);
  // Balanced.Stop();
  // Balanced.LeftAngle(180);
  // Balanced.ForwardDist(20);
  // Balanced.RightAngle(180);

  Balanced.ForwardDist(3*tile);
  Balanced.LeftAngle(90);
  Balanced.ForwardDist(5*tile);
  Balanced.RightAngle(90);
  Balanced.ForwardDist(6*tile);
  Balanced.RightAngle(90);
  Balanced.ForwardDist(9*tile);
  Balanced.LeftAngle(90);
  Balanced.ForwardDist(3*tile);
  Balanced.Stop();


}
void loop() 
{
  
}

  

