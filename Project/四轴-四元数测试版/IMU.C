#include <STC15W4K60S4.H>
#include <IMU.H>
#include <math.H>
#define pi 3.14159265f                           
#define Kp 10.0f                        
#define Ki 0.008f                         
#define halfT 0.004f           
float idata q0=1,q1=0,q2=0,q3=0;   
float idata exInt=0,eyInt=0,ezInt=0;  
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float idata norm;
  float idata vx, vy, vz;
  float idata ex, ey, ez;

  float idata q0q0 = q0*q0;
  float idata q0q1 = q0*q1;
  float idata q0q2 = q0*q2;
  float idata q0q3 = q0*q3;
  float idata q1q1 = q1*q1;
  float idata q1q2 = q1*q2;
  float idata q1q3 = q1*q3;
  float idata q2q2 = q2*q2;
  float idata q2q3 = q2*q3;
  float idata q3q3 = q3*q3;

  norm = sqrt(ax*ax + ay*ay + az*az);      
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;
           
  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								 
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  gx = gx + Kp*ex + exInt;					   							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							
					   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  Angle=asin(2*(q0*q2-q1*q3 ))* 57.2957795f; // ¸©Ñö
  Angley=asin(2*(q0*q1+q2*q3 ))* 57.2957795f; // ºá¹ö
}
