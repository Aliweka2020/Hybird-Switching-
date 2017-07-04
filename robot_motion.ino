/*
  vmax = 95 pulses per 200 ms = 23.75 pulses per 50ms
  vmax = 2*pi*R*(23/420) cm per 50ms
  2*pi*R = 31, L = 35
  vmax = 31*(23.75/420) = 1.75 cm per 50ms
  v = (R/2)(vr+vl)
  w = (R/L)(vr-vl)
  wheel_radius = R = 31/(2*3.1418);
  v_max = (1/2)(vr+vl)
  w_max = (1/L)(vr-vl) = (1/L)((1.75 per 50ms) - 0)
  = (1.75 per 50ms)/35 = 0.05
*/

//old robot
/* L=8  R=6.5/2
  2*pi*R= 20.4*/

//new robot
//L=30 R=7.5  2*pi*R=47.1
double v_max = 0;
double w_max = 0;

#include <math.h>
#include <Timer.h>

Timer t;

//Define Variables we'll be connecting to
double motor_output_R;  //PWM right
double motor_output_L; //pwm left


#define motorR 10
#define motorL 11

void get_x_y_new_direct();
void go_to_goal();
void go_to_goal_ao();
void unicycle_to_diff_drive();
void output_shaping();

//The sample code for driving one way motor encoder
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 4;//B pin -> the digital pin 4
byte encoder0PinALast;
int duration;//the number of the pulses
boolean Direction;//the rotation direction
//The sample code for driving one way motor encoder
const byte encoder1pinA = 3;//A pin -> the interrupt pin 0
const byte encoder1pinB = 5;//B pin -> the digital pin 4
byte encoder1PinALast;
int duration2;//the number of the pulses
boolean Direction2;//the rotation direction
void EncoderInit();
void wheelSpeed();
void wheelSpeed2();
void print_every_500();
volatile long encoderR = 0, encoderL = 0;
unsigned long encoderR_old = 0, encoderL_old = 0;
int encoderR_dif = 0, encoderL_dif = 0;

char incomingByte; // for incoming serial data

double diff_x, diff_y, distance_error, theta_goal, theta_error , theta_g_o , theta_switch;
double Prop_error, Int_error, Dif_error, PID_output;
double omega, velocity, vel_r, vel_l, wheel_radius;
double vel_r_in_pulses, vel_l_in_pulses, vel_rl_max, vel_rl_min;
double vel_Kp, vel_PID_output;

int get_x_y_new_Event;
int go_to_goal_Event;
int go_to_goal_ao_Event;
int myprint;

//positions of sensors
double xsr = 22, ysr = -15, thetasr = -3.1418 / 2;
double xsr60 = 30, ysr60 = -15, thetasr60 = -3.1418 / 4;
double xsf = 34, ysf = 0, thetasf = 0;
double xsl60 = 30, ysl60 = 15, thetasl60 = 3.1418 / 4;
double xsl = 22, ysl = 15, thetasl = 3.1418 / 2;

//varaibles for robot frame
double vsr60_1, vsr60_2, vsr60_3;
double vsr_1, vsr_2, vsr_3;
double vsf_1, vsf_2, vsf_3;
double vsl60_1, vsl60_2, vsl60_3;
double vsl_1, vsl_2, vsl_3;

//varaibles for world frame
double vssr60_1, vssr60_2, vssr60_3;
double vssr_1, vssr_2, vssr_3;
double vssf_1, vssf_2, vssf_3;
double vssl60_1, vssl60_2, vssl60_3;
double vssl_1, vssl_2, vssl_3;

//compute u
double usr60_1, usr60_2, usr60_3;
double usr_1, usr_2, usr_3;
double usf_1, usf_2, usf_3;
double usl60_1, usl60_2, usl60_3;
double usl_1, usl_2, usl_3;

//compute u_ao
double u_ao_1, u_ao_2;
double theta_ao;

//sensors
unsigned long echo = 0;
int ultraSoundSignal = 6; // Ultrasound signal pin
int echoSignal = 7; // echo signal pin
unsigned long ultrasoundValue = 0;

unsigned long echo2 = 0;
int ultraSoundSignal2 = 8; // Ultrasound signal pin
int echoSignal2 = 9; // echo signal pin
unsigned long ultrasoundValue2 = 0;

unsigned long echo3 = 0;
int ultraSoundSignal3 = A3; // Ultrasound signal pin
int echoSignal3 = A2; // echo signal pin
unsigned long ultrasoundValue3 = 0;

unsigned long echo4 = 0;
int ultraSoundSignal4 = A1; // Ultrasound signal pin
int echoSignal4 = A0; // echo signal pin
unsigned long ultrasoundValue4 = 0;

unsigned long echo5 = 0;
int ultraSoundSignal5 = A5; // Ultrasound signal pin
int echoSignal5 = A4; // echo signal pin
unsigned long ultrasoundValue5 = 0;

unsigned long dsr60, dsr, dsf, dsl60, dsl;

void EncoderInit()
{
  Direction = true;//default -> Forward
  Direction2 = true;//default -> Forward
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(0, wheelSpeed, CHANGE);
  pinMode(encoder1pinB, INPUT);
  attachInterrupt(1, wheelSpeed2, CHANGE);
}

void setup() {
  motor_output_R = 0;
  motor_output_L = 0;
  Serial.begin(57600);//Initialize the serial port
  EncoderInit();//Initialize the module

  pinMode(motorR, OUTPUT);
  analogWrite(motorR, motor_output_R);

  pinMode(motorL, OUTPUT);
  analogWrite(motorL, motor_output_L);
  pinMode(ultraSoundSignal, OUTPUT);
  pinMode(ultraSoundSignal2, OUTPUT);
  pinMode(ultraSoundSignal3, OUTPUT);
  pinMode(ultraSoundSignal4, OUTPUT);
  pinMode(ultraSoundSignal5, OUTPUT);
  //Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

unsigned long ping()
{
  pinMode(ultraSoundSignal, OUTPUT); // Switch signalpin to output
  digitalWrite(ultraSoundSignal, LOW); // Send low pulse
  delayMicroseconds(4); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignal, HIGH); // Send high pulse
  delayMicroseconds(10); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignal, LOW); // Holdoff

  pinMode(echoSignal, INPUT); // Switch signalpin to input
  digitalWrite(echoSignal, HIGH); // Turn on pullup resistor
  // please note that pulseIn has a 1sec timeout, which may
  // not be desirable. Depending on your sensor specs, you
  // can likely bound the time like this -- marcmerlin
  // echo = pulseIn(ultraSoundSignal, HIGH, 38000)
  echo = pulseIn(echoSignal, HIGH, 38000); //Listen for echo
  ultrasoundValue = (echo / 57); //convert to CM then to inches
  if (ultrasoundValue == 0)
    ultrasoundValue = 350;
  return ultrasoundValue;
}

unsigned long ping2()
{
  pinMode(ultraSoundSignal2, OUTPUT); // Switch signalpin to output
  digitalWrite(ultraSoundSignal2, LOW); // Send low pulse
  delayMicroseconds(4); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignal2, HIGH); // Send high pulse
  delayMicroseconds(10); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignal2, LOW); // Holdoff

  pinMode(echoSignal2, INPUT); // Switch signalpin to input
  digitalWrite(echoSignal2, HIGH); // Turn on pullup resistor
  // please note that pulseIn has a 1sec timeout, which may
  // not be desirable. Depending on your sensor specs, you
  // can likely bound the time like this -- marcmerlin
  // echo = pulseIn(ultraSoundSignal, HIGH, 38000)
  echo2 = pulseIn(echoSignal2, HIGH, 38000); //Listen for echo
  ultrasoundValue2 = (echo2 / 57); //convert to CM then to inches
  if (ultrasoundValue2 == 0)
    ultrasoundValue2 = 350;
  return ultrasoundValue2;
}

unsigned long ping3()
{
  pinMode(ultraSoundSignal3, OUTPUT); // Switch signalpin to output
  digitalWrite(ultraSoundSignal3, LOW); // Send low pulse
  delayMicroseconds(4); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignal3, HIGH); // Send high pulse
  delayMicroseconds(10); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignal3, LOW); // Holdoff

  pinMode(echoSignal3, INPUT); // Switch signalpin to input
  digitalWrite(echoSignal3, HIGH); // Turn on pullup resistor
  // please note that pulseIn has a 1sec timeout, which may
  // not be desirable. Depending on your sensor specs, you
  // can likely bound the time like this -- marcmerlin
  // echo = pulseIn(ultraSoundSignal, HIGH, 38000)
  echo3 = pulseIn(echoSignal3, HIGH, 38000); //Listen for echo
  ultrasoundValue3 = (echo3 / 57); //convert to CM then to inches
  if (ultrasoundValue3 == 0)
    ultrasoundValue3 = 350;
  return ultrasoundValue3;
}

unsigned long ping4()
{
  pinMode(ultraSoundSignal4, OUTPUT); // Switch signalpin to output
  digitalWrite(ultraSoundSignal4, LOW); // Send low pulse
  delayMicroseconds(4); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignal4, HIGH); // Send high pulse
  delayMicroseconds(10); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignal4, LOW); // Holdoff

  pinMode(echoSignal4, INPUT); // Switch signalpin to input
  digitalWrite(echoSignal4, HIGH); // Turn on pullup resistor
  // please note that pulseIn has a 1sec timeout, which may
  // not be desirable. Depending on your sensor specs, you
  // can likely bound the time like this -- marcmerlin
  // echo = pulseIn(ultraSoundSignal, HIGH, 38000)
  echo4 = pulseIn(echoSignal4, HIGH, 38000); //Listen for echo
  ultrasoundValue4 = (echo4 / 57); //convert to CM then to inches
  if (ultrasoundValue4 == 0)
    ultrasoundValue4 = 350;
  return ultrasoundValue4;
}

unsigned long ping5()
{
  pinMode(ultraSoundSignal5, OUTPUT); // Switch signalpin to output
  digitalWrite(ultraSoundSignal5, LOW); // Send low pulse
  delayMicroseconds(4); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignal5, HIGH); // Send high pulse
  delayMicroseconds(10); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignal5, LOW); // Holdoff

  pinMode(echoSignal5, INPUT); // Switch signalpin to input
  digitalWrite(echoSignal5, HIGH); // Turn on pullup resistor
  // please note that pulseIn has a 1sec timeout, which may
  // not be desirable. Depending on your sensor specs, you
  // can likely bound the time like this -- marcmerlin
  // echo = pulseIn(ultraSoundSignal, HIGH, 38000)
  echo5 = pulseIn(echoSignal5, HIGH, 38000); //Listen for echo
  ultrasoundValue5 = (echo5 / 57); //convert to CM then to inches
  if (ultrasoundValue5 == 0)
    ultrasoundValue5 = 350;
  return ultrasoundValue5;
}

double DR = 0, DL = 0;
double DC, delta_theta, radius, prev_theta, new_theta;
double x, y, x_new, y_new, x_goal, y_goal;
double Kp, Ki, Kd, prev_err, accum_err;
boolean has_reached_goal = false;
int flag = 1, d_at_obs=50,d_unsafe=40;
int pwm_max = 0;
double alpha = 0.1;//0.2;
void loop() {
  t.update();
  if (flag)
  {
    get_x_y_new_Event = t.every(50, get_x_y_new_direct);
    x_goal = 300;
    y_goal = 150;
    /*Kp = 0.05;
      Ki = 0.0001;
      Kd = 0.001;*/

    // Kp = 0.44;
    Kp = 0.44;
    v_max = 3.23;
    w_max = 0.69;
    velocity = 24.1;
    pwm_max = 50;

    Ki = 0;
    Kd = 0;
    prev_err = 0;
    accum_err = 0;
    has_reached_goal = false;
    go_to_goal_Event = t.every(250, go_to_goal);
    go_to_goal_ao_Event = t.every(100, go_to_goal_ao);

    //    analogWrite(motorR, 0);
    //    analogWrite(motorL, 0);
    // digitalWrite(13, HIGH);

    myprint = t.every(500, print_every_500);

    flag = 0;
  }

 
}
void go_to_goal_ao()
{
   dsr60 = 100;
  dsl60 = 100;
  dsf = 100;
  dsr = 100;
  dsl = 100;
      dsr60 = min(ping(), dsr60);
    dsl60 = min(ping2(), dsl60);
    dsf = min(ping3(), dsf);
    dsr = min(ping4(), dsr);
    dsl = min(ping5(), dsl);
//  for (int j = 0; j < 3; j++)
//  {
//    dsr60 = min(ping(), dsr60);
//    dsl60 = min(ping2(), dsl60);
//    dsf = min(ping3(), dsf);
//    dsr = min(ping4(), dsr);
//    dsl = min(ping5(), dsl);
//    delay(10);
//  }
  //  Serial.print("\t\tdsr: ");
  //  Serial.print(dsr);
  //  Serial.print("dsr60: ");
  //  Serial.print(dsr60);
  //  Serial.print("\t\tdsf: ");
  //  Serial.print(dsf);
  //  Serial.print("\t\tdsl60: ");
  //  Serial.println(dsl60);
  //  Serial.print("\t\tdsl: ");
  //  Serial.println(dsl);

  //robot frame transformation
  vsr_1 = cos(thetasr) * dsr + xsr; ///cos(60)
  vsr_2 = sin(thetasr) * dsr + ysr;
  vsr_3 = 1;

  vsr60_1 = cos(thetasr60) * dsr60 + xsr60; ///cos(60)
  vsr60_2 = sin(thetasr60) * dsr60 + ysr60;
  vsr60_3 = 1;
  //  Serial.print("vsr60_1: ");
  //  Serial.print(vsr60_1);
  //  Serial.print("\t\tvsr60_2: ");
  //  Serial.println(vsr60_2);
  vsf_1 = cos(thetasf) * dsf + xsf; ///cos(60)
  vsf_2 = sin(thetasf) * dsf + ysf;
  vsr60_3 = 1;
  //  Serial.print("vsf_1: ");
  //  Serial.print(vsf_1);
  //  Serial.print("\t\tvsf_2: ");
  //  Serial.println(vsf_2);
  vsl60_1 = cos(thetasl60) * dsl60 + xsl60; ///cos(60)
  vsl60_2 = sin(thetasl60) * dsl60 + ysl60;
  vsr60_3 = 1;
  //  Serial.print("vsl60_1: ");
  //  Serial.print(vsl60_1);
  //  Serial.print("\t\tvsl60_2: ");
  //  Serial.println(vsl60_2);
  vsl_1 = cos(thetasl) * dsl + xsl; ///cos(60)
  vsl_2 = sin(thetasl) * dsl + ysl;
  vsr_3 = 1;

  //world fram transformation
  vssr_1 = cos(prev_theta) * vsr_1 - sin(prev_theta) * vsr_2 + x;
  vssr_2 = sin(prev_theta) * vsr_1 + cos(prev_theta) * vsr_2 + y;
  vssr_3 = 1;

  vssr60_1 = cos(prev_theta) * vsr60_1 - sin(prev_theta) * vsr60_2 + x;
  vssr60_2 = sin(prev_theta) * vsr60_1 + cos(prev_theta) * vsr60_2 + y;
  vssr60_3 = 1;
  //  Serial.print("vssr60_1: ");
  //  Serial.print(vssr60_1);
  //  Serial.print("\t\tvssr60_2: ");
  //  Serial.println(vssr60_2);
  vssf_1 = cos(prev_theta) * vsf_1 - sin(prev_theta) * vsf_2 + x;
  vssf_2 = sin(prev_theta) * vsf_1 + cos(prev_theta) * vsf_2 + y;
  vssf_3 = 1;
  //  Serial.print("vssf_1: ");
  //  Serial.print(vssf_1);
  //  Serial.print("\t\tvssf_2: ");
  //  Serial.println(vssf_2);
  vssl60_1 = cos(prev_theta) * vsl60_1 - sin(prev_theta) * vsl60_2 + x;
  vssl60_2 = sin(prev_theta) * vsl60_1 + cos(prev_theta) * vsl60_2 + y;
  vssl60_3 = 1;
  //  Serial.print("vssl60_1: ");
  //  Serial.print(vssl60_1);
  //  Serial.print("\t\tvssl60_2: ");
  //  Serial.println(vssl60_2);

  vssl_1 = cos(prev_theta) * vsl_1 - sin(prev_theta) * vsl_2 + x;
  vssl_2 = sin(prev_theta) * vsl_1 + cos(prev_theta) * vsl_2 + y;
  vssl_3 = 1;
  //compute u
  usr_1 = vssr_1 - x;
  usr_2 = vssr_2 - y;

  usr60_1 = vssr60_1 - x;
  usr60_2 = vssr60_2 - y;
  //  Serial.print("usr60_1: ");
  //  Serial.print(vssl60_1);
  //  Serial.print("\t\tusr60_2: ");
  //  Serial.println(usr60_2);
  usf_1 = vssf_1 - x;
  usf_2 = vssf_2 - y;
  //  Serial.print("usf_1: ");
  //  Serial.print(usf_1);
  //  Serial.print("\t\tusf_2: ");
  //  Serial.println(usf_2);
  usl60_1 = vssl60_1 - x;
  usl60_2 = vssl60_2 - y;
  //  Serial.print("usl60_1: ");
  //  Serial.print(usl60_1);
  //  Serial.print("\t\tusl60_1: ");
  //  Serial.println(usl60_1);
  usl_1 = vssl_1 - x;
  usl_2 = vssl_2 - y;

  u_ao_1 = usr_1 + usr60_1 + usf_1 + usl60_1 + usl_1;
  u_ao_2 = usr_2 + usr60_2 + usf_2 + usl60_2 + usl_2;

  theta_ao = atan2(u_ao_2, u_ao_1);
  //  Serial.print("theta_ao: ");
  //  Serial.print(theta_ao * 180 / 3.1416);
  theta_g_o = alpha * theta_goal + (1 - alpha) * theta_ao;

//  //switch mode
//    if ((abs(diff_x) < 25) && (abs(diff_y) < 25))
//    {
//      // stop
//      analogWrite(motorR, 0);
//      analogWrite(motorL, 0);
//      t.stop(go_to_goal_Event);
//      t.stop(go_to_goal_ao_Event);
//      t.stop(get_x_y_new_Event);
//    }
     if ((dsr60 < d_at_obs) | (dsl60 < d_at_obs) | (dsf < d_at_obs) | (dsr < d_at_obs) | (dsl < d_at_obs))  // at obstacle
    {
      theta_switch = theta_g_o;
    }
    else if ((dsr60 < d_unsafe) | (dsl60 < d_unsafe) | (dsf < d_unsafe) | (dsr < d_unsafe) | (dsl < d_unsafe))  // un safe
    {
      theta_switch = theta_ao;
    }
    else if ((dsr60 > d_at_obs) && (dsl60 > d_at_obs) && (dsf > d_at_obs) && (dsr > d_at_obs) && (dsl > d_at_obs))  // obstacle clear
    {
      theta_switch = theta_goal;
    }

  theta_error = theta_switch - prev_theta;
  theta_error = atan2(sin(theta_error), cos(theta_error));
  //  Serial.print("\t\tprev_theta: ");
  //  Serial.print(prev_theta * 180 / 3.1416);
  //  Serial.print("\t\ttheta_error: ");
  //  Serial.println(theta_error * 180 / 3.1416);
  Prop_error = theta_error;
  Int_error = accum_err + theta_error;
  Dif_error = theta_error - prev_err;
  PID_output = Kp * Prop_error + Ki * Int_error + Kd * Dif_error;

  accum_err = Int_error;
  prev_err = theta_error;

  omega = PID_output;
  if (omega > w_max)
    omega = w_max;
  else if (omega < -w_max)
    omega = -w_max;

  unicycle_to_diff_drive();
  output_shaping();

  motor_output_R = map(vel_r * 100, 0, v_max * 100, 0, pwm_max);
  motor_output_L = map(vel_l * 100, 0, v_max * 100, 0, pwm_max);

  analogWrite(motorR, motor_output_R);
  analogWrite(motorL, motor_output_L);
}
void go_to_goal()
{
  // every 250 ms
  diff_x = x_goal - x;
  diff_y = y_goal - y;
  distance_error = sqrt((diff_x * diff_x) + (diff_y * diff_y));
  theta_goal = atan2(diff_y , diff_x); //0.785
  //  theta_error = theta_goal - prev_theta;
  //  theta_error = atan2(sin(theta_error), cos(theta_error));
  //  /* Serial.print("theta_error= ");
  //    Serial.print(theta_error);
  //    Serial.print("   theta_goal= ");
  //    Serial.println(theta_goal);*/
  //  //Kp = 0.05; Ki = 0.0001; Kd = 0.001;
  //  Prop_error = theta_error;
  //  Int_error = accum_err + theta_error;
  //  Dif_error = theta_error - prev_err;
  //  PID_output = Kp * Prop_error + Ki * Int_error + Kd * Dif_error;
  //
  //  accum_err = Int_error;
  //  prev_err = theta_error;
  //
  //  omega = PID_output;
  //  /*Serial.print("   omega= ");
  //    Serial.println(omega);*/
  //  if (omega > w_max)
  //    omega = w_max;
  //  else if (omega < -w_max)
  //    omega = -w_max;
  //
  //  unicycle_to_diff_drive();
  //  output_shaping();
  //
  //  motor_output_R = map(vel_r * 100, 0, v_max * 100, 0, pwm_max);
  //  motor_output_L = map(vel_l * 100, 0, v_max * 100, 0, pwm_max);
  //  /*Serial.print("motor_output_R= ");
  //    Serial.print(motor_output_R);
  //    Serial.print("   motor_output_L= ");
  //    Serial.println(motor_output_L);*/
  //  analogWrite(motorR, motor_output_R);
  //  analogWrite(motorL, motor_output_L);

  if ((abs(diff_x) < 25) && (abs(diff_y) < 25))
  {
    // stop
    analogWrite(motorR, 0);
    analogWrite(motorL, 0);
    t.stop(go_to_goal_Event);
    t.stop(go_to_goal_ao_Event);
    t.stop(get_x_y_new_Event);
  }
}

//  v = (R/2)(vr+vl)
//  w = (R/L)(vr-vl)

//    kp = 0.44;
//    v_max = 3.23;
//    w_max = 0.69;
//    velocity = 24.1;
//    pwm_max = 60;

//  at pwm==60 && sample==50ms
//  encoderR_dif = 42 from 600  >>  3.297 cm in 50 ms
//  encoderL_dif = 64 from 960  >>  3.14 cm in 50 ms
//  v_max = 3.2285*R = 24.13875
//  w_max = (7.5/34)*(3.14-0) = 0.69
//  vel_r, vel_l = (2 * 24.1 + 0)/(2*7.5) = 3.21 at W = zero
//  vel_r = (2 * 24.1 + 0.69*34)/(2*7.5) = 4.8 at W = w_max
//  vel_l = (2 * 24.1 - 0.69*34)/(2*7.5) = 1.65 at W = w_max
//  shaping
//  vel_rl_max = 4.8
//  vel_rl_min = 1.65
//  v_max = 3.23
//  vel_r = 4.8 - (4.8 - 3.23) = 3.23
//  vel_l = 1.65 - (4.8 - 3.23) = 0.08
//  kp = w_max/(pi/2) = 0.44

//with output shaping
void unicycle_to_diff_drive()
{
  // 2*pi*R=31
  wheel_radius = 47.1 / (2 * 3.1418); //31/(2*3.1418);
  vel_r = (2 * velocity + omega * 34) / (2 * wheel_radius); //vel_r = (2*velocity+omega*35)/(2*wheel_radius);
  vel_l = (2 * velocity - omega * 34) / (2 * wheel_radius); //vel_l = (2*velocity-omega*35)/(2*wheel_radius);
  /* Serial.print("   omega= ");
    Serial.println(o
    mega);
    Serial.print("vel_r= ");
    Serial.print(vel_r);
    Serial.print("   vel_l= ");
    Serial.println(vel_l);*/
}

void output_shaping()
{
  vel_rl_max = max(vel_r, vel_l);
  vel_rl_min = min(vel_r, vel_l);
  if (vel_rl_max > v_max)
  {
    vel_r = vel_r - (vel_rl_max - v_max);
    vel_l = vel_l - (vel_rl_max - v_max);
  }
  if (vel_rl_min < 0)
  {
    vel_r = vel_r + (0 - vel_rl_min);
    vel_l = vel_l + (0 - vel_rl_min);
  }
  else
  {
    vel_r = vel_r;
    vel_l = vel_l;
  }
}

void get_x_y_new_direct()
{
  // every 50 ms
  encoderR_dif = encoderR - encoderR_old;
  encoderR_old = encoderR;
  encoderL_dif = encoderL - encoderL_old;
  encoderL_old = encoderL;

  DR = 47.1 * encoderR_dif / 600;   // DR = 31.0 * encoderR_dif/420.0;
  DL = 47.1 * encoderL_dif / 960;    //DL = 31.0 * encoderL_dif/420.0;
  /*Serial.print("DR= ");
    Serial.print(DR);
    Serial.print("   DL= ");
    Serial.println(DL);*/
  DC = (DR + DL) / 2.0;
  delta_theta = (DR - DL) / 34; ///(DR-DL)
  /*Serial.print("   DC= ");
    Serial.println(DC);
    Serial.print("delta_theta= ");
    Serial.print(delta_theta);
    Serial.print("   prev_theta= ");
    Serial.println(prev_theta);*/
  x_new = x + DC * cos(prev_theta);
  y_new = y + DC * sin(prev_theta);
  new_theta = prev_theta + delta_theta;
  new_theta = atan2(sin(new_theta), cos(new_theta));
  //  Serial.print("x_new= ");
  //  Serial.print(x_new);
  //  Serial.print("   y_new= ");
  //  Serial.println(y_new);
  x = x_new;
  y = y_new;
  prev_theta = new_theta;
}

//void wheelSpeed()
//{
//  int Lstate = digitalRead(encoder0pinA);
//  if((encoder0PinALast == LOW) && Lstate==HIGH)
//  {
//    int val = digitalRead(encoder0pinB);
//    if(val == LOW && Direction)
//    {
//      Direction = false; //Reverse
//    }
//    else if(val == HIGH && !Direction)
//    {
//      Direction = true;  //Forward
//    }
//  }
//  encoder0PinALast = Lstate;
//
//  if(!Direction)  encoderR++;
//  else  encoderR--;
//}
//void wheelSpeed2()
//{
//  int Lstate = digitalRead(encoder1pinA);
//  if((encoder1PinALast == LOW) && Lstate==HIGH)
//  {
//    int val = digitalRead(encoder1pinB);
//    if(val == LOW && Direction2)
//    {
//      Direction2 = false; //Reverse
//    }
//    else if(val == HIGH && !Direction2)
//    {
//      Direction = true;  //Forward
//    }
//  }
//  encoder1PinALast = Lstate;
//
//  if(!Direction2)  encoderL++;
//  else  encoderL--;
//}
void print_every_500()
{
  //  Serial.print("encoderR: ");
  //  Serial.println(encoderR);
  //  Serial.print("encoderL: ");
  //  Serial.println(encoderL);

  //  Serial.print("encoderR_dif: ");
  //  Serial.print(encoderR_dif);
  //  Serial.print("   encoderL_dif: ");
  //  Serial.println(encoderL_dif);
  Serial.print("dsr: ");
  Serial.print(dsr);
  Serial.print("\t\tdsr60: ");
  Serial.print(dsr60);
  Serial.print("\t\tdsf: ");
  Serial.print(dsf);
  Serial.print("\t\tdsl60: ");
  Serial.print(dsl60);
  Serial.print("\t\tdsl: ");
  Serial.println(dsl);
  Serial.print("x_new: ");
  Serial.print(x_new);
  Serial.print("\t\ty_new: ");
  Serial.print(y_new);
  Serial.print("\t\ttheta_new: ");
  Serial.println(new_theta * 180 / 3.1418);
  Serial.print("theta_ao: ");
  Serial.print(theta_ao * 180 / 3.1416);
  //  Serial.print("\t\tprev_theta: ");
  //  Serial.print(prev_theta * 180 / 3.1416);
  //  Serial.print("\t\ttheta_error: ");
  //  Serial.println(theta_error * 180 / 3.1416);
  Serial.print("\t\ttheta_goal: ");
  Serial.print(theta_goal * 180 / 3.1416);
  Serial.print("\t\ttheta_g_o: ");
  Serial.print(theta_g_o * 180 / 3.1416);
  Serial.print("\t\ttheta_switch: ");
  Serial.print(theta_switch * 180 / 3.1416);
  Serial.println();
  //  Serial.print("theta_error: ");
  //  Serial.print(theta_error);
  //  Serial.print("   PID_output: ");
  //  Serial.print(PID_output);
  //  Serial.print("   omega: ");
  //  Serial.print(omega);
  //  Serial.print("   vel_r: ");
  //  Serial.print(vel_r);
  //  Serial.print("   vel_l: ");
  //  Serial.println(vel_l);
}

void wheelSpeed()
{
  if (digitalRead(encoder0pinA) == digitalRead(encoder0pinB)) {
    encoderR--;
  }
  else {
    encoderR++;
  }
}

void wheelSpeed2()
{
  if (digitalRead(encoder1pinA) == digitalRead(encoder1pinB)) {
    encoderL--;
  }
  else {
    encoderL++;
  }
}
