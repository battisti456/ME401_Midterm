#ifndef PIDMOTOR
#define PIDMOTOR

#include <SoftPWMServo.h>
#include <PID_v1.h>

#define PIN_A 2
#define PIN_B 20
#define DIR_PIN 4
#define PWM_PIN 3
#define COUNTS_PER_REVOLUTION 2800
#define KP 1104
#define KI 8117
#define KD 37

#define PIDMOTOR_POSITION_UPDATE_MS 1
#define PIDMOTOR_PID_UPDATE_MS 100

class PIDMotor {
private:
    double input = 0;
    double output = 0;
    double setpoint = 0;

    double r_per_count = 0;

    volatile char last_A;
    volatile char last_B;

    volatile int position = 0;
 
    PID pid = PID(&input,&output,&setpoint,1,0,0,DIRECT);
public:
    void setup();
    void update_position();
    void update_PID();
    void update(int ms);

    void set_tunings(double p, double i, double d);

    double get_angle() const;
    double set_angle(double a);

    void set_position(double a);
};

void PIDMotor::setup() {
  pinMode(PIN_A,INPUT);
  pinMode(PIN_B,INPUT);
  last_A = digitalRead(PIN_A);
  last_B = digitalRead(PIN_B);
  pinMode(PWM_PIN,OUTPUT);
  pinMode(DIR_PIN,OUTPUT);
  digitalWrite(PWM_PIN,0);
  digitalWrite(DIR_PIN,0);
  SoftPWMServoPWMWrite(PWM_PIN,0);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(PIDMOTOR_PID_UPDATE_MS);
  pid.SetOutputLimits(-255,255);
  
  set_tunings(KP,KI,KD);
  r_per_count = 2*3.141592653/COUNTS_PER_REVOLUTION;
}
void PIDMotor::set_tunings(double p, double i, double d){
    pid.SetTunings(p,i,d);
}
void PIDMotor::update(int ms) {
  
  if(ms % PIDMOTOR_POSITION_UPDATE_MS == 0) {
    update_position();
  }
  if(ms % PIDMOTOR_PID_UPDATE_MS ==0) {
    update_PID();
  }
  
}
void PIDMotor::update_position() {
    char new_A = digitalRead(PIN_A);
    char new_B = digitalRead(PIN_B);

    position += (new_A^last_B)-(last_A^new_B);

    last_A = new_A;
    last_B = new_B;
}
void PIDMotor::update_PID() {
    input = get_angle();
    pid.Compute();
    digitalWrite(DIR_PIN,output > 0);
    SoftPWMServoPWMWrite(PWM_PIN,abs(output));
}
double PIDMotor::get_angle() const {
    return r_per_count*position;
}
double PIDMotor::set_angle(double a) {
    setpoint = a;
}
void PIDMotor::set_position(double a){
  position = (int) a/r_per_count;
}

#endif
