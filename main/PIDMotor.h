#include <SoftPWMServo.h>
#include <PID_v1.h>
#include "config.h"

class PIDMotor {
private:
    int A_pin = 0;
    int B_pin = 0;
    int dir_pin = 0;
    int pwm_pin = 0;

    double kp = 0;
    double ki = 0;
    double kd = 0;

    double input = 0;
    double output = 0;
    double setpoint = 0;

    double r_per_count = 0;

    static volatile char last_A;
    static volatile char last_B;

    volatile double position;

    PID pid(&input,&output,&setpoint,1,0,0,DIRECT);
public:
    void setup(double p, double i, double d, int A, int B, int dir, int pwm, int rev_count);
    void update_position();
    void update_PID();

    void set_PID(double p, double i, double d);

    double get_angle() const;
    double set_angle(double a);
}

void PIDMotor::setup(double p, double i, double d, int A, int B, int dir, int pwm, int rev_count) {
    set_tunings(p,i,d);
    A_pin = A;
    B_pin = B;
    dir_pin = dir;
    pwm_pin = pwm;
    r_per_count = 2*3.141592653/rev_count;
}
void PIDMotor::set_tunings(double p, double i, double d){
    kp = p;
    ki = i;
    kd = d;
    pid.SetTunings(kp,ki,kd);
}
void PIDMotor::update_position() {
    char new_A = digitalRead(A_pin);
    char new_B = digitalRead(B_pin);

    position += (new_A^last_B)-(last_A^new_B);

    last_A = new_A;
    last_B = new_B;
}
void PIDMotor::update_PID() {
    input = get_angle();
    pid.Compute();
    digitalWrite(dir_pin,output < 0);
    SoftPWMServoPWMWrite(3,abs(output));
}
double PIDMotor::get_angle() const {
    return r_per_count*position;
}
double PIDMotor::set_angle(double a) {
    setpoint = a;
}

void test_motor() {
    Serial.println("Testing PID Motor. Setting up...");
    PIDMotor motor;
    double p = 1;
    int settle_time = 5000;
    int pos = 0;
    double pos_angle = 1.57079632679;
    int stable = 1;
    motor.setup(p,0,0,PIN_A,PIN_B,DIR_PIN,PWM_PIN,COUNTS_PER_REVOLUTION);
    motor.set_angle(0);
    delay(settle_time);
    while(stable) {
        stable = 1;
        p++;
        pos = !pos;
        Serial.print("Testing kp=");
        Serial.println(p);
        motor.set_angle(pos*pos_angle);
        delay(settle_time);
        for(int i = 0; i^2 < settle_time; i += 10) {
            delay(i^2);//uneven spacing to ensure not same frequency as oscillations (if any)
            stable = stable & abs(motor.get_angle() - pos*pos_angle) < pos_angle*0.1;//if ever false, we consider it to be unstable
        }
    }
    Serial.print("Beggining test with kp=");
    Serial.println(p);
    for(int i = 0; i < 200; i++) {
        Serial.print(motor.get_angle());
        Serial.print(",");
        Serial.println(millis());
        delay(0.2);
    }

}