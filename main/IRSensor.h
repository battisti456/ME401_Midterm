 #include "MedianFilterLib.h"
 #include "config.h"
 

 class IRSensor {
private:
    int sensor_pin = 0;
    double coefficients[NUMBER_OF_COEFFICIENTS] = {0};//from 0 to high
    MedianFilter<int> filter;
public:
    IRSensor(int pin, int filter_size, int c[NUMBER_OF_COEFFICIENTS]);

    void setup();
    int update();
    double d();
 }

 IRSensor::IRSensor(int pin, int filter_size,double c[NUMBER_OF_COEFFICIENTS]) {
     sensor_pin = pin;
     filter = filter(filter_size);
     for(int i = 0; i < NUMBER_OF_COEFFICIENTS; i++) {
         coefficients[i] = c[i];
     }
 }
void IRSensor::setup() {}
int IRSensor::update(){
    return filter.AddValue(analogRead(sensor_pin));
}
double IRSensor::d(){
    int val = filter.GetFiltered();
    double rtn = 0;
    for(int i = 0; i < NUMBER_OF_COEFFICIENTS;i++) {
        rtn += coefficients[i]*val^i;
    }
    return rtn;
}

void test_sensor(int pin, int filter_size) {
    int val = 0;
    int last_val = val;
    double c[NUMBER_OF_COEFFICIENTS] = {0};
    IRSensor sensor(pin,filter_size,c);
    Serial.print("Testing IRSensor on pin ");
    Serial.println(pin);
    while(1) {
        val = sensor.update();
        if(val != last_val) {
            last_val = val;
            Serial.println(val);
        }
    }
}
