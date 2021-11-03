#ifndef IRSENSOR
#define IRSENSOR

#define NUMBER_OF_COEFFICIENTS 9
#define FILTER_LENGTH 50
#define IRSENSOR_UPDATE_MS 100

class IRSensor {
private:
    int sensor_pin = 0;
    double coefficients[NUMBER_OF_COEFFICIENTS] = {0};//from 0 to high

    int value_array[FILTER_LENGTH] = {0};
    int sorted_array[FILTER_LENGTH] = {0};

    void add_value(int value);
    void sort_array();
    int get_median();
public:
    void setup(int pin,double c[NUMBER_OF_COEFFICIENTS]);
    void update(int count);
    int get_i();
    double get_d();

    void print_value_array() const;
    void print_sorted_array() const;
};
void IRSensor::add_value(int value){
  for(int i = 0; i < FILTER_LENGTH-1;i++) {
    value_array[i] = value_array[i+1];
  }
  value_array[FILTER_LENGTH-1] = value;
}
void IRSensor::sort_array(){
  int value = 0;
  int hold = 0;
  for(int i = 0; i < FILTER_LENGTH; i++) {
    sorted_array[i] = 0;
  }
  for(int i = 0; i < FILTER_LENGTH;i++) {
    value = value_array[i];
    for(int j = FILTER_LENGTH-1;j>= 0; j--) {
      if(value == 0) {//0 lowest
        break;
      }
      if(value > sorted_array[j]){
        hold = value;
        value = sorted_array[j];
        sorted_array[j] = hold;
      }
    }
  }
}
int IRSensor::get_median(){
  return sorted_array[(int) (FILTER_LENGTH/2)];//no averaging, just seems easier, biased low, not true median
}
void IRSensor::setup(int pin,double c[NUMBER_OF_COEFFICIENTS]) {
     sensor_pin = pin;
     for(int i = 0; i < NUMBER_OF_COEFFICIENTS; i++) {
         coefficients[i] = c[i];
     }
} 
void IRSensor::update(int count){
  if(count%IRSENSOR_UPDATE_MS == 0) {
    add_value(analogRead(sensor_pin));
  }
}
int IRSensor::get_i(){
  sort_array();
  return get_median();
}
double IRSensor::get_d(){
    int val = get_i();
    double rtn = 0;
    for(int i = 0; i < NUMBER_OF_COEFFICIENTS;i++) {
        rtn += coefficients[i]*pow(val,i);
    }
    return rtn;
}
void IRSensor::print_value_array() const{
  Serial.print("v[");
  for(int i = 0; i < FILTER_LENGTH; i++) {
    Serial.print(value_array[i]);
    if(i != FILTER_LENGTH-1){
      Serial.print(",");
    }
  }
  Serial.println("]");
}
void IRSensor::print_sorted_array() const{
  Serial.print("s[");
  for(int i = 0; i < FILTER_LENGTH; i++) {
    Serial.print(sorted_array[i]);
    if(i != FILTER_LENGTH-1){
      Serial.print(",");
    }
  }
  Serial.println("]");
}

void test_sensor(IRSensor& sensor) {
    int val = 0;
    int last_val = val;
    Serial.println("Testing IRSensor");
    while(1) {
        //sensor.update(IRSENSOR_UPDATE_MS);
        val = sensor.get_i();
        if(val != last_val) {
            last_val = val;
            Serial.println(val);
        }
        //Serial.println(val);
        //sensor.print_value_array();
        //sensor.print_sorted_array();
        delay(250);
    }
}
#endif
