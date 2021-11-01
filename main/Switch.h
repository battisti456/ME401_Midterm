/*
#ifndef SWITCH
#define SWITCH

class Switch {
  private:
  int pin = 0;
  public:
  void setup(int p);
  int get_val() const;
};
void Switch::setup(int p){
  pin = p;
  pinMode(pin,INPUT);
  //attachInterrupt(digitalPinToInterrupt(p),callback,RISING);
}
int Switch::get_val() const {
  return digitalRead(pin);
}

#endif
*/
