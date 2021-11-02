#ifndef SWITCH
#define SWITCH

#define SWITCH_UPDATE_MS 100

class Switch {
  private:
  int pin = 0;
  public:
  void setup(int p);
  void update(int ms);
  int get_val() const;
};
void Switch::setup(int p){
  pin = p;
  pinMode(pin,INPUT);
}
int Switch::get_val() const {
  return digitalRead(pin);
}
#endif
