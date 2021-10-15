#include <math.h>

struct pos {
    double x;//in meters
    double y;//in meters
    double a;//in radians
};

struct pos current;

int l_speed = 0;//-100 to 100
int r_speed = 0;//-100 to 100

double wheel_distance = 1;//measure, in meters
double wheel_circumfrance = 1;//measure, in meters

double s_from_speed(int speed) {//int servo values to arclength
    return (double) speed/100*wheel_circumfrance;
}
int speed_from_s(double s) {//arclength to int servo values
    return s/wheel_circumfrance*100;
}

void find_sr_sl_for_move(double x1, double y1, double& sr, double& sl) {//SFDR
    double dx = x1 - current.x;
    double dy = y1 - current.y;
    double a = atan(dy/dx);
    double smx = dx/cos(a);
    double smy = dy/sin(a);
    double sm = (smx+smy)/2;//so no direction preference

    sr = sm+wheel_distance*(a-current.a);
    sl = 2*sm-sr;
}

struct pos pos_from_movement() {//ud6yh5ysa45rhtye5y5yrgh
    double sr = s_from_speed(r_speed);
    double sl = s_from_speed(l_speed);

    

}