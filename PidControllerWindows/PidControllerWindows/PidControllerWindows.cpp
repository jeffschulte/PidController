
#include <iostream>
#include <stdint.h>
#include <pidProject.h>
#include <chrono>


int main(int argc, char* argv[]) {

    float kp = 0.092; // 0.042 %power/C (1--% power is 1.0) 
    float ti = 188; // seconds
    float td = 0.15 * ti; // seconds
    float tSet = 40.0;
    PidProject::PidController pid(kp, ti, td, tSet);
    for (int i = 0; i < 100; i++) {
        Sleep(500);
        pid.UpdatePower();
    }
    return 0;
}
