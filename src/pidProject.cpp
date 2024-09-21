#include <iostream>
#include <pidProject.h>
#include <chrono>


int main(int argc, char* argv[]) {

    float kp = 0.042; // %power/C (1--% power is 1.0) 
    float ti = 188; // seconds
    flaot td = 0.15 * ti // seconds
    PidController pid(kp, ti, td);

    for (int i=0;i<5;i++) {
        pid.UpdatePower()
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}