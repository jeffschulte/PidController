#include <iostream>
#include <pidProject.h>
#include <chrono>
#include <thread>


int main(int argc, char* argv[]) {

    float kp = 0.042; // %power/C (1--% power is 1.0) 
    float ti = 188; // seconds
    float td = 0.15 * ti; // seconds
    PidProject::PidController pid(kp, ti, td);

    // for (int i=0;i<5;i++) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     pid.UpdatePower();
    // }
    return 0;
}