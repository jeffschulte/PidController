#ifndef PID_PROJECT_H  
#define PID_PROJECT_H

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
//#include "slab_usb_i2c.h"  // Include Silicon Labs CP2112 API
#include "labjackusb.h"  // Include LabJackUD library
#include "hidapi.h"
#include "HIDDevice.h"
#include <Max31889.h>

namespace PidProject {

// PID constants
#define defaultKp 0.042 // %power/C (1--% power is 1.0) 
#define defaultTi 188.0 // seconds
#define defaultTd 28.2 // seconds


class HeaterController {
public:
    // Put these outside of constructor to give user of PID a chance to disable the heater before its intialized
    void Initialize();
    void Cleanup();
    // Function to set heater power in percentage (0% to 100%)
    void SetPower(float powerPercentage);

    bool heaterEnabled;
    bool powerUpdateEnabled;
};

class PidController {
public:
    PidController(float inKp = defaultKp, 
                  float inTi = defaultTi, 
                  float inTd = defaultTd, 
                  bool inPowerUpdateEnabled = true, 
                  bool inPTermEnabled = true, 
                  bool inITermEnabled = true,
                  bool inDTermEnabled = true,
                  bool inHeaterEnabled = true
    );
    ~PidController();

    void SetTemperatureSetpoint(float setpoint);
    float GetTemperatureSetpoint() const;

    void SetKp(float newKp);
    float GetKp() const;

    void SetTi(float newTi);
    float GetTi() const;

    void SetTd(float newTd);
    float GetTd() const;

    // Enable/Disable terms in PID adjustment calculation
    void EnablePowerUpdate(bool enable);
    void EnablePterm(bool enable);
    void EnableIterm(bool enable);
    void EnableDterm(bool enable);
    void EnableHeater(bool enable);

    // Set/Get duty cycle of PWM signal
    void SetPower(float power);
    float GetPower() const;

    // Do PID calculation and set power accordingly 
    void UpdatePower();
    Max31889 max31889; // Thermometer (originally had my own class but Coakley's did everything that was needed)

private:
    float temperatureSetpoint;
    float kp, ti, td; // PID coefficients
    float lastError, integralSum;
    float currentPower;
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime = std::chrono::high_resolution_clock::now(); 
    std::chrono::time_point<std::chrono::high_resolution_clock> latestTime;
    
    bool pTermEnabled;
    bool iTermEnabled;
    bool dTermEnabled;
    HeaterController heater; // Heater controller object
    std::ofstream logFile;
};

} // namespace PidProject

#endif // PID_PROJECT_H

