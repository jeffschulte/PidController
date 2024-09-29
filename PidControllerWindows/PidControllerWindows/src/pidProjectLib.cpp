#include <pidProject.h>
#include <CP2112.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//#include "labjackusb.h"  // Include the Exodriver header


namespace PidProject {


// Function to configure the LabJack U3 for PWM output on FIO4
void HeaterController::configurePWM() {
    // Reset all pin assignments to default
    ePut(lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);

    // Set the pin offset to 4, so Timer0 starts on FIO4
    ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_COUNTER_PIN_OFFSET, 4, 0);

    // Enable 1 timer (Timer0) to be used on FIO4
    ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 1, 0);

    // Set the timer clock base to 48MHz with a divisor
    ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc48MHZ_DIV, 0);

    // Set the timer clock divisor to 48, creating a 1 MHz clock
    ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 48, 0);

    // Configure Timer0 for 8-bit PWM output mode
    ePut(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmPWM8, 0);

    // Execute the requests
    GoOne(lngHandle);
}



void HeaterController::Initialize() {
    if (heaterEnabled) {
        // Initialize variables
        LJ_ERROR lngErrorcode;

        // Open the LabJack U3 device
        lngErrorcode = OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", true, &lngHandle);
        if (lngErrorcode != 0) {
            std::cerr << "Error opening LabJack U3: " << lngErrorcode << std::endl;
            exit(1);
        }
        configurePWM();
    } else {
        printf("Heater disabled when run Initialize\n");
    }

}

void HeaterController::Cleanup() {
    if (heaterEnabled) {
        // // Disable PWM on FIO4 and close the device
        Close();
    } else {
        printf("Heater disabled when run Cleanup\n");
    }
}

void HeaterController::SetPower(float powerPercentage) {
    if (powerUpdateEnabled) {
        printf("Setting power to %f\n", powerPercentage);
        //SetPwmPower();
        // Configure PWM on FIO4 with an initial duty cycle of 50% (32768 out of 65535)
        
        //int dutyCycle = 32768;  // Start at 50% duty cycle
        int dutyCycle = floor(powerPercentage * 65535.0 / 100.0);  // Start at 50% duty cycle
        if (dutyCycle > 65535) dutyCycle = 65535;  // Cap, just incase
        if (dutyCycle < 0) dutyCycle = 0;  // Cap, just incase

        // Set the PWM duty cycle (0-65535 for 0%-100%)
        // For an 8-bit PWM, this corresponds to the low 8 bits of the timer value.
        ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, dutyCycle, 0);
        // Execute the request
        GoOne(lngHandle);
        std::cout << "Changed duty cycle to: " << (dutyCycle * 100) / 65535 << "%" << std::endl;

    } else {
        printf("Power update disabled when run SetPower\n");
    }
    
    return;
};


PidController::PidController(float inKp, 
                                float inTi, 
                                float inTd,
                                float inTemperatureSetpoint,
                                bool inPowerUpdateEnabled, 
                                bool inPTermEnabled, 
                                bool inITermEnabled,
                                bool inDTermEnabled,
                                bool inHeaterEnabled)
    : temperatureSetpoint(inTemperatureSetpoint), lastError(0.0f), integralSum(0.0f), currentPower(0.0f), kp(inKp),
        ti(inTi), td(inTd), pTermEnabled(inPTermEnabled), iTermEnabled(inITermEnabled), dTermEnabled(inDTermEnabled) {

        heater.heaterEnabled = inHeaterEnabled;
        heater.powerUpdateEnabled = inPowerUpdateEnabled;
        max31889.initialize();
        heater.Initialize();
        latestTime = startTime;
        logFile.open("logFile.csv");
        if (!logFile.is_open()) {
            std::cout << "Error opening to Log File!" << std::endl;
            exit(1);
        }
        logFile << "TimeSinceStart,T_set,T_measured,PowerOutput,PtermContribution,ItermContribution,DtermContribution\n";
}

PidController::~PidController() {
    heater.Cleanup();
    max31889.cleanup();
    logFile.close();
}

void PidController::SetTemperatureSetpoint(float setpoint) {
    temperatureSetpoint = setpoint;
}

float PidController::GetTemperatureSetpoint() const {
    return temperatureSetpoint;
}

void PidController::SetKp(float newKp) {
    kp = newKp;
}

float PidController::GetKp() const {
    return kp;
}

void PidController::SetTi(float newTi) {
    ti = newTi;
}

float PidController::GetTi() const {
    return ti;
}

void PidController::SetTd(float newTd) {
    td = newTd;
}

float PidController::GetTd() const {
    return td;
}

void PidController::EnablePowerUpdate(bool enable) {
    heater.powerUpdateEnabled = enable;
}

void PidController::EnablePterm(bool enable) {
    pTermEnabled = enable;
}

void PidController::EnableIterm(bool enable) {
    iTermEnabled = enable;
}

void PidController::EnableDterm(bool enable) {
    dTermEnabled = enable;
}

void PidController::EnableHeater(bool enable) {
    heater.heaterEnabled = enable;
}

void PidController::SetPower(float power) {
    currentPower = power;
    heater.SetPower(currentPower); // Update the heater power
}

float PidController::GetPower() const {
    return currentPower;
}

void PidController::LogToConsole(float timeSinceStart, float temperatureSetpoint, float currentTemperature, float power, float pTerm, float iTerm, float dTerm) {
    std::cout << "TimeSinceStart " <<  timeSinceStart \
              << "\nT_set " << temperatureSetpoint << "\nT_measured " << currentTemperature \
              << "\nPowerOutput " << power << "\nPtermContribution " << 100.0*(pTerm/power) << "\nItermContribution " \
              << 100.0*(iTerm/power) << "\nDtermContribution " << 100.0*(dTerm/power) << std::endl;
}

void PidController::LogToFile(float timeSinceStart, float temperatureSetpoint, float currentTemperature, float power, float pTerm, float iTerm, float dTerm) {
    logFile << timeSinceStart << "," << temperatureSetpoint << "," << currentTemperature << "," \
            << power << "," << 100.0*(pTerm/power) << "," << 100.0*(iTerm/power) << "," << 100.0*(dTerm/power) << std::endl;
}

void PidController::UpdatePower() {
    //Calculate the error (difference between setpoint and current temperature)
    float currentTemperature = max31889.temperature();
    //float currentTemperature = 10.0;
    printf("Temp = %f\n", currentTemperature);
    float error = temperatureSetpoint - currentTemperature;
    
    // Get time and calculate time delta between now and last update
    auto newTime = std::chrono::high_resolution_clock::now(); 
    float timeDelta = (std::chrono::duration<float>(newTime - latestTime)).count();

    // Proportional term
    float pTerm = pTermEnabled ? kp * error : 0.0f;

    // Integral term
    integralSum += error * timeDelta; // Accumulate error over time
    float iTerm = iTermEnabled ? kp * integralSum / ti : 0.0f;

    // Derivative term (difference in error)
    float dTerm = dTermEnabled ? - kp * td * (error - lastError) / timeDelta: 0.0f;


    // Calculate total output power (clamp to 0-100%)
    float power = pTerm + iTerm + dTerm;
    // if power is outside of bounds, adjust the integral term so that it is in bounds (anti-windup)
    if (power > 100.0f) {
        iTerm -= (power - 100.0);
        integralSum = ti * iTerm / kp;
        power = 100.0;
    } else if (power < 0.0f) {
        iTerm += power;
        integralSum = ti * iTerm / kp;
        power = 0.0f;
    }

    float timeSinceStart = (std::chrono::duration<float>(newTime - latestTime)).count();
    LogToConsole(timeSinceStart, temperatureSetpoint, currentTemperature, power, pTerm, iTerm, dTerm);
    LogToFile(timeSinceStart, temperatureSetpoint, currentTemperature, power, pTerm, iTerm, dTerm);

    // Set the power to the heater
    heater.SetPower(power);

    // Update last error
    lastError = error;
    latestTime = newTime;
}

} // namespace pidProject

