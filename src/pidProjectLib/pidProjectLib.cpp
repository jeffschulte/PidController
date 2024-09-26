#include <pidProject.h>
#include <CP2112.h>
#include <Max31889.h>
#include <u3.h>
//#include <cmath>


namespace PidProject {

#define defaultKp 1.0
#define defaultTi 1.0
#define defaultTd 1.0


Thermometer::Thermometer() {

    Max31889 max31889;
    max31889.initialize();

    // CP2112 cp2112;
    // cp2112.initialize();
    // cp2112.cleanup();

    // // Initialize the CP2112 USB-to-I2C bridge
    // cp2112Handle = CP2112_Open(0);  // 0 indicates the first CP2112 device connected
    // if (cp2112Handle == INVALID_HANDLE_VALUE) {
    //     std::cerr << "Failed to open CP2112 device" << std::endl;
    //     //TODO exit like this?
    //     exit(1);
    // }

    // // Configure I2C communication (assuming standard settings)
    // CP2112_SetI2CConfig(cp2112Handle, I2C_FREQUENCY_STANDARD);
}

Thermometer::~Thermometer() {
    max31889.cleanup();
};


float Thermometer::GetTemperature() const {
    //double temp = max31889.temperature();
    //printf("\nRead tempurature = %f\n\n", temp);
    auto currentTime = std::chrono::high_resolution_clock::now(); 
    float timeDelta = (std::chrono::duration<float>(currentTime - startTime)).count();
    printf("TimeDelta %f\n", timeDelta);
    return 2.0 - (2.0 / (1.0 + timeDelta));


    // // Send an I2C command to read temperature from MAX31889
    // // MAX31889 I2C address (example: 0x48) and temperature register address (example: 0x00)
    // uint8_t i2cAddress = 0x48;  // Replace with actual I2C address of MAX31889
    // uint8_t readTempCmd[1] = {0x00};  // Register to read temperature data
    // uint8_t tempData[2] = {0};  // Buffer for the 16-bit temperature data

    // // Write command to the sensor to request temperature
    // CP2112_I2CWrite(cp2112Handle, i2cAddress, readTempCmd, sizeof(readTempCmd));

    // // Read temperature data (2 bytes)
    // CP2112_I2CRead(cp2112Handle, i2cAddress, tempData, sizeof(tempData));

    // // Convert the received data into a temperature value
    // int16_t rawTemperature = (tempData[0] << 8) | tempData[1];
    // return rawTemperature * 0.0078125;  // Conversion per MAX31889 datasheet
}


// Defines how long the command is
#define CONFIGU3_COMMAND_LENGTH 26

// Defines how long the response is
#define CONFIGU3_RESPONSE_LENGTH 38


HeaterController::HeaterController() {
    float thing = LJUSB_GetLibraryVersion();
    printf("LJUSB_GetLibraryVersion %f\n", thing);
    unsigned int numDevices = LJUSB_GetDevCount(U3_PRODUCT_ID);
    printf("LJUSB_GetDevCount %d\n", numDevices);

    // HANDLE handle = myBasicConfigMain();
    // printf("handle %d\n", handle);

    // int res = feedback_setup_HV_example(handle, 128);
    // printf("Res = %d\n", res);
    // u3CalibrationInfo caliInfo;
    // if( getCalibrationInfo(handle, &caliInfo) < 0 ) {printf("cali failed\n"); exit(1);}
    // if( feedback_setup_HV_example(handle, &caliInfo) != 0 ) {printf("feedback failed\n"); exit(1);}

    
}









HeaterController::~HeaterController() {
    // // Disable PWM on FIO4 and close the device
    // ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chPWM_ENABLE, 0, 0);  // Disable PWM
    // CloseLabJack(lngHandle);
}

void HeaterController::SetPower(float powerPercentage) {

    // // Set PWM frequency (in Hz) and duty cycle (0.0 to 1.0)
    // double frequency = 1000.0;  // Example: 1 kHz PWM frequency
    // double dutyCycle = 0.5;     // 50% duty cycle

    // // Configure FIO4 for PWM with specified frequency and duty cycle
    // ePut(lngHandle, LJ_ioPUT_DAC, 0, frequency, 0);  // Set frequency
    // ePut(lngHandle, LJ_ioPUT_DAC, 1, dutyCycle, 0);  // Set duty cycle

    // // // Simulate a workload (e.g., wait for user input or run a loop)
    // // std::cout << "PWM signal output on FIO4 with 50% duty cycle" << std::endl;
    // // std::cin.get();  // Wait for user input to stop

    printf("Setting power to %f\n", powerPercentage);
    return;
};


PidController::PidController(float inKp, float inTi, float inTd)
    : temperatureSetpoint(0.0f), kp(defaultKp), ti(defaultTi), td(defaultTd),
      lastError(0.0f), integralSum(0.0f), currentPower(0.0f),
      powerUpdateEnabled(true), pTermEnabled(true), iTermEnabled(true), dTermEnabled(true), heaterEnabled(true) {
        if (kp > 0) kp = inKp;
        if (ti > 0) ti = inTi;
        if (td > 0) td = inTd;
}

//PidController::~PidController() {}

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
    powerUpdateEnabled = enable;
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
    heaterEnabled = enable;
}

void PidController::SetPower(float power) {
    currentPower = power;
    heater.SetPower(currentPower); // Update the heater power
}

float PidController::GetPower() const {
    return currentPower;
}

void PidController::UpdatePower() {
    if (!powerUpdateEnabled || !heaterEnabled) {
        return;
    }

    // Calculate the error (difference between setpoint and current temperature)
    float currentTemperature = thermometer.GetTemperature();
    float error = temperatureSetpoint - currentTemperature;
    
    // Get time and calculate time delta between now and last update
    // TODO check the units (check all units)
    auto newTime = std::chrono::high_resolution_clock::now(); 
    float timeDelta = (std::chrono::duration<float>(newTime - latestTime)).count();

    // Proportional term
    float pTerm = pTermEnabled ? kp * error : 0.0f;

    // Integral term
    integralSum += error * timeDelta; // Accumulate error over time
    float iTerm = iTermEnabled ? kp * integralSum / ti : 0.0f;

    // Derivative term (difference in error)
    float dTerm = dTermEnabled ? - kp * td * (error - lastError) / timeDelta: 0.0f;
    printf("time delta %f, error %f, intergralSum %f, currentTemperature %f\n pTerm %f, iTerm %f, dTerm %f\n", 
                timeDelta, error, integralSum, currentTemperature, pTerm, iTerm, dTerm);


    // Calculate total output power (clamp to 0-100%)
    float power = pTerm + iTerm + dTerm;
    
    // TODO check this they have a note about cutting off
    if (power > 100.0f) power = 100.0f;
    if (power < 0.0f) power = 0.0f;

    // Set the power to the heater
    heater.SetPower(power);

    // Update last error
    lastError = error;
    latestTime = newTime;
}

} // namespace pidProject

