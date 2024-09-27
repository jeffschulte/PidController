#include <pidProject.h>
#include <CP2112.h>
#include <u3.h>
//#include <cmath>


namespace PidProject {

// Defines how long the command is
#define CONFIGU3_COMMAND_LENGTH 26

// Defines how long the response is
#define CONFIGU3_RESPONSE_LENGTH 38


void HeaterController::Initialize() {
    if (false) {
        float thing = LJUSB_GetLibraryVersion();
        printf("LJUSB_GetLibraryVersion %f\n", thing);
        unsigned int numDevices = LJUSB_GetDevCount(U3_PRODUCT_ID);
        printf("LJUSB_GetDevCount %d\n", numDevices);

        //HANDLE handle = myBasicConfigMain();
        // Setup the variables we will need.
        int r = 0; // For checking return values
        HANDLE devHandle = 0;
        BYTE sendBuffer[CONFIGU3_COMMAND_LENGTH], recBuffer[CONFIGU3_RESPONSE_LENGTH];

        // Open the U3
        devHandle = LJUSB_OpenDevice(1, 0, U3_PRODUCT_ID);
        
        if( devHandle == NULL ) {
            printf("Couldn't open U3. Please connect one and try again.\n");
            exit(-1);
        }

        // Builds the ConfigU3 command
        buildConfigU3Bytes(sendBuffer);
        
        // Write the command to the device.
        // LJUSB_Write( handle, sendBuffer, length of sendBuffer )
        r = LJUSB_Write( devHandle, sendBuffer, CONFIGU3_COMMAND_LENGTH );
        
        if( r != CONFIGU3_COMMAND_LENGTH ) {
            printf("An error occurred when trying to write the buffer. The error was: %d\n", errno);
            // *Always* close the device when you error out.
            LJUSB_CloseDevice(devHandle);
            exit(-1);
        }
        
        // Read the result from the device.
        // LJUSB_Read( handle, recBuffer, number of bytes to read)
        r = LJUSB_Read( devHandle, recBuffer, CONFIGU3_RESPONSE_LENGTH );
        
        if( r != CONFIGU3_RESPONSE_LENGTH ) {
            printf("An error occurred when trying to read from the U3. The error was: %d\n", errno);
            LJUSB_CloseDevice(devHandle);
            exit(-1);
        }
        
        // Check the command for errors
        if( checkResponseForErrors(recBuffer) != 0 ){
            LJUSB_CloseDevice(devHandle);
            exit(-1);
        }
        
        // Parse the response into something useful
        parseConfigU3Bytes(recBuffer);
        

        // uint8 sendBuff[32], recBuff[18];
        // uint16 checksumTotal;
        // int sendChars, recChars;

        // sendBuff[1] = (uint8)(0xF8);  //Command byte
        // sendBuff[2] = 13;  //Number of data words (.5 word for echo, 8 words for
        //                 //IOTypes and data, and .5 words for the extra byte)
        // sendBuff[3] = (uint8)(0x00);  //Extended command number

        // sendBuff[6] = 0;  //Echo

        // sendBuff[7] = 13;  //IOType is BitDirWrite
        // sendBuff[8] = 130;  //IONumber (bits 0 - 4) is 2 and Direction (bit 7) is
        //                     //output

        // sendBuff[9] = 13;  //IOType is BitDirWrite
        // sendBuff[10] = 3;  //IONumber (bits 0 - 4) is 3 and Direction (bit 7) is
        //                 //input

        // sendBuff[11] = 11;  //IOType is BitStateWrite
        // sendBuff[12] = 2;  //IONumber (bits 0 - 4) is 2 and State (bit 7) is low

        // sendBuff[13] = 43;  //IOType is Timer0Config
        // sendBuff[14] = 0;  //TimerMode is 16 bit PWM output (mode 0)
        // sendBuff[15] = 0;  //Value LSB
        // sendBuff[16] = 0;  //Value MSB, Whole value is 32768

        // sendBuff[17] = 42;  //IOType is Timer0
        // sendBuff[18] = 1;  //UpdateReset
        // sendBuff[19] = 0;  //Value LSB
        // sendBuff[20] = 128;  //Value MSB, Whole Value is 32768

        // sendBuff[21] = 45;  //IOType is Timer1Config
        // sendBuff[22] = 1;  //TimerMode is 8 bit PWM output (mode 1)
        // sendBuff[23] = 0;  //Value LSB
        // sendBuff[24] = 0;  //Value MSB, Whole value is 32768

        // sendBuff[25] = 44;  //IOType is Timer1
        // sendBuff[26] = 1;  //UpdateReset
        // sendBuff[27] = 0;  //Value LSB
        // sendBuff[28] = 128;  //Value MSB, Whole Value is 32768

        // sendBuff[29] = 34;  //IOType is DAC0 (8-bit)

        // //Value is 1.5 volts (in binary form)
        // //getDacBinVoltCalibrated8Bit(caliInfo, 0, 1.5, &sendBuff[30]);
        // sendBuff[31] = 0;  //Extra byte

        // extendedChecksum(sendBuff, 32);

        // //Sending command to U3
        // if( (sendChars = LJUSB_Write(devHandle, sendBuff, 32)) < 32 )
        // {
        //     if( sendChars == 0 )
        //         printf("Feedback setup error : write failed\n");
        //     else
        //         printf("Feedback setup error : did not write all of the buffer\n");
        //     return;
        // }

        // //Reading response from U3
        // if( (recChars = LJUSB_Read(devHandle, recBuff, 18)) < 18 )
        // {
        //     if( recChars == 0 )
        //     {
        //         printf("Feedback setup error : read failed\n");
        //         return;
        //     }
        //     else
        //         printf("Feedback setup error : did not read all of the buffer\n");
        // }

        // checksumTotal = extendedChecksum16(recBuff, 18);
        // if( (uint8)((checksumTotal / 256 ) & 0xFF) != recBuff[5] )
        // {
        //     printf("Feedback setup error : read buffer has bad checksum16(MSB)\n");
        //     return;
        // }

        // if( (uint8)(checksumTotal & 0xFF) != recBuff[4] )
        // {
        //     printf("Feedback setup error : read buffer has bad checksum16(LBS)\n");
        //     return;
        // }

        // if( extendedChecksum8(recBuff) != recBuff[0] )
        // {
        //     printf("Feedback setup error : read buffer has bad checksum8\n");
        //     return;
        // }

        // if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != 6 || recBuff[3] != (uint8)(0x00) )
        // {
        //     printf("Feedback setup error : read buffer has wrong command bytes \n");
        //     return;
        // }

        // if( recBuff[6] != 0 )
        // {
        //     printf("Feedback setup error : received errorcode %d for frame %d in Feedback response. \n", recBuff[6], recBuff[7]);
        //     return;
        // }

        // // Configure FIO4 for PWM output
        // // Enable FIO4 for PWM (0 = off, 1 = on)
        //ePut(devHandle, LJ_ioPUT_CONFIG, LJ_chPWM_ENABLE, 1, 0);

        //Close the device.
        LJUSB_CloseDevice(devHandle);
        // printf("handle %d\n", handle);

        // int res = feedback_setup_HV_example(handle, 128);
        // printf("Res = %d\n", res);
        // u3CalibrationInfo caliInfo;
        // if( getCalibrationInfo(handle, &caliInfo) < 0 ) {printf("cali failed\n"); exit(1);}
        // if( feedback_setup_HV_example(handle, &caliInfo) != 0 ) {printf("feedback failed\n"); exit(1);}

        // Open the LabJack U3-HV device
        // if (OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle) != LJE_NOERROR) {
        //     std::cerr << "Failed to open LabJack U3-HV device" << std::endl;
        //     // TODO exit like this?
        //     exit(1);
        // }
        // Open the U3
        //HANDLE devHandle = LJUSB_OpenDevice(1, 0, U3_PRODUCT_ID);

    } else {
        printf("Heater disabled when run Initialize\n");
    }

}

void HeaterController::Cleanup() {
    if (heaterEnabled) {
        // // Disable PWM on FIO4 and close the device
        // ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chPWM_ENABLE, 0, 0);  // Disable PWM
        // CloseLabJack(lngHandle);
    } else {
        printf("Heater disabled when run Cleanup\n");
    }
}

void HeaterController::SetPower(float powerPercentage) {
    if (powerUpdateEnabled) {
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
    } else {
        printf("Power update disabled when run SetPower\n");
    }
    
    return;
};


PidController::PidController(float inKp, 
                                float inTi, 
                                float inTd, 
                                bool inPowerUpdateEnabled, 
                                bool inPTermEnabled, 
                                bool inITermEnabled,
                                bool inDTermEnabled,
                                bool inHeaterEnabled)
    : temperatureSetpoint(0.0f), lastError(0.0f), integralSum(0.0f), currentPower(0.0f), kp(inKp),
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


void PidController::UpdatePower() {
    //Calculate the error (difference between setpoint and current temperature)
    float currentTemperature = max31889.temperature();
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

    std::cout << "TimeSinceStart " <<  (std::chrono::duration<float>(newTime - latestTime)).count() \
              << "\nT_set " << temperatureSetpoint << "\nT_measured " << currentTemperature \
              << "\nPowerOutput " << power << "\nPtermContribution " << 100.0*(pTerm/power) << "\nItermContribution " \
              << 100.0*(iTerm/power) << "\nDtermContribution " << 100.0*(dTerm/power) << std::endl;
    logFile << "Hello, world!" << std::endl;

    // Set the power to the heater
    heater.SetPower(power);

    // Update last error
    lastError = error;
    latestTime = newTime;
}

} // namespace pidProject

