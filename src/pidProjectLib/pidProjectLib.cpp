#include <pidProject.h>
#include <CP2112.h>
#include <u3.h>
//#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "labjackusb.h"  // Include the Exodriver header


namespace PidProject {

// // Defines how long the command is
// #define CONFIGU3_COMMAND_LENGTH 26

// // Defines how long the response is
// #define CONFIGU3_RESPONSE_LENGTH 38



// Function prototypes for checksum calculations
uint8_t extendedChecksum8(uint8_t *b, int n);
uint16_t extendedChecksum16(uint8_t *b, int n);
void extendedChecksum(uint8_t *b, int n);

// Function to set the PWM duty cycle
void SetPWMDutyCycle(HANDLE devHandle, double dutyCyclePercent);

int SetPwmPower()
{


    HANDLE devHandle;
    uint8_t sendBuffer[14], recBuffer[12];
    int sendChars, recChars;
    uint16_t timerValue;

    printf("Opening device\n");

    // Open the U3 device
    devHandle = LJUSB_OpenDevice(1, 0, U3_PRODUCT_ID);
    if (devHandle == NULL) {
        printf("Couldn't open U3. Please connect one and try again.\n");
        exit(1);
    }

    // Build the Feedback command to configure Timer0
    uint8_t sendBufferSize = 14;
    sendBuffer[0] = 0x00;  // Checksum8 (calculated later)
    sendBuffer[1] = 0xF8;  // Command byte
    sendBuffer[2] = 0x05;  // Number of data words (5)
    sendBuffer[3] = 0x00;  // Extended command number (Feedback)
    sendBuffer[4] = 0x00;  // Checksum16 LSB (calculated later)
    sendBuffer[5] = 0x00;  // Checksum16 MSB (calculated later)

    sendBuffer[6] = 0x00;  // Echo

    int idx = 7; // Start of IOTypes and data

    // Configure FIO4 as Timer0 (PWM output)
    // IOType: Timer0Config (43)
    sendBuffer[7] = 43;    // IOType
    sendBuffer[8] = 0x00;  // TimerMode (0 = 16-bit PWM)
    sendBuffer[9] = 0x00;  // Value LSB (not used for PWM)
    sendBuffer[10] = 0x00;  // Value MSB (not used for PWM)

    // Set initial timer value
    // IOType: Timer0 (42)
    sendBuffer[11] = 42;    // IOType
    sendBuffer[12] = 0x01;  // UpdateReset (bit 0 set)
    sendBuffer[13] = 0x00;  // Value LSB
    sendBuffer[14] = 0x00;  // Value MSB

    // Extra byte to make even number of bytes
    sendBuffer[15] = 0x00;

    // Calculate checksums
    extendedChecksum(sendBuffer, idx);

    // Print the sendBuffer content
    printf("Sending Feedback command to configure Timer0:\n");
    for (int i = 0; i < idx; i++) {
        printf("sendBuffer[%d] = 0x%02X\n", i, sendBuffer[i]);
    }

    // Write the command to the device
    printf("Writing command to device...\n");
    if ((sendChars = LJUSB_Write(devHandle, sendBuffer, idx)) < idx) {
        if (sendChars == 0)
            printf("Write failed\n");
        else
            printf("Did not write all of the buffer (wrote %d bytes)\n", sendChars);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    } else {
        printf("Successfully wrote %d bytes.\n", sendChars);
    }

    // Read the response from the device
    printf("Reading response from device...\n");
    if ((recChars = LJUSB_Read(devHandle, recBuffer, 10)) < 10) {
        if (recChars == 0) {
            printf("Read failed\n");
            LJUSB_CloseDevice(devHandle);
            exit(1);
        } else {
            printf("Did not read all of the buffer (read %d bytes)\n", recChars);
            LJUSB_CloseDevice(devHandle);
            exit(1);
        }
    } else {
        printf("Successfully read %d bytes.\n", recChars);
    }

    // Print the recBuffer content
    printf("Received response:\n");
    for (int i = 0; i < recChars; i++) {
        printf("recBuffer[%d] = 0x%02X\n", i, recBuffer[i]);
    }

    // Check for errors (checksum and command response)
    uint8_t checksum8 = extendedChecksum8(recBuffer, recChars);
    uint16_t checksum16 = extendedChecksum16(recBuffer, recChars);
    if (checksum8 != recBuffer[0]) {
        printf("Checksum8 mismatch! Calculated: 0x%02X, Received: 0x%02X\n", checksum8, recBuffer[0]);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    }
    if ((checksum16 & 0xFF) != recBuffer[4] || ((checksum16 >> 8) & 0xFF) != recBuffer[5]) {
        printf("Checksum16 mismatch! Calculated: 0x%04X, Received: 0x%02X%02X\n", checksum16, recBuffer[5], recBuffer[4]);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    }
    printf("Checksums verified.\n");

    // Check for command errors
    if (recBuffer[6] != 0x00) {
        printf("Command error! Errorcode: %d\n", recBuffer[6]);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    }
    printf("No command errors. PWM output on FIO4 configured.\n");

    // Now set the initial duty cycle
    SetPWMDutyCycle(devHandle, 50.0);

    // Example: Change the duty cycle in a loop
    for (double duty = 0.0; duty <= 100.0; duty += 10.0) {
        SetPWMDutyCycle(devHandle, duty);
        sleep(1); // Wait for 1 second
    }

    // Keep the PWM running until user input
    printf("Press Enter to exit.\n");
    getchar();

    // Close the device
    LJUSB_CloseDevice(devHandle);

    return 0;
}

// Function to set the PWM duty cycle
void SetPWMDutyCycle(HANDLE devHandle, double dutyCyclePercent)
{
    uint8_t sendBuffer[12], recBuffer[8];
    int sendChars, recChars;
    uint16_t value;

    if (dutyCyclePercent < 0.0)
        dutyCyclePercent = 0.0;
    if (dutyCyclePercent > 100.0)
        dutyCyclePercent = 100.0;

    value = (uint16_t)((dutyCyclePercent / 100.0) * 65535.0);

    // Build the Feedback command to set Timer0 value
    sendBuffer[0] = 0x00;  // Checksum8 (calculated later)
    sendBuffer[1] = 0xF8;  // Command byte
    sendBuffer[2] = 0x03;  // Number of data words (3)
    sendBuffer[3] = 0x00;  // Extended command number (Feedback)
    sendBuffer[4] = 0x00;  // Checksum16 LSB (calculated later)
    sendBuffer[5] = 0x00;  // Checksum16 MSB (calculated later)

    sendBuffer[6] = 0x00; // Echo

    int idx = 7;

    // IOType: Timer0 (42)
    sendBuffer[7] = 42; // IOType
    sendBuffer[8] = 1;   // UpdateReset (bit 0 set)
    sendBuffer[9] = value & 0xFF;        // Value LSB
    sendBuffer[10] = (value >> 8) & 0xFF; // Value MSB

    // Extra byte to make even number of bytes
    sendBuffer[11] = 0x00;

    // Recalculate number of data words
    sendBuffer[2] = (12 - 6 + 1) / 2;

    // Calculate checksums
    extendedChecksum(sendBuffer, idx);

    // Print the sendBuffer content
    printf("\nSetting PWM duty cycle to %.2f%%\n", dutyCyclePercent);
    printf("Sending Feedback command:\n");
    for (int i = 0; i < idx; i++) {
        printf("sendBuffer[%d] = 0x%02X\n", i, sendBuffer[i]);
    }

    // Write the command to the device
    printf("Writing command to device...\n");
    if ((sendChars = LJUSB_Write(devHandle, sendBuffer, idx)) < idx) {
        if (sendChars == 0) {
            printf("Write failed\n");
            LJUSB_CloseDevice(devHandle);
            exit(1);
        } else {
            printf("Did not write all of the buffer (wrote %d bytes)\n", sendChars);
            LJUSB_CloseDevice(devHandle);
            exit(1);
        }
    } else {
        printf("Successfully wrote %d bytes.\n", sendChars);
    }

    // Read the response from the device
    printf("Reading response from device...\n");
    if ((recChars = LJUSB_Read(devHandle, recBuffer, 8)) < 8) {
        if (recChars == 0) {
            printf("Read failed\n");
            LJUSB_CloseDevice(devHandle);
            exit(1);
        } else {
            printf("Did not read all of the buffer (read %d bytes)\n", recChars);
            LJUSB_CloseDevice(devHandle);
            exit(1);
        }
    } else {
        printf("Successfully read %d bytes.\n", recChars);
    }

    // Print the recBuffer content
    printf("Received response:\n");
    for (int i = 0; i < recChars; i++) {
        printf("recBuffer[%d] = 0x%02X\n", i, recBuffer[i]);
    }

    // Check for errors (checksum and command response)
    uint8_t checksum8 = extendedChecksum8(recBuffer, recChars);
    uint16_t checksum16 = extendedChecksum16(recBuffer, recChars);
    if (checksum8 != recBuffer[0]) {
        printf("Checksum8 mismatch! Calculated: 0x%02X, Received: 0x%02X\n", checksum8, recBuffer[0]);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    }
    if ((checksum16 & 0xFF) != recBuffer[4] || ((checksum16 >> 8) & 0xFF) != recBuffer[5]) {
        printf("Checksum16 mismatch! Calculated: 0x%04X, Received: 0x%02X%02X\n", checksum16, recBuffer[5], recBuffer[4]);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    }
    printf("Checksums verified.\n");

    // Check for command errors
    if (recBuffer[6] != 0x00) {
        printf("Command error! Errorcode: %d\n", recBuffer[6]);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    }
    printf("PWM duty cycle set to %.2f%%\n", dutyCyclePercent);
}


int GetLabJackFirmware()
{
    HANDLE devHandle;
    uint8_t sendBuffer[26], recBuffer[38]; // Adjust buffer sizes as needed
    int sendChars, recChars;

    printf("Opening device\n");

    // Open the U3 device
    devHandle = LJUSB_OpenDevice(1, 0, U3_PRODUCT_ID);
    if (devHandle == NULL) {
        printf("Couldn't open U3. Please connect one and try again.\n");
        exit(1);
    }

    // Build the ConfigU3 command
    sendBuffer[0] = 0x00;  // Checksum8 (calculated later)
    sendBuffer[1] = 0xF8;  // Command byte
    sendBuffer[2] = 0x0A;  // Number of data words (10)
    sendBuffer[3] = 0x08;  // Extended command number (ConfigU3, 8 decimal)
    sendBuffer[4] = 0x00;  // Checksum16 LSB (calculated later)
    sendBuffer[5] = 0x00;  // Checksum16 MSB (calculated later)

    // Data bytes (fill with zeros)
    for (int i = 6; i < 26; i++) {
        sendBuffer[i] = 0x00;
    }

    // Calculate checksums
    extendedChecksum(sendBuffer, 26);

    // Print the sendBuffer content
    printf("Sending ConfigU3 command:\n");
    for (int i = 0; i < 26; i++) {
        printf("sendBuffer[%d] = 0x%02X\n", i, sendBuffer[i]);
    }

    // Write the command to the device
    printf("Writing command to device...\n");
    if ((sendChars = LJUSB_Write(devHandle, sendBuffer, 26)) < 26) {
        if (sendChars == 0)
            printf("Write failed\n");
        else
            printf("Did not write all of the buffer (wrote %d bytes)\n", sendChars);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    } else {
        printf("Successfully wrote %d bytes.\n", sendChars);
    }

    // Read the response from the device
    printf("Reading response from device...\n");
    if ((recChars = LJUSB_Read(devHandle, recBuffer, 38)) < 38) {
        if (recChars == 0) {
            printf("Read failed\n");
            LJUSB_CloseDevice(devHandle);
            exit(1);
        } else {
            printf("Did not read all of the buffer (read %d bytes)\n", recChars);
            // Continue even if fewer bytes were read
        }
    } else {
        printf("Successfully read %d bytes.\n", recChars);
    }

    // Print the recBuffer content
    printf("Received response:\n");
    for (int i = 0; i < recChars; i++) {
        printf("recBuffer[%d] = 0x%02X\n", i, recBuffer[i]);
    }

    // Check for errors (checksum and command response)
    uint8_t checksum8 = extendedChecksum8(recBuffer, recChars);
    uint16_t checksum16 = extendedChecksum16(recBuffer, recChars);
    if (checksum8 != recBuffer[0]) {
        printf("Checksum8 mismatch! Calculated: 0x%02X, Received: 0x%02X\n", checksum8, recBuffer[0]);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    }
    if ((checksum16 & 0xFF) != recBuffer[4] || ((checksum16 >> 8) & 0xFF) != recBuffer[5]) {
        printf("Checksum16 mismatch! Calculated: 0x%04X, Received: 0x%02X%02X\n", checksum16, recBuffer[5], recBuffer[4]);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    }
    printf("Checksums verified.\n");

    // Check for command errors
    if (recBuffer[6] != 0x00) {
        printf("Command error! Errorcode: %d\n", recBuffer[6]);
        LJUSB_CloseDevice(devHandle);
        exit(1);
    }

    // Parse firmware version
    // Firmware version is located at recBuffer[10] and recBuffer[11]
    uint8_t firmwareMinor = recBuffer[10];
    uint8_t firmwareMajor = recBuffer[11];
    printf("Firmware Version: %d.%02d\n", firmwareMajor, firmwareMinor);

    // Close the device
    LJUSB_CloseDevice(devHandle);

    return 0;
}



// Checksum calculation functions
uint8_t extendedChecksum8(uint8_t *b, int n)
{
    uint16_t a = 0;
    for (int i = 1; i < n; i++)
        a += b[i];
    a = (a & 0xFF) + (a >> 8);
    return (uint8_t)(a & 0xFF);
}

uint16_t extendedChecksum16(uint8_t *b, int n)
{
    uint16_t a = 0;
    for (int i = 6; i < n; i++)
        a += b[i];
    return a;
}

void extendedChecksum(uint8_t *b, int n)
{
    uint16_t a;

    b[4] = 0; // Checksum16 LSB
    b[5] = 0; // Checksum16 MSB
    b[0] = 0; // Checksum8

    a = extendedChecksum16(b, n);
    b[4] = (uint8_t)(a & 0xFF);       // Checksum16 LSB
    b[5] = (uint8_t)((a >> 8) & 0xFF); // Checksum16 MSB

    b[0] = extendedChecksum8(b, n);
}




void HeaterController::Initialize() {
    printf("Running SetPwmPower\n");
    SetPwmPower();
    //GetLabJackFirmware();
    // if (false) {
    //     float thing = LJUSB_GetLibraryVersion();
    //     printf("LJUSB_GetLibraryVersion %f\n", thing);
    //     unsigned int numDevices = LJUSB_GetDevCount(U3_PRODUCT_ID);
    //     printf("LJUSB_GetDevCount %d\n", numDevices);

    //     //HANDLE handle = myBasicConfigMain();
    //     // Setup the variables we will need.
    //     int r = 0; // For checking return values
    //     HANDLE devHandle = 0;
    //     BYTE sendBuffer[CONFIGU3_COMMAND_LENGTH], recBuffer[CONFIGU3_RESPONSE_LENGTH];

    //     // Open the U3
    //     devHandle = LJUSB_OpenDevice(1, 0, U3_PRODUCT_ID);
        
    //     if( devHandle == NULL ) {
    //         printf("Couldn't open U3. Please connect one and try again.\n");
    //         exit(1);
    //     }

    //     // Builds the ConfigU3 command
    //     buildConfigU3Bytes(sendBuffer);
        
    //     // Write the command to the device.
    //     // LJUSB_Write( handle, sendBuffer, length of sendBuffer )
    //     r = LJUSB_Write( devHandle, sendBuffer, CONFIGU3_COMMAND_LENGTH );
        
    //     if( r != CONFIGU3_COMMAND_LENGTH ) {
    //         printf("An error occurred when trying to write the buffer. The error was: %d\n", errno);
    //         // *Always* close the device when you error out.
    //         LJUSB_CloseDevice(devHandle);
    //         exit(1);
    //     }
        
    //     // Read the result from the device.
    //     // LJUSB_Read( handle, recBuffer, number of bytes to read)
    //     r = LJUSB_Read( devHandle, recBuffer, CONFIGU3_RESPONSE_LENGTH );
        
    //     if( r != CONFIGU3_RESPONSE_LENGTH ) {
    //         printf("An error occurred when trying to read from the U3. The error was: %d\n", errno);
    //         LJUSB_CloseDevice(devHandle);
    //         exit(1);
    //     }
        
    //     // Check the command for errors
    //     if( checkResponseForErrors(recBuffer) != 0 ){
    //         LJUSB_CloseDevice(devHandle);
    //         exit(1);
    //     }
        
    //     // Parse the response into something useful
    //     parseConfigU3Bytes(recBuffer);
        

    //     // uint8 sendBuff[32], recBuff[18];
    //     // uint16 checksumTotal;
    //     // int sendChars, recChars;

    //     // sendBuff[1] = (uint8)(0xF8);  //Command byte
    //     // sendBuff[2] = 13;  //Number of data words (.5 word for echo, 8 words for
    //     //                 //IOTypes and data, and .5 words for the extra byte)
    //     // sendBuff[3] = (uint8)(0x00);  //Extended command number

    //     // sendBuff[6] = 0;  //Echo

    //     // sendBuff[7] = 13;  //IOType is BitDirWrite
    //     // sendBuff[8] = 130;  //IONumber (bits 0 - 4) is 2 and Direction (bit 7) is
    //     //                     //output

    //     // sendBuff[9] = 13;  //IOType is BitDirWrite
    //     // sendBuff[10] = 3;  //IONumber (bits 0 - 4) is 3 and Direction (bit 7) is
    //     //                 //input

    //     // sendBuff[11] = 11;  //IOType is BitStateWrite
    //     // sendBuff[12] = 2;  //IONumber (bits 0 - 4) is 2 and State (bit 7) is low

    //     // sendBuff[13] = 43;  //IOType is Timer0Config
    //     // sendBuff[14] = 0;  //TimerMode is 16 bit PWM output (mode 0)
    //     // sendBuff[15] = 0;  //Value LSB
    //     // sendBuff[16] = 0;  //Value MSB, Whole value is 32768

    //     // sendBuff[17] = 42;  //IOType is Timer0
    //     // sendBuff[18] = 1;  //UpdateReset
    //     // sendBuff[19] = 0;  //Value LSB
    //     // sendBuff[20] = 128;  //Value MSB, Whole Value is 32768

    //     // sendBuff[21] = 45;  //IOType is Timer1Config
    //     // sendBuff[22] = 1;  //TimerMode is 8 bit PWM output (mode 1)
    //     // sendBuff[23] = 0;  //Value LSB
    //     // sendBuff[24] = 0;  //Value MSB, Whole value is 32768

    //     // sendBuff[25] = 44;  //IOType is Timer1
    //     // sendBuff[26] = 1;  //UpdateReset
    //     // sendBuff[27] = 0;  //Value LSB
    //     // sendBuff[28] = 128;  //Value MSB, Whole Value is 32768

    //     // sendBuff[29] = 34;  //IOType is DAC0 (8-bit)

    //     // //Value is 1.5 volts (in binary form)
    //     // //getDacBinVoltCalibrated8Bit(caliInfo, 0, 1.5, &sendBuff[30]);
    //     // sendBuff[31] = 0;  //Extra byte

    //     // extendedChecksum(sendBuff, 32);

    //     // //Sending command to U3
    //     // if( (sendChars = LJUSB_Write(devHandle, sendBuff, 32)) < 32 )
    //     // {
    //     //     if( sendChars == 0 )
    //     //         printf("Feedback setup error : write failed\n");
    //     //     else
    //     //         printf("Feedback setup error : did not write all of the buffer\n");
    //     //     return;
    //     // }

    //     // //Reading response from U3
    //     // if( (recChars = LJUSB_Read(devHandle, recBuff, 18)) < 18 )
    //     // {
    //     //     if( recChars == 0 )
    //     //     {
    //     //         printf("Feedback setup error : read failed\n");
    //     //         return;
    //     //     }
    //     //     else
    //     //         printf("Feedback setup error : did not read all of the buffer\n");
    //     // }

    //     // checksumTotal = extendedChecksum16(recBuff, 18);
    //     // if( (uint8)((checksumTotal / 256 ) & 0xFF) != recBuff[5] )
    //     // {
    //     //     printf("Feedback setup error : read buffer has bad checksum16(MSB)\n");
    //     //     return;
    //     // }

    //     // if( (uint8)(checksumTotal & 0xFF) != recBuff[4] )
    //     // {
    //     //     printf("Feedback setup error : read buffer has bad checksum16(LBS)\n");
    //     //     return;
    //     // }

    //     // if( extendedChecksum8(recBuff) != recBuff[0] )
    //     // {
    //     //     printf("Feedback setup error : read buffer has bad checksum8\n");
    //     //     return;
    //     // }

    //     // if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != 6 || recBuff[3] != (uint8)(0x00) )
    //     // {
    //     //     printf("Feedback setup error : read buffer has wrong command bytes \n");
    //     //     return;
    //     // }

    //     // if( recBuff[6] != 0 )
    //     // {
    //     //     printf("Feedback setup error : received errorcode %d for frame %d in Feedback response. \n", recBuff[6], recBuff[7]);
    //     //     return;
    //     // }

    //     // // Configure FIO4 for PWM output
    //     // // Enable FIO4 for PWM (0 = off, 1 = on)
    //     //ePut(devHandle, LJ_ioPUT_CONFIG, LJ_chPWM_ENABLE, 1, 0);

    //     //Close the device.
    //     LJUSB_CloseDevice(devHandle);
    //     // printf("handle %d\n", handle);

    //     // int res = feedback_setup_HV_example(handle, 128);
    //     // printf("Res = %d\n", res);
    //     // u3CalibrationInfo caliInfo;
    //     // if( getCalibrationInfo(handle, &caliInfo) < 0 ) {printf("cali failed\n"); exit(1);}
    //     // if( feedback_setup_HV_example(handle, &caliInfo) != 0 ) {printf("feedback failed\n"); exit(1);}

    //     // Open the LabJack U3-HV device
    //     // if (OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle) != LJE_NOERROR) {
    //     //     std::cerr << "Failed to open LabJack U3-HV device" << std::endl;
    //     //     // TODO exit like this?
    //     //     exit(1);
    //     // }
    //     // Open the U3
    //     //HANDLE devHandle = LJUSB_OpenDevice(1, 0, U3_PRODUCT_ID);

    // } else {
    //     printf("Heater disabled when run Initialize\n");
    // }

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
        //max31889.initialize();
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
    //float currentTemperature = max31889.temperature();
    float currentTemperature = 10.0;
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

