// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <stdlib.h>
#include <LabJackM.h>
#include <LabJackUD.h>
#include <Windows.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <CP2112.h>
#include <Max31889.h>
#include <SLABCP2112.h>




// Function to configure the LabJack U3 for PWM output on FIO4
void configurePWM(LJ_HANDLE lngHandle, int dutyCycle) {
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

    // Set the PWM duty cycle (0-65535 for 0%-100%)
    // For an 8-bit PWM, this corresponds to the low 8 bits of the timer value.
    ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, dutyCycle, 0);

    // Execute the requests
    GoOne(lngHandle);
}


int main()
{
    std::cout << "Hello World!\n";
    Max31889 max31889;
    int initRes = max31889.initialize();
    printf("Temp initRes %d\n", initRes);



    // Initialize variables
    LJ_HANDLE lngHandle;
    LJ_ERROR lngErrorcode;

    // Open the LabJack U3 device
    lngErrorcode = OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", true, &lngHandle);
    if (lngErrorcode != 0) {
        std::cerr << "Error opening LabJack U3: " << lngErrorcode << std::endl;
        return lngErrorcode;
    }

    // Configure PWM on FIO4 with an initial duty cycle of 50% (32768 out of 65535)
    int dutyCycle = 32768;  // Start at 50% duty cycle
    configurePWM(lngHandle, dutyCycle);

    std::cout << "PWM output started on FIO4 at 50% duty cycle." << std::endl;

    // Adjust the duty cycle dynamically in a loop
    char userInput;
    while (true) {
        double temp = max31889.temperature();
        printf("Tempurature = %f\n", temp);
        std::cout << "Enter 'i' to increase duty cycle, 'd' to decrease, 'q' to quit: ";
        std::cin >> userInput;

        if (userInput == 'i') {
            if (dutyCycle < 65535) {
                dutyCycle += 6553;  // Increase duty cycle by ~10%
                if (dutyCycle > 65535) dutyCycle = 65535;  // Cap at 100%
                configurePWM(lngHandle, dutyCycle);
                std::cout << "Increased duty cycle to: " << (dutyCycle * 100) / 65535 << "%" << std::endl;
            }
            else {
                std::cout << "Duty cycle already at 100%." << std::endl;
            }
        }
        else if (userInput == 'd') {
            if (dutyCycle > 0) {
                dutyCycle -= 6553;  // Decrease duty cycle by ~10%
                if (dutyCycle < 0) dutyCycle = 0;  // Cap at 0%
                configurePWM(lngHandle, dutyCycle);
                std::cout << "Decreased duty cycle to: " << (dutyCycle * 100) / 65535 << "%" << std::endl;
            }
            else {
                std::cout << "Duty cycle already at 0%." << std::endl;
            }
        }
        else if (userInput == 'q') {
            break;  // Exit the loop
        }
        else {
            std::cout << "Invalid input." << std::endl;
        }
    }

    int cleanRes = max31889.cleanup();
    printf("cleanRes = %d\n", cleanRes);

    Close();
}

    // Initialize variables
    //LJ_HANDLE lngHandle;
    //LJ_ERROR lngErrorcode;

    //// Open the LabJack U3 device
    //lngErrorcode = OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", true, &lngHandle);
    //if (lngErrorcode != 0) {
    //    std::cerr << "Error opening LabJack U3: " << lngErrorcode << std::endl;
    //    return lngErrorcode;
    //}

    //// Configure FIO4 as a waveform output
    //ePut(lngHandle, LJ_ioPUT_DAC, 4, 0.0, 0); // Initialize FIO4 to 0V
    //ePut(lngHandle, LJ_ioPUT_ANALOG_ENABLE_BIT, 4, 1, 0); // Enable analog output on FIO4

    //// Set up the waveform parameters
    //const int numPoints = 100; // Number of points in the waveform
    //double waveform[numPoints];

    //// Create a square wave
    //double highValue = 2.5; // High voltage level
    //double lowValue = 0.0;  // Low voltage level
    //int halfPeriod = numPoints / 2; // Points for half the wave

    //for (int i = 0; i < numPoints; ++i) {
    //    waveform[i] = (i < halfPeriod) ? highValue : lowValue; // Generate square wave
    //}

    //// Write the waveform to FIO4
    //uint32_t error = AddRequest(lngHandle, LJ_ioPUT_WAVEFORM, 4, &waveform[0], numPoints);
    //if (error != 0) {
    //    std::cerr << "Error adding waveform request: " << error << std::endl;
    //}

    //// Execute the requests
    //GoOne(lngHandle);

    //// Start outputting the waveform
    //ePut(lngHandle, LJ_ioPUT_WAVEFORM_START, 4, 0, 0); // Start outputting the waveform

    //std::cout << "Outputting square wave signal on FIO4." << std::endl;

    //// Wait for a bit to see the output (you can adjust this)
    //std::this_thread::sleep_for(std::chrono::seconds(5));

    //// Stop outputting the waveform
    //ePut(lngHandle, LJ_ioPUT_WAVEFORM_STOP, 4, 0, 0); // Stop outputting the waveform




    // Initialize variables
    //LJ_HANDLE lngHandle;
    //LJ_ERROR lngErrorcode;
    //double voltage;

    //// Open the LabJack U3 device
    //lngErrorcode = OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", true, &lngHandle);
    //if (lngErrorcode != 0) {
    //    std::cerr << "Error opening LabJack U3: " << lngErrorcode << std::endl;
    //    return lngErrorcode;
    //}

    //// Configure FIO4 as an analog output
    //ePut(lngHandle, LJ_ioPUT_ANALOG_ENABLE_BIT, 4, 1, 0); // Enable analog output on FIO4

    //// Output a square wave signal on FIO4
    //const int duration = 5; // Duration in seconds to output the wave
    //const int frequency = 1; // Frequency in Hz
    //const int halfPeriod = 1000 / (2 * frequency); // Half period in milliseconds

    //std::cout << "Outputting a square wave signal on FIO4 for " << duration << " seconds." << std::endl;

    //// Loop to output the square wave signal
    //auto endTime = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
    //while (std::chrono::steady_clock::now() < endTime) {
    //    // Set FIO4 high (2.5V)
    //    ePut(lngHandle, LJ_ioPUT_DAC, 4, 2.5, 0); // Set FIO4 high
    //    GoOne(lngHandle);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(halfPeriod));

    //    // Set FIO4 low (0V)
    //    ePut(lngHandle, LJ_ioPUT_DAC, 4, 0, 0); // Set FIO4 low
    //    GoOne(lngHandle);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(halfPeriod));
    //}

    //// Read back the output from FIO4
    //uint32_t error = AddRequest(lngHandle, LJ_ioGET_AIN, 4, 0, 0, 0); // Read from FIO4 (AIN4)
    //if (error != 0) {
    //    std::cerr << "Error adding request for reading: " << error << std::endl;
    //}

    //// Execute the requests
    //GoOne(lngHandle);

    //// Get the voltage reading from FIO4
    //error = GetResult(lngHandle, LJ_ioGET_AIN, 4, &voltage);
    //if (error != 0) {
    //    std::cerr << "Error getting result: " << error << std::endl;
    //}
    //else {
    //    std::cout << "Voltage on FIO4 (AIN4): " << voltage << " V" << std::endl;
    //}






    // Open the LabJack U3 device
    //LJ_HANDLE lngHandle;
    //LJ_ERROR lngErrorcode = OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", true, &lngHandle);
    //if (lngErrorcode != 0) {
    //    std::cerr << "Error opening LabJack U3: " << lngErrorcode << std::endl;
    //    return lngErrorcode;
    //}

    //// Set FIO4 as a digital output
    //ePut(lngHandle, LJ_ioPUT_ANALOG_ENABLE_BIT, 4, 0, 0); // Enable digital output on FIO4

    //// Generate a square wave signal
    //const int duration = 5; // Duration in seconds to output the wave
    //const int frequency = 1; // Frequency in Hz
    //const int halfPeriod = 1000 / (2 * frequency); // Half period in milliseconds

    //std::cout << "Outputting a square wave signal on FIO4 for " << duration << " seconds." << std::endl;

    //// Loop to output the square wave signal
    //auto endTime = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
    //while (std::chrono::steady_clock::now() < endTime) {
    //    // Set FIO4 high
    //    ePut(lngHandle, LJ_ioPUT_DIGITAL_BIT, 4, 1, 0); // Set FIO4 high
    //    GoOne(lngHandle);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(halfPeriod));

    //    // Set FIO4 low
    //    ePut(lngHandle, LJ_ioPUT_DIGITAL_BIT, 4, 0, 0); // Set FIO4 low
    //    GoOne(lngHandle);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(halfPeriod));
    //}

    //// Clean up
    //ePut(lngHandle, LJ_ioPUT_DIGITAL_BIT, 4, 0, 0); // Ensure FIO4 is low at the end
    //Close();

    //std::cout << "Finished outputting the wave signal." << std::endl;





    //// Initialize variables
    //LJ_HANDLE lngHandle;
    //LJ_ERROR lngErrorcode;
    //double voltage;

    //// Open the LabJack U3 device
    //lngErrorcode = OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", true, &lngHandle);
    //if (lngErrorcode != 0) {
    //    std::cerr << "Error opening LabJack U3: " << lngErrorcode << std::endl;
    //    return lngErrorcode;
    //}

    //// Set FIO4 as an analog input (which corresponds to AIN4)
    //// Configure the FIO4 channel (analog enable bit)
    //ePut(lngHandle, LJ_ioPUT_ANALOG_ENABLE_BIT, 4, 1, 0); // FIO4 is channel 4

    //// Request a read from AIN4 (which corresponds to FIO4)
    //uint32_t error = AddRequest(lngHandle, LJ_ioGET_AIN, 4, 0, 0, 0);
    //if (error != 0) {
    //    std::cerr << "Error adding request: " << error << std::endl;
    //    Close();
    //    return error;
    //}

    //// Execute the requests
    //GoOne(lngHandle);

    //// Get the voltage reading from FIO4
    //error = GetResult(lngHandle, LJ_ioGET_AIN, 4, &voltage);
    //if (error != 0) {
    //    std::cerr << "Error getting result: " << error << std::endl;
    //}
    //else {
    //    std::cout << "Voltage on FIO4 (AIN4): " << voltage << " V" << std::endl;
    //}

    
    
    
    //
    //
    //LJ_HANDLE lngHandle;
    //printf("Hello!! Opening device\n");
    //LJ_ERROR lngErrorcode = OpenLabJack(LJ_dtU6, LJ_ctUSB, "1", true, &lngHandle);
    //Sleep(1000);
    ////AddRequest(lngHandle, LJ_ioGET_DIGITAL_BIT, 4, 0, 0, 0);
    //
    ////Configure FIO3 as an analog input.
    //ePut(lngHandle, LJ_ioPUT_ANALOG_ENABLE_BIT, 0, 1, 0);
    ////Request a single-ended read from AIN2.
    ////AddRequest(lngHandle, LJ_ioGET_AIN, 2, 0, 0, 0);

    ////Request a read from FIO2.
    //uint32_t error = AddRequest(lngHandle, LJ_ioGET_AIN, 0, 0, 0, 0);
    //printf("error %d\n", error);
    ////Execute the requests.
    //GoOne(lngHandle);
    ////Get the FIO2 read.
    //double dblValue = 0;
    //uint32_t result = GetResult(lngHandle, LJ_ioGET_AIN, 0, &dblValue);
    //printf("result %d, dblValue %f\n", result, dblValue);
    //printf("Resetting pin config\n");
    //ePut(lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
    //Sleep(1000);
    //printf("Doing pin setup\n");
    //AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_COUNTER_PIN_OFFSET, 4, 0, 0);
    //printf("Enable both timers.They will use FIO4 - FIO5\n");
    //AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 2, 0, 0);
    ////Make sure Counter0 is disabled.
    //AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 0, 0, 0, 0);

    ////Enable Counter1. It will use the next available line, FIO6.
    //AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 1, 1, 0, 0);

    ////All output timers use the same timer clock, configured here. The
    ////base clock is set to 48MHZ_DIV, meaning that the clock divisor
    ////is supported and Counter0 is not available. Note that this timer
    ////clock base is not valid with U3 hardware version 1.20.
    //AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc48MHZ_DIV, 0, 0);

    ////Set the timer clock divisor to 48, creating a 1 MHz timer clock.
    //AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 48, 0, 0);

    ////Configure Timer0 as 8-bit PWM. It will have a frequency
    ////of 1M/256 = 3906.25 Hz.
    //AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmPWM8, 0, 0);

    ////Initialize the 8-bit PWM with a 50% duty cycle.
    ////AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 32768, 0, 0);
    //AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 65530, 0, 0);
    ////AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 0, 0, 0);

    ////Configure Timer1 as duty cycle input.
    //AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 1, LJ_tmDUTYCYCLE, 0, 0);

    ////Execute the requests.
    //GoOne(lngHandle);
    //Sleep(2000);
    ////Read duty-cycle from Timer1.
    //double dblValue;
    ////Change Timer0 PWM duty cycle to 25%.
    ////ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 49152, 0);
    //ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 0, 0);

    //uint32_t dutyCycleRead = eGet(lngHandle, LJ_ioGET_TIMER_VALUE, 1, &dblValue, 0);
    //uint64_t lowValue = (uint64_t(dblValue) & 0x0000000000001111);
    //uint64_t highValue = (uint64_t(dblValue) & 0x0000000011110000) >> 16;
    //uint64_t rest = (uint64_t(dblValue) & 0x1111111100000000) >> 32;
    //uint64_t thing = (uint64_t(dblValue) & 0x1111111111111111);
    ////printf("dbl lowValue %lld, highValue %lld, rest %lld, dblValue %lld, thing %lld, thing2 0x%x\n", lowValue, highValue, rest, uint64_t(dblValue), thing, thing2);
    //printf("dutyCycleRead 0x%x\n", dutyCycleRead);
    //AddRequest(lngHandle, LJ_ioGET_COUNTER, 1, 0, 0, 0);
    //AddRequest(lngHandle, LJ_ioPUT_COUNTER_RESET, 1, 1, 0, 0);
    //GoOne(lngHandle);
    //uint32_t thing2 = GetResult(lngHandle, LJ_ioGET_COUNTER, 1, &dblValue);
    //printf("dblValue %f, %d\n", dblValue, thing2);
    //GetResult(lngHandle, LJ_ioPUT_COUNTER_RESET, 1, 0);
    //Sleep(2000);
    //printf("Closing device\n");

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
