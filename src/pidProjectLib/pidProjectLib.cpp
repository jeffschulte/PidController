#include <pidProject.h>
#include <CP2112.h>
#include <Max31889.h>
//#include <cmath>


namespace PidProject {

#define defaultKp 1.0
#define defaultTi 1.0
#define defaultTd 1.0


Thermometer::Thermometer() {

    uint16_t vid = 0x10c4; // Silicon Labs.
    uint16_t  pid = 0xea90; // CP2112 HID I2C Bridge.
    
    unsigned int thing = GetNumHidDevices(vid, pid);
    printf("GetNumHidDevices %d\n", thing);
    //uint8_t status = GetHidString(DWORD deviceIndex, WORD vid, WORD pid, BYTE hidStringType, char* deviceString, DWORD deviceStringLength);
    char device_string_vid[MAX_VID_LENGTH];
    uint8_t status = GetHidString(0, vid, pid, HID_VID_STRING, device_string_vid, MAX_VID_LENGTH);
    char device_string_pid[MAX_PID_LENGTH];
    status = GetHidString(0, vid, pid, HID_PID_STRING, device_string_pid, MAX_PID_LENGTH);

    char device_string_path[MAX_PATH_LENGTH];
    status = GetHidString(0, vid, pid, HID_PATH_STRING, device_string_path, MAX_PATH_LENGTH);
    
    char device_string_serial[MAX_SERIAL_STRING_LENGTH];
    status = GetHidString(0, vid, pid, HID_SERIAL_STRING, device_string_serial, MAX_SERIAL_STRING_LENGTH);

    char device_string_manu[MAX_MANUFACTURER_STRING_LENGTH];
    status = GetHidString(0, vid, pid, HID_MANUFACTURER_STRING, device_string_manu, MAX_MANUFACTURER_STRING_LENGTH);

    char device_string_prod[MAX_PRODUCT_STRING_LENGTH];
    status = GetHidString(0, vid, pid, HID_PRODUCT_STRING, device_string_prod, MAX_PRODUCT_STRING_LENGTH);

    printf("pid %s, vid %s\ndevice_string_path %s, device_string_serial %s\ndevice_string_manu %s, device_string_prod %s\n", 
                device_string_vid, device_string_pid, device_string_path, device_string_serial, device_string_manu, device_string_prod);


    Max31889 max31889;
    max31889.initialize();
    uint64_t temp = max31889.temperature();
    max31889.cleanup();
    printf("\nRead tempurature = %f\n\n", temp);

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
    // // Close the CP2112 device
    // CP2112_Close(cp2112Handle);
};


float Thermometer::GetTemperature() const {
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


HeaterController::HeaterController() {
    float thing = LJUSB_GetLibraryVersion();
    printf("LJUSB_GetLibraryVersion %f\n", thing);
    unsigned int count = LJUSB_GetDevCount(0);
    printf("LJUSB_GetDevCount %d\n", count);

    // // Open the LabJack U3-HV device
    // if (OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle) != LJE_NOERROR) {
    //     std::cerr << "Failed to open LabJack U3-HV device" << std::endl;
    //     // TODO exit like this?
    //     exit(1);
    // }

    // // Configure FIO4 for PWM output
    // // Enable FIO4 for PWM (0 = off, 1 = on)
    // ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chPWM_ENABLE, 1, 0);
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

// void EventLog::AddEvent(Event event) { 
//     eventLog.push_back(event);
// }

// // Aggregate stats for a specific truck and print
// void EventLog::PrintTruckStats(int truck) {
//     std::vector<Event> truckLog;
//     for (int i=0;i<eventLog.size();i++) {
//         if (eventLog[i].truckNumber == truck) {
//             truckLog.push_back(eventLog[i]);
//         }
//     }
//     int numberLoadsMined = 0;
//     int timeSpentWaitingToUnload = 0;
//     for (int i=0;i<truckLog.size();i++) {
//         switch (truckLog[i].eventType)
//         {
//             case ARRIVE_AT_MINING_SITE:
//                 break;
//             case FINISH_MINING:
//                 numberLoadsMined++;
//                 break;
//             case ARRIVE_AT_STATION:
//                 break;
//             case FINISH_UNLOADING:
//                 timeSpentWaitingToUnload += truckLog[i].timeSpentWaiting;
//                 break;
//             default:
//                 printf("\nBadTruckLogEntry\n");
//                 exit(1);
//         }
//     }
//     printf("\nTruck Number %d: NumberLoadsMined = %d, TotalTimeSpentWaitingToUnload = %d, Average Wait Time = %.3f\n", truck, numberLoadsMined, timeSpentWaitingToUnload, (float)timeSpentWaitingToUnload/(float)numberLoadsMined);
// }

// // Aggregate stats for a specific station and print
// void EventLog::PrintStationStats(int station) {
//     std::vector<Event> stationLog;
//     for (int i=0;i<eventLog.size();i++) {
//         if (eventLog[i].stationNumber == station) {
//             stationLog.push_back(eventLog[i]);
//         }
//     }
//     int totalWaitTime = 0;
//     int totalNumberTrucksUnloaded = 0;
//     int j;
//     for (int i=0;i<stationLog.size();i++) {
//         if (stationLog[i].eventType == FINISH_UNLOADING) {
//             totalNumberTrucksUnloaded++;
//             j = i - 1;
//             while (stationLog[j].truckNumber != stationLog[i].truckNumber) j--;
//             totalWaitTime += stationLog[i].time - stationLog[j].time - TRUCK_UNLOAD_TIME;
//         }
//     }
//     printf("\nStation Number %d: TotalNumberTrucksUnloaded = %d, AverageWaitTime = %.3f\n", station, totalNumberTrucksUnloaded, (float)totalWaitTime/(float)totalNumberTrucksUnloaded);
// }

// void EventLog::PrintLog() {
//     printf("\n\neventType,time,truckNumber,stationNumber,truckReservationTime\n");
//     for (int i=0;i<eventLog.size();i++) {
//         printf("%d,%d,%d,%d,%d\n", eventLog[i].eventType, eventLog[i].time, eventLog[i].truckNumber, eventLog[i].stationNumber, eventLog[i].truckReservationTime);
//     }
// }

// int Station::GetRsvdTime() {
//     return rsvdUntil;
// }

// // Reserve next time at a given station. A truck will call this from the site, so first check whether it will have to wait by the time
// // it travels thirty minutes to get there. If it will have to wait, then simply tack on the unload time to the current reserved time.
// // If it will not have to wait, then reserve a time that is the time it takes to travel plus the unload time
// int Station::ReserveNextTime(int currentTime) {
//     if (rsvdUntil > currentTime + TRUCK_TRAVEL_TIME) {
//         rsvdUntil += TRUCK_UNLOAD_TIME;
//         return rsvdUntil;
//     } else {
//         rsvdUntil = currentTime + TRUCK_TRAVEL_TIME + TRUCK_UNLOAD_TIME;
//         return rsvdUntil;
//     }
// }

// // Seed the event list with all the trucks arriving at a mining site at time 0
// MiningManager::MiningManager(int numTrucks, int numStations, EventLog* log, GetMiningTimeFunc func) : stations(numStations, Station(0)), log(log), GetMiningTime(func) {
//     for (int i=0;i<numTrucks;i++) {
//         eventList.insert({ ARRIVE_AT_MINING_SITE, 0, i });
//     }
// }

// // Using singleton pattern for the MiningManager because there should only be one of these in the simulation
// MiningManager* MiningManager::singleInstance = nullptr;
// MiningManager* MiningManager::GetInstance(int numTrucks, int numStations, EventLog* log, GetMiningTimeFunc func) {
//     if (singleInstance == nullptr) {
//         singleInstance = new MiningManager(numTrucks, numStations, log, func);
//     }
//     return singleInstance;
// }

// // Main loop. Picks next event, runs HandleEvent to get next event, then adds that to the sorted list.
// // then picks the next one, etc.
// void MiningManager::RunSimulation() {
//     int currentEventTime = 0;
//     while (eventList.size() > 0) {
//         auto it = eventList.begin();
//         Event nextEvent = HandleEvent(*it);
//         if (nextEvent.time < TOTAL_SIMULATION_TIME) {
//             eventList.insert(nextEvent);
//         }
//         eventList.erase(it);
//     }
// }

// // Scans through all the stations, looking at when each one is reserved, and returns the station with the
// // earliest reservation time
// int MiningManager::GetNextMiningStation() {
//     int currentMinRsvdUntil = TOTAL_SIMULATION_TIME * 0x1000; // just needs to be large
//     int stationNumber = -1;
//     for (int i=0;i<stations.size();i++) {
//         if (stations[i].GetRsvdTime() < currentMinRsvdUntil) {
//             currentMinRsvdUntil = stations[i].GetRsvdTime();
//             stationNumber = i;
//         }
//     }
//     if (stationNumber == -1) {
//         printf("\nIncorrect station number in GetAndReserveNextMiningStation\n");
//         exit(1);
//     }
//     return stationNumber;
// }


// // Process the event, then calculate what is the next event for the same truck that was in this one.  Return the next event.
// Event MiningManager::HandleEvent(Event event) {
//     log->AddEvent(event);
//     int miningTime;
//     int stationToTravelTo;
//     int newRsvdTime;
//     switch(event.eventType) {
//         case ARRIVE_AT_MINING_SITE:
//             // the truck will mine for miningTime and then FINISH_MINING at current + miningTime.
//             miningTime = GetMiningTime();
//             return { FINISH_MINING, event.time + miningTime, event.truckNumber };
//         case FINISH_MINING:
//             // the truck will get a reservation time at a specific station, then depart for that station.
//             stationToTravelTo = GetNextMiningStation();
//             newRsvdTime = stations[stationToTravelTo].ReserveNextTime(event.time);
//             return { ARRIVE_AT_STATION, event.time + TRUCK_TRAVEL_TIME, event.truckNumber, stationToTravelTo, newRsvdTime };
//         case ARRIVE_AT_STATION:
//             // the truck will finish unloading at its pre-given reservation time. Will fill in time spent waiting feild
//             return { FINISH_UNLOADING, event.truckReservationTime, event.truckNumber, event.stationNumber, 0, event.truckReservationTime - event.time - TRUCK_UNLOAD_TIME };
//         case FINISH_UNLOADING:
//             // the truck will depart for the mining site
//             return { ARRIVE_AT_MINING_SITE, event.time + TRUCK_TRAVEL_TIME, event.truckNumber };
//         default:
//             printf("\nUnknown event type! %d\n", event.eventType);
//             exit(1);
//     }
// }


// // Util functions
// void PrintArt() {
//     printf("  _______                  _______              \n");
//     printf(" /       L\_     .-.       /       L\_     __     \n");
//     printf("|           |==( @ )     |           |==|  \\_   \n");
//     printf("'-OO--OO--O-'   '-'      '-OO--OO--O-'   `--´   \n");
//     printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
// }

// void PrintUsage() {
//     printf("\nUsage: ./pidProject -n <number of trucks> -m <number of unload stations>\n");
// }

// void ParseArgs(int argc, char* argv[], int* numTrucks, int* numStations, bool* printLog) {
//     if (argc != 5 && argc != 6) {
//         PrintUsage();
//         std::exit(1);
//     }

//     bool mFound = false, nFound = false;

//     for (int i = 1; i < argc; i++) {
//         std::string arg = argv[i];
//         if (arg == "-m") {
//             *numStations = std::atoi(argv[i + 1]);
//             mFound = true;
//         } else if (arg == "-n") {
//             *numTrucks = std::atoi(argv[i + 1]);
//             nFound = true;
//         } else if (arg == "-p") {
//             *printLog = true;
//         }
//     }

//     if (!mFound || !nFound) {
//         PrintUsage();
//         std::exit(1);
//     }
// }

