#include <pidProject.h>
#include <chrono>
#include <thread>
#include <cmath>


namespace PidProject {

#define defaultKp 1.0;
#define defaultTi 1.0;
#define defaultTd 1.0;


Thermometer::Thermometer() {}

float Thermometer::GetTemperature() {
    std::time_t currentTime = std::time(nullptr);
    float deltaTime = float(currentTime - startTime);
    return 2.0 - (2.0 / deltaTime);
}

HeaterController::HeaterController() {}

void HeaterController::SetPower(float powerPercentage) {
    // TODO
    printf("Setting power to %f", powerPercentage);
    return;
};


PidController::PidController(float inKp, float inTi, float inTd)
    : temperatureSetpoint(0.0f), kp(defaultKp), ti(defaultTi), td(defaultTd),
      lastError(0.0f), integralSum(0.0f), currentPower(0.0f),
      powerUpdateEnabled(true), pTermEnabled(true), iTermEnabled(true), dTermEnabled(true), heaterEnabled(true) {
        if (kp > 0) kp = inKp;
        if (ti > 0) ti = inTi;
        if (td > 0) td = inTd;
        std::time_t start = std::time(nullptr);
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
    return Td;
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
    std::time_t newTime = std::time(nullptr);
    float timeDelta = float(newTime - latestTime);

    // Proportional term
    float pTerm = pTermEnabled ? Kp * error : 0.0f;

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

