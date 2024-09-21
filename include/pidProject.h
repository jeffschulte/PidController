#ifndef PID_PROJECT_H  
#define PID_PROJECT_H

#include <iostream>
//#include <ctime>
#include <chrono>
#include <thread>

namespace PidProject {

class Thermometer {
public:
    Thermometer();
    //~Thermometer();

    float GetTemperature() const;
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime = std::chrono::high_resolution_clock::now(); 
};

class HeaterController {
public:
    HeaterController();
    //~HeaterController();

    // Function to set heater power in percentage (0% to 100%)
    // put guards around 1 and 0, thats the xscale, comment
    void SetPower(float powerPercentage);
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime = std::chrono::high_resolution_clock::now(); 
};

class PidController {
public:
    PidController(float inKp, float inTi, float inTd);
    //~PidController();

    // Setters and Getters for PID parameters
    void SetTemperatureSetpoint(float setpoint);
    float GetTemperatureSetpoint() const;

    void SetKp(float newKp);
    float GetKp() const;

    void SetTi(float newTi);
    float GetTi() const;

    void SetTd(float newTd);
    float GetTd() const;

    // Enable/Disable control features
    void EnablePowerUpdate(bool enable);
    void EnablePterm(bool enable);
    void EnableIterm(bool enable);
    void EnableDterm(bool enable);
    void EnableHeater(bool enable);

    // Set/Get power
    void SetPower(float power);
    float GetPower() const;

    // PID calculation and loop execution
    void UpdatePower();

private:
    float temperatureSetpoint;
    float kp, ti, td; // PID coefficients
    float lastError, integralSum;
    float currentPower;
    std::chrono::time_point<std::chrono::high_resolution_clock> latestTime = std::chrono::high_resolution_clock::now(); 

    bool powerUpdateEnabled;
    bool pTermEnabled;
    bool iTermEnabled;
    bool dTermEnabled;
    bool heaterEnabled;

    HeaterController heater; // Heater controller object
    Thermometer thermometer;
};

} // namespace PidProject

#endif // PID_PROJECT_H


// #define TRUCK_TRAVEL_TIME 30 // thirty minutes to travel from site to station
// #define TRUCK_UNLOAD_TIME 5 // 5 minutes to unload load at station
// #define TOTAL_SIMULATION_TIME 72*60 // 72 hours of total simulation
// //#define TOTAL_SIMULATION_TIME 1500 // 72 hours of total simulation

// enum EventType {
//     ARRIVE_AT_MINING_SITE,
//     FINISH_MINING,
//     ARRIVE_AT_STATION,
//     FINISH_UNLOADING,
// };

// struct Event {
//     EventType eventType;
//     int time;
//     int truckNumber;
//     int stationNumber = 0;
//     int truckReservationTime = 0;
//     int timeSpentWaiting = 0;
// };

// // This is the sorted list (sorted in time) of all the events.
// class EventLog {
//     std::vector<Event> eventLog;
// public:
//     void AddEvent(Event event);
//     void PrintLog();
//     void PrintTruckStats(int truck);
//     void PrintStationStats(int station);
// };

// // The stations keep track of their latest reservation time, which can be quaried and updated
// class Station {
//     int rsvdUntil = 0;
// public:
//     int GetRsvdTime();
//     int ReserveNextTime(int currentTime);
//     Station(int initRsvdUntil) : rsvdUntil(initRsvdUntil) {};
// };

// // The conducting class is the MiningManager, and we use singleton pattern to ensure that there is only one of these.
// // It holds the event list, which the core loop of the program, RunSimulation(), will loop through.
// // It also holds a list of the stations
// // Also holds the event log, which has copies of all the events that occured along the way. 
// typedef int (*GetMiningTimeFunc)();
// class MiningManager {
//     static MiningManager* singleInstance;
//     struct EventCompare {
//         bool operator()(const Event& lhs, const Event& rhs) const {
//             return lhs.time < rhs.time; // Order elements in descending order
//         };
//     };
//     std::multiset<Event, EventCompare> eventList;
//     GetMiningTimeFunc GetMiningTime;
// // about this ifdef. I wouldnt do this in production. I would either set up a class in the test executable that is a freind of this one,
// // and then in that class would instantiate a MiningManager, and then be able to run all the private functions on it,
// // or I would use google test (we used gtest at my first company), which has ways of dealing with this. Unfoturnatley Im running out of time
// // so this is my quick and dirty solution to the testing-access-to-private-function issue
// #define UNIT_TEST
// #ifdef UNIT_TEST
// public:
// #endif
//     std::vector<Station> stations;

//     Event HandleEvent(Event event);
//     int GetNextMiningStation();
//     EventLog* log;
// public:
//     MiningManager(int numTrucks, int numStations, EventLog* log, GetMiningTimeFunc func);
//     // Singleton design pattern - disallow copying by deleting copy and assignment constructors
//     MiningManager(const MiningManager&) = delete;
//     MiningManager& operator=(const MiningManager&) = delete;
//     static MiningManager* GetInstance(int numTrucks, int numStations, EventLog* log, GetMiningTimeFunc func);
//     void RunSimulation();
// };

// // Util functions
// void PrintArt();
// void PrintUsage();
// void ParseArgs(int argc, char* argv[], int* numTrucks, int* numStations, bool* printLog);

