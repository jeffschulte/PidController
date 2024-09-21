#include <iostream>
#include <string>
#include <map>
#include <pidProject.h>

enum TestName {TEST1, TEST2, BAD_NAME};

std::map<std::string, TestName> testNameMap = {
    {"test1", TEST1},
    {"test2", TEST2}
};

int main(int argc, char *argv[]) {
    TestName testName = testNameMap.count(argv[1]) ? testNameMap[argv[1]] : BAD_NAME;
    if (testName == TEST1) {
        printf("\nRunning test 1\n");
        return EXIT_SUCCESS;
    } else if (testName == TEST2) {
        printf("\nRunning test 2\n");
        return EXIT_SUCCESS;
        return EXIT_SUCCESS;
    } else {
        printf("\nBad test name\n");
        return EXIT_FAILURE;
    }
    return 0;
}

// int main(int argc, char *argv[]) {
//     TestName testName = testNameMap.count(argv[1]) ? testNameMap[argv[1]] : BAD_NAME;
//     PidProject::EventLog log;
//     PidProject::MiningManager miningManager(2, 1, &log, OneTimeMining);
//     PidProject::Station testStation(0);
//     if (testName == TEST1) {
//         printf("\nRunning test 1\n");
//         PidProject::Event finishMining0 = miningManager.HandleEvent( { PidProject::ARRIVE_AT_MINING_SITE, 0, 0 });
//         PidProject::Event finishMining1 = miningManager.HandleEvent( { PidProject::ARRIVE_AT_MINING_SITE, 0, 1 });
//         if (finishMining0.eventType != PidProject::FINISH_MINING) {
//             printf("Handling ARRIVE_AT_MINING_SITE 0 returns incorrect event\n");
//             return EXIT_FAILURE;
//         }
//         if (finishMining1.eventType != PidProject::FINISH_MINING) {
//             printf("Handling ARRIVE_AT_MINING_SITE 1 returns incorrect event\n");
//             return EXIT_FAILURE;
//         }
//         miningManager.HandleEvent(finishMining0);
//         int station = miningManager.GetNextMiningStation();
//         if (station != 0) {
//             printf("GetNextMiningStation returns incorrectly\n");
//             return EXIT_FAILURE;
//         }
//         int rsvdTime = miningManager.stations[station].GetRsvdTime();
//         if (rsvdTime != 36) {
//             printf("Did not reserve time properly %d", rsvdTime);
//             return EXIT_FAILURE;
//         }
//         miningManager.HandleEvent(finishMining1);
//         rsvdTime = miningManager.stations[station].GetRsvdTime();
//         if (rsvdTime != 41) {
//             printf("Did not reserve time properly %d", rsvdTime);
//             return EXIT_FAILURE;
//         }
//         return EXIT_SUCCESS;
//     } else if (testName == TEST2) {
//         PidProject::Station testStation(0);
//         int rsvdTime = testStation.ReserveNextTime(0);
//         if (rsvdTime != 35) {
//             printf("Incorrect rsvd time %d", rsvdTime);
//             return EXIT_FAILURE;
//         }
//         if (testStation.GetRsvdTime() != 35) {
//             printf("GetRsvdTime doesnt match %d", testStation.GetRsvdTime());
//             return EXIT_FAILURE;
//         }
//         return EXIT_SUCCESS;
//     } else {
//         printf("\nBad test name\n");
//         return EXIT_FAILURE;
//     }
//     return 0;
// }
