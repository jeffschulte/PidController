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

