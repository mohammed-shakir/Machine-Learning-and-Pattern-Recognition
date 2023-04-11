#include "preprocessor.h"
#include <iostream>

int main() {
    Preprocessor p("train-final.csv");
    // p.printData();

    p.missingValues();

    return 0;
}
