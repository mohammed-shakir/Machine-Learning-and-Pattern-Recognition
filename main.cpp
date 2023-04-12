#include <iostream>
#include "model.h"

int main() {
    Model model("train-final.csv");

    model.missingData();

    //model.printData();

    return 0;
}
