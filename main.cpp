#include <iostream>
#include "model.h"

int main() {
    Model model("train-final.csv");

    model.missingData();

    //model.printData();

    model.visualizeGesture(0);

    return 0;
}