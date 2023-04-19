#include <iostream>
#include "model.h"

int main() {
    Model model("train-final.csv");

    model.missingData();
    model.removeSimilarGestures();

    //model.printData();

    model.visualizeGestureByName("bye");

    return 0;
}