#include "model.h"
#include <iostream>
#include <string>

int main()
{
    Model model("train-final.csv");

    model.missingData();
    model.removeSimilarGestures();

    //model.printData();

    model.listAllGestures();

    std::string gestureName;
    std::cout << "Enter the name of the gesture you want to visualize: ";
    std::getline(std::cin, gestureName);

    model.visualizeGestureByName(gestureName);

    return 0;
}
