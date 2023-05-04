#include "model.h"
#include <iostream>
#include <string>

int main()
{
    Model model("train-final.csv");

    model.missingData();
    model.removeSimilarGestures();

    // model.printData();

    model.listAllGestures();
    model.exportData("exported_train_data.csv");

    Model test_model("test-final.csv");
    test_model.missingData();

    test_model.exportData("exported_test_data.csv");
    // std::string gestureName;
    // std::cout << "Enter the name of the gesture you want to visualize: ";
    // std::getline(std::cin, gestureName);

    // model.visualizeGestureByName(gestureName);

    do
    {
        cout << '\n'
             << "Press a key to continue...";
    } while (cin.get() != '\n');

    return 0;

    return 0;
}
