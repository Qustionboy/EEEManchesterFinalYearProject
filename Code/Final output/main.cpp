
#include "point_cloud_creation.h"
#include "PointCloudProcessor.h"
#include <chrono>
#include <iostream>

int main() {
    std::chrono::duration<double> Time;
   
    auto start1 = std::chrono::high_resolution_clock::now();
    PointCloudCreation creator;
    creator.drawStem();
    
    creator.ResultGenerator_KD();
    std::cout << std::endl;
    

    // Generate PGM file

    
    creator.SpaceCalculaterI();
    //creator.GenerateSlicePGMZ();
    creator.GenerateFullModelPGM("full_model_I.pgm");

    auto finish1 = std::chrono::high_resolution_clock::now();
    Time = finish1 - start1;



    std::cout << "Excution time for PGM with improvement: " << Time.count() << " seconds" << std::endl;

    system("pause");
    return 0;
}