//This code test the kalmanFilter library
//Developed by Ansu Man Singh
//Date :6/04/2017


#include"kalmanFilter.hpp"

TEST_CASE("KalmanFilter")
{
    auto filter = Kalman::KalmanFilter{}

    REQUIRE(filter.numOfMeasurements() == 1);
    REQUIRE(filter.numOfStates() == 1);
    REQUIRE(filter.numOfOutputs() == 1);
    REQUIRE(filter.numOfInputs() == 1);
}