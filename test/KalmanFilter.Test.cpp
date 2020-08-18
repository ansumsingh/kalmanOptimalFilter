//This code test the kalmanFilter library
//Developed by Ansu Man Singh
//Date :6/04/2017


#include "KalmanFilter.hpp"
#include "Catch2/catch.hpp"

TEST_CASE("KalmanFilter")
{
    Kalman::KalmanFilter<double> filter{};

    REQUIRE(filter.numOfMeasurements() == 1);
    REQUIRE(filter.numOfStates() == 1);
    REQUIRE(filter.numOfInputs() == 1);
}