//This code test the kalmanFilter library
//Developed by Ansu Man Singh
//Date :6/04/2017

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "KalmanFilter.hpp"
#include "Catch2/catch.hpp"

namespace kalman::unittests{
TEST_CASE("KalmanFilter::KalmanFilter{}")
{
	auto filter = Kalman::KalmanFilter<double, 1, 1, 1>{};
    
    REQUIRE(filter.numOfMeasurements() == 1);
    REQUIRE(filter.numOfStates() == 1);
    REQUIRE(filter.numOfInputs() == 1);
    
    auto filter2 = Kalman::KalmanFilter<double, 2, 2, 2>{};

    REQUIRE(filter2.numOfMeasurements() == 2);
    REQUIRE(filter2.numOfStates() == 2);
    REQUIRE(filter2.numOfInputs() == 2);
}

}// namespace kalman::unittests