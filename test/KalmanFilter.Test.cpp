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
	auto filter = kalman::KalmanFilter<double, 1, 1, 1>{};
    
    REQUIRE(filter.numOfMeasurements() == 1);
    REQUIRE(filter.numOfStates() == 1);
    REQUIRE(filter.numOfInputs() == 1);
    
    auto filter2 = kalman::KalmanFilter<double, 2, 2, 2>{};

    REQUIRE(filter2.numOfMeasurements() == 2);
    REQUIRE(filter2.numOfStates() == 2);
    REQUIRE(filter2.numOfInputs() == 2);

    SECTION("KalmanFilter{A, B, C, Q, R, preStatus}")
    {
        auto A = kalman::KalmanMatrix<double, 3, 3>{};
        A << 1, 1, 1, 
             1, 1, 1, 
             1, 1, 1;

        auto B = kalman::KalmanMatrix<double, 3, 2>{};
        B << 1, 1,
             1, 1,
             1, 1;
        
        auto C = kalman::KalmanMatrix<double, 2, 3>{};
        C << 1, 1, 1,
             1, 1, 1;
        
        auto Q = kalman::KalmanMatrix<double, 3, 3>{};
        Q << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        
        auto R = kalman::KalmanMatrix<double, 2, 2>{};

        
        auto preStates = kalman::KalmanVector<double, 3>{};
        auto postStates = kalman::KalmanVector<double, 3>{};

        auto filter = kalman::KalmanFilter<double, 3, 2, 2>{A, B, C, Q, R, preStates};

    }
}

}// namespace kalman::unittests