#include "KalmanMatrix.hpp"

#include <catch2/catch.hpp>

namespace kalman::unittests{

    TEST_CASE("KalmanMatrix::SystemMatrix")
    {
        auto matrixTest = kalman::KalmanMatrix<double, 2, 3>{};

        REQUIRE(matrixTest.rows() == 2);
        REQUIRE(matrixTest.cols() == 3);
    }
    
}// namespace kalman::unittests