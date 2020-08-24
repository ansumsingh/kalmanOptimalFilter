#include "KalmanMatrix.hpp"

#include <algorithm>
#include <catch2/catch.hpp>
#include <vector>

namespace kalman::unittests
{

	TEST_CASE("KalmanMatrix::KalmanMatrix")
	{
		auto matrixA = kalman::KalmanMatrix<double, 2, 3>{};

		REQUIRE(matrixA.rows() == 2);
		REQUIRE(matrixA.cols() == 3);

		auto matrixB = kalman::KalmanMatrix<double, 3, 3>{};

		REQUIRE(matrixB.rows() == 3);
		REQUIRE(matrixB.cols() == 3);
	}

	TEST_CASE("operator <<")
	{
		auto matrixTest = kalman::KalmanMatrix<double, 2, 3>{};
		auto result = std::vector<double>{ 1, 1, 1, 1, 1, 1 };

		matrixTest << 1, 1, 1, 1, 1, 1;
		auto vecMatrixTest = std::vector<double>{ matrixTest.data(), (matrixTest.data() + matrixTest.size()) };

		REQUIRE(std::equal(result.begin(), result.end(), vecMatrixTest.begin(), vecMatrixTest.end()));
	}

	TEST_CASE("operator ==")
	{
		auto result = GENERATE(
			std::vector<double>{ 1, 1, 1, 1, 1, 1 },
			std::vector<double>{ 1, 2, 1, 2, 1, 2 });

		auto matrixA = kalman::KalmanMatrix<double, 2, 3>{};
		auto matrixB = kalman::KalmanMatrix<double, 2, 3>{};
		auto matrixC = Eigen::Matrix<double, 2, 3>{};
		
		auto vecMatrixA = std::vector<double>{ matrixA.data(), (matrixA.data() + matrixA.size()) };
		auto vecMatrixB = std::vector<double>{ matrixB.data(), (matrixB.data() + matrixB.size()) };
		auto vecMatrixC = std::vector<double>{ matrixC.data(), (matrixC.data() + matrixC.size()) };
		
		std::transform(result.begin(), result.end(), matrixA.data(), [](const auto& val) {return val; });
		std::transform(result.begin(), result.end(), matrixB.data(), [](const auto& val) {return val; });
		std::transform(result.begin(), result.end(), matrixC.data(), [](const auto& val) {return val; });

		REQUIRE(matrixA == matrixB);
		REQUIRE(matrixA == matrixC);
		REQUIRE(matrixC == matrixB);
	}
} // namespace kalman::unittests