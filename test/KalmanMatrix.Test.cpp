#include "KalmanMatrix.hpp"

#include <algorithm>
#include <catch2/catch.hpp>
#include <vector>

namespace kalman::unittests
{

	TEST_CASE("KalmanMatrix::KalmanMatrix")
	{
		auto matrixA = kalman::KalmanMatrix<double, 2, 3>{};
		
		auto matrixA2 = matrixA;

		REQUIRE(matrixA == matrixA2);
						   

		auto matrixB = kalman::KalmanMatrix<double, 3, 3>{};

		REQUIRE(matrixB.rows() == 3);
		REQUIRE(matrixB.cols() == 3);

		SECTION("KalmanMatrix<Scalar..>(const KalmanMatrix<Scalar..>&)")
		{
			kalman::KalmanMatrix<double, 2, 3> matrixA2 = matrixA;
			
			REQUIRE(matrixA2.rows() == 2);
			REQUIRE(matrixA2.cols() == 3);
		}
		

		SECTION("KalmanMatrix<Scalar..>(KalmanMatrix<Scalar..>&&")
		{
			auto matrixA3 = std::move(matrixA);
			
			REQUIRE(matrixA3.rows() == 2);
			REQUIRE(matrixA3.cols() == 3);
		}

		SECTION("KalmanMatrix<Scalar..>(const OtherDerived&)")
		{
			auto matrixOrig = Eigen::Matrix<double, 3, 3>{};
			matrixOrig.setIdentity();
			kalman::KalmanMatrix<double, 3, 3> matrixC = matrixOrig;
			
			REQUIRE(matrixC(0, 0) == 1);
			REQUIRE(matrixC(1, 1) == 1);
			REQUIRE(matrixC(2, 2) == 1);
		}

		SECTION("KalmanMatrix<Scalar..>(OtherDerived&&)")
		{
			kalman::KalmanMatrix<double, 3, 3> matrixC = Eigen::Matrix<double, 3, 3>::Identity();
			// kalman::KalmanMatrix<double, 3, 3> matrixC = Eigen::MatrixXd::Identity(3, 3); This fails the compilation cause rows and columns for MatrixXd is not known at compile time 
			
			REQUIRE(matrixC(0, 0) == 1);
			REQUIRE(matrixC(1, 1) == 1);
			REQUIRE(matrixC(2, 2) == 1);
		}
	}

	

	TEST_CASE("KalmanMatrix::KalmanMatrix operator <<")
	{
		auto matrixTest = kalman::KalmanMatrix<double, 2, 3>{};
		auto result = std::vector<double>{ 1, 1, 1, 1, 1, 1 };

		matrixTest << 1, 1, 1, 1, 1, 1;
		auto vecMatrixTest = std::vector<double>{matrixTest.data(), (matrixTest.data() + matrixTest.size())};

		REQUIRE(std::equal(result.begin(), result.end(), vecMatrixTest.begin(), vecMatrixTest.end()));
	}

	TEST_CASE("KalmanMatrix::KalmanMatrix operator ==")
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

	TEST_CASE("KalmanMatrix::KalmanMatrix operator*")
	{
		auto matrixA = kalman::KalmanMatrix<double,2,3>{};
		auto matrixB = kalman::KalmanMatrix<double,3,2>{};

		auto result = std::vector<double>{2, 1, 1, 2};

		matrixA<< 1, 0, 1,
				  0, 1, 1;
		
		matrixB<< 1, 0,
				  0, 1,
				  1, 1;

		auto resultMatrix = kalman::KalmanMatrix<double, 2, 3>{};
		resultMatrix = matrixA*matrixB;
		
		REQUIRE(resultMatrix.rows()==2);
		REQUIRE(resultMatrix.cols()==2);
		
		auto resultMatrixVec = std::vector<double>{resultMatrix.data(), (resultMatrix.data() + resultMatrix.size())};
		REQUIRE(std::equal(result.begin(), result.end(), resultMatrixVec.begin(), resultMatrixVec.end()));

	}
} // namespace kalman::unittests