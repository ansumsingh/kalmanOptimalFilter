#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace kalman
{
    template <class Scalar, size_t ROW_DIM, size_t COL_DIM>
    class KalmanMatrix : public Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
    {
      public:
        using Base = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
        KalmanMatrix() : Base(ROW_DIM, COL_DIM) {}

        KalmanMatrix(const KalmanMatrix<Scalar, ROW_DIM, COL_DIM>& other) : Base(other)  {}
        KalmanMatrix(KalmanMatrix<Scalar, ROW_DIM, COL_DIM>&& other) : Base(other)  {}
        
        template <typename OtherDerived>
        KalmanMatrix(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {
            static_assert(other.RowsAtCompileTime == ROW_DIM, "other.rows != ROW_DIM");
            static_assert(other.ColsAtCompileTime == COL_DIM, "other.cols != COL_DIM");
        }
        
        template <typename OtherDerived>
        KalmanMatrix(Eigen::MatrixBase<OtherDerived>&& other) : Base(other) {
            static_assert(other.RowsAtCompileTime == ROW_DIM, "other.rows != ROW_DIM");
            static_assert(other.ColsAtCompileTime == COL_DIM, "other.cols != COL_DIM");
        }
        
        template <typename OtherDerived>
        KalmanMatrix& operator=(const Eigen::MatrixBase<OtherDerived>& other)
        {
            this->Base::operator=(other);
            return *this;
        }
    };
}// namespace kalman