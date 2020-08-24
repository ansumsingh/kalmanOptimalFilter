#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace kalman
{
    template <class T, size_t ROW_DIM, size_t COL_DIM>
    class KalmanMatrix : public Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
    {
      public:
        using Base = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
        KalmanMatrix() : Base(ROW_DIM, COL_DIM) {}
        
        template <typename OtherDerived>
        KalmanMatrix& operator=(const Eigen::MatrixBase<OtherDerived>& other)
        {
            this->Base::operator=(other);
            return *this;
        }
    };
}// namespace kalman