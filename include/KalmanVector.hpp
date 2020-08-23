#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace kalman
{
    template <class Scalar, size_t ROW_DIM>
    class KalmanVector : public Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
    {
    public:
        using Base = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
        KalmanVector() : Base(ROW_DIM, 1) {}

        template <typename OtherDerived>
        KalmanVector &operator=(const Eigen::MatrixBase<OtherDerived> &other)
        {
            this->Base::operator=(other);
            return *this;
        }
    };

} // namespace kalman