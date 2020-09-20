//Kalman Filter using the Eigen3 library
//Developed by Ansu Man Singh
//Email: ansumsingh@gmail.com

#pragma once

#include "KalmanMatrix.hpp"
#include "KalmanVector.hpp"
#include <ostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <type_traits>




namespace kalman
{
    using namespace Eigen;
    template <typename T, size_t STATES_DIM, size_t INPUT_DIM, size_t MEASUREMENT_DIM>
    class KalmanFilter
    {

    public:
        using kalmanStatesMatrix = kalman::KalmanMatrix<T, STATES_DIM, STATES_DIM>;
        using kalmanInputMatrix = kalman::KalmanMatrix<T, STATES_DIM, INPUT_DIM>;
        using kalmanOutputMatrix = kalman::KalmanMatrix<T, MEASUREMENT_DIM, STATES_DIM>;
        using kalmanMeasurementMatrix = kalman::KalmanMatrix<T, MEASUREMENT_DIM, MEASUREMENT_DIM>;
        using kalmanMeasurementVector = kalman::KalmanVector<T, MEASUREMENT_DIM>;
        using kalmanStatesVector = kalman::KalmanVector<T, STATES_DIM>;
        //default constructor
        KalmanFilter() : A{},
                         B{},
                         C{},
                         Q{},
                         R{},
                         P{},
                         pre_state{},
                         post_state{}
                         {};
        //
        //Generalized constructor
        // At the moment copy constructor is disabled
        KalmanFilter(const KalmanFilter<T, STATES_DIM, INPUT_DIM, MEASUREMENT_DIM> &filter) = delete;
        KalmanFilter& operator= (const KalmanFilter<T, STATES_DIM, INPUT_DIM, MEASUREMENT_DIM> &filter) = delete;
        
        template <typename D,int i, int j, int k>
        KalmanFilter(const KalmanFilter<D, i, j, k> &other) = delete;
        
        template <typename D,int i, int j, int k>
        KalmanFilter& operator= (const KalmanFilter<D, i, j, k> &other) = delete;
        
        template <typename D,int i, int j, int k>
        KalmanFilter& operator= (KalmanFilter<D, i, j ,k> &&other) = delete;
        
        KalmanFilter(const kalmanStatesMatrix &A,
                     const kalmanInputMatrix &B,
                     const kalmanOutputMatrix &C,
                     const kalmanStatesMatrix &Q,
                     const kalmanMeasurementMatrix &R,
                     const kalmanStatesVector &pre_state) : A(A),
                                                      B(B),
                                                      C(C),
                                                      Q(Q),
                                                      R(R),
                                                      P(Eigen::Matrix<double, STATES_DIM, STATES_DIM>::Identity()),
                                                      pre_state(pre_state)
        {
            static_assert(STATES_DIM > 0, "STATES_DIM should be greater than 0");
            static_assert(MEASUREMENT_DIM > 0, "MEASUREMENT_DIM should be greater than 0");
            static_assert(INPUT_DIM > 0, "INPUT_DIM should be greater than 0");
        }

        inline int numOfStates() const { return STATES_DIM; }
        inline int numOfMeasurements() const { return MEASUREMENT_DIM; }
        inline int numOfInputs() const { return INPUT_DIM; }
        
        inline const kalmanStatesMatrix &matrixA() const { return A; }
        inline const kalmanInputMatrix &matrixB() const { return B; }
        inline const kalmanOutputMatrix &matrixC() const { return C; }
        inline const kalmanStatesMatrix &matrixQ() const { return Q; }
        inline const kalmanMeasurementMatrix &matrixR() const { return R; }
        inline const kalmanStatesMatrix &matrixP() const { return P; }
        inline const kalmanStatesVector &preState() const { return pre_state; }
        inline const kalmanStatesVector &postState() const { return post_state; }
        //set the transitional Matrix;
        void setMatrixA(const kalmanStatesMatrix &A_ip)
        {
            assert((A.cols() == STATES_DIM) && (A.rows() == STATES_DIM));
            A = A_ip;
        }
        //set the Input matrix
        void setMatrixB(const kalmanInputMatrix &B_ip)
        {
            assert((B.cols() == inputs) && (A.rows() == states));
            B = B_ip;
        }
        //Set the Output matrix
        void setMatrixC(const kalmanOutputMatrix &C_ip)
        {
            assert((C.cols() == measurements) && (C.rows() == states));
            C = C_ip;
        }
        //Set the process noise covariance
        void setMatrixQ(const kalmanStatesMatrix &Q_ip)
        {
            assert((Q.cols() == states) && (Q.rows() == states));
            Q = Q_ip;
        }
        //Set the measurement noise covariance
        void setMatrixR(const kalmanMeasurementMatrix &R_ip)
        {
            assert((R.cols() == measurements) && (Q.rows() == measurements));
            R = R_ip;
        }
        //Set the pre state
        void setMatrixPreState(const kalmanStatesVector &pre_state_ip)
        {
            assert((R.cols() == measurements) && (Q.rows() == measurements));
            pre_state = pre_state_ip;
        }

        //Predict the value
        const kalmanStatesVector &predict(const kalmanStatesVector &input)
        {
            assert(input.rows() == inputs);
            P = A * P * Transpose<kalmanStatesMatrix>(A) + Q;
            post_state = A * pre_state + B * input;
            pre_state = post_state;
            return post_state;
        }
        //Correction steps
        const kalmanStatesVector &correct(const kalmanMeasurementVector &measurement, const kalmanStatesVector &input)
        {
            assert(input.rows() == inputs && measurement.rows() == measurements);

            P = A * P * A.transpose() + Q;
            //Inovation and optimal kalman gain calculation
            const kalmanMatrix K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
            //Update or the correction step
            pre_state = pre_state + K * (measurement - C * pre_state);
            //Prediction
            post_state = A * pre_state + B * input;
            //update covariance step
            P = (kalmanMatrix::Identity(states, states) - K * C) * P;
            pre_state = post_state;
            return post_state;
        }

        //For displaying the filter parameters, For Why friend please refer to Scott Mayer
        friend std::ostream &operator<<(std::ostream &os, const KalmanFilter<T, STATES_DIM, MEASUREMENT_DIM, INPUT_DIM> &filter)
        {
            printFilter(os, filter);
            return os;
        }

    private:
        kalmanStatesMatrix A;
        kalmanInputMatrix B;
        kalmanOutputMatrix C;
        kalmanStatesMatrix Q;
        kalmanMeasurementMatrix R;
        kalmanStatesMatrix P;
        kalmanStatesVector pre_state;
        kalmanStatesVector post_state;
    };

    //For displaying all the parameters of the filter
    template <typename W, int i, int j, int k>
    void printFilter(std::ostream &os, const KalmanFilter<W, i, j, k> &filter)
    {
        os << std::endl
           << "Transitional matrix:" << std::endl
           << filter.getMatrixA();
        os << std::endl
           << "Input matrix:" << std::endl
           << filter.getMatrixB();
        os << std::endl
           << "Measurement matrix:" << std::endl
           << filter.getMatrixC();
        os << std::endl
           << "Process noise co-variance matrix:" << std::endl
           << filter.getMatrixQ();
        os << std::endl
           << "Measurment noise co-variance matrix:" << std::endl
           << filter.getMatrixR();
    }

} // namespace Kalman


