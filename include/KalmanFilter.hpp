//Kalman Filter using the Eigen3 library
//Developed by Ansu Man Singh
//Email: ansumsingh@gmail.com

#pragma once

#include <ostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <type_traits>




namespace Kalman
{
    using namespace Eigen;

    template <typename T, int numStates, int numInputs, int numMeasurements>
    class KalmanFilter
    {
        typedef Matrix<T, Dynamic, Dynamic> kalmanMatrix;

    public:
        //default constructor
        KalmanFilter() : A(numStates, numStates),
                         B(numStates, numInputs),
                         C(numMeasurements, numStates),
                         Q(numStates, numStates),
                         R(numMeasurements, numMeasurements),
                         P(numStates, numStates),
                         pre_state(numStates, 1),
                         post_state(numStates, 1)
                         {};
        //
        //Generalized constructor
        // At the moment copy constructor is disabled
        KalmanFilter(const KalmanFilter<T, numStates, numInputs, numMeasurements> &filter) = delete;
        KalmanFilter& operator= (const KalmanFilter<T, numStates, numInputs, numMeasurements> &filter) = delete;
        KalmanFilter& operator= (KalmanFilter<T, numStates, numInputs, numMeasurements> &filter) = delete;
        
        template <typename D,int i, int j, int k>
        KalmanFilter(const KalmanFilter<D, i, j, k> &other) = delete;
        
        template <typename D,int i, int j, int k>
        KalmanFilter& operator= (const KalmanFilter<D, i, j, k> &other) = delete;
        
        template <typename D,int i, int j, int k>
        KalmanFilter& operator= (KalmanFilter<D, i, j ,k> &&other) = delete;
        
        KalmanFilter(const kalmanMatrix &A,
                     const kalmanMatrix &B,
                     const kalmanMatrix &C,
                     const kalmanMatrix &Q,
                     const kalmanMatrix &R,
                     const kalmanMatrix &pre_state) : A(A),
                                                      B(B),
                                                      C(C),
                                                      Q(Q),
                                                      R(R),
                                                      P(kalmanMatrix::Identity(numStates, numStates)),
                                                      pre_state(pre_state)
        {
            static_assert((numOfStates > 0) && (numMeasurements > 0) && (numInputs > 0) &&
                          (A.cols() == numStates) && (A.rows() == numStates) &&
                          (B.rows() == numStates) && (B.cols() == numInputs) &&
                          (C.cols() == numStates) && (C.rows() == numMeasurements) &&
                          (Q.rows() == numStates) && (Q.cols() == numStates) &&
                          (R.cols() == numMeasurements) && (R.rows() == numMeasurements) &&
                          (pre_state.rows() == numStaes) && (pre_state.cols() == 1));
        }

        int numOfStates() const { return numStates; }
        int numOfMeasurements() const { return numMeasurements; }
        int numOfInputs() const { return numInputs; }
        
        const kalmanMatrix &getMatrixA() const { return A; }
        const kalmanMatrix &getMatrixB() const { return B; }
        const kalmanMatrix &getMatrixC() const { return C; }
        const kalmanMatrix &getMatrixQ() const { return Q; }
        const kalmanMatrix &getMatrixR() const { return R; }
        const kalmanMatrix &getMatrixP() const { return P; }
        const kalmanMatrix &getPreState() const { return pre_state; }
        const kalmanMatrix &getPostState() const { return post_state; }
        //set the transitional Matrix;
        void setMatrixA(const kalmanMatrix &A_ip)
        {
            assert((A.cols() == states) && (A.rows() == states));
            A = A_ip;
        }
        //set the Input matrix
        void setMatrixB(const kalmanMatrix &B_ip)
        {
            assert((B.cols() == inputs) && (A.rows() == states));
            B = B_ip;
        }
        //Set the Output matrix
        void setMatrixC(const kalmanMatrix &C_ip)
        {
            assert((C.cols() == measurements) && (C.rows() == states));
            C = C_ip;
        }
        //Set the process noise covariance
        void setMatrixQ(const kalmanMatrix &Q_ip)
        {
            assert((Q.cols() == states) && (Q.rows() == states));
            Q = Q_ip;
        }
        //Set the measurement noise covariance
        void setMatrixR(const kalmanMatrix &R_ip)
        {
            assert((R.cols() == measurements) && (Q.rows() == measurements));
            R = R_ip;
        }
        //Set the pre state
        void setMatrixPreState(const kalmanMatrix &pre_state_ip)
        {
            assert((R.cols() == measurements) && (Q.rows() == measurements));
            pre_state = pre_state_ip;
        }

        //Predict the value
        const kalmanMatrix &predict(const kalmanMatrix &input)
        {
            assert(input.rows() == inputs);
            P = A * P * Transpose<kalmanMatrix>(A) + Q;
            post_state = A * pre_state + B * input;
            pre_state = post_state;
            return post_state;
        }
        //Correction steps
        const kalmanMatrix &correct(const kalmanMatrix &measurement, const kalmanMatrix &input)
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
        friend std::ostream &operator<<(std::ostream &os, const KalmanFilter<T, numStates, numMeasurements, numInputs> &filter)
        {
            printFilter(os, filter);
            return os;
        }

    private:
        kalmanMatrix A;
        kalmanMatrix B;
        kalmanMatrix C;
        kalmanMatrix Q;
        kalmanMatrix R;
        kalmanMatrix P;
        kalmanMatrix pre_state;
        kalmanMatrix post_state;
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


