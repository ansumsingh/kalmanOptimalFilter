//Kalman Filter using the Eigen3 library
//Developed by Ansu Man Singh
//Email: ansumsingh@gmail.com

#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Core>
#include<iostream>


namespace Kalman {
using namespace Eigen;

template<typename T>
class KalmanFilter{
typedef Matrix<T,Dynamic,Dynamic> kalmanMatrix;
public:
    //default constructor
    KalmanFilter(int states=1,int measurements=1,int inputs=1):
        states(states),
        measurements(measurements),
        inputs(inputs),
        A(states,states),
        B(states,inputs),
        C(measurements,states),
        Q(states,states),
        R(measurements,measurements),
        P(states,states),
        pre_state(states,1),
        post_state(states,1){};
    //
    //Generalized constructor
    template<typename D>
    kalmanFilter(const KalmanFilter<D> &other):
        states(other.getNoOfStates()),
        measurements(other.getNoOfMeasurements()),
        inputs(other.getNoOfInputs()),
        A(other.getMatrixA().template cast<T>()),
        B(other.getMatrixB().template cast<T>()),
        C(other.getMatrixC().template cast<T>()),
        Q(other.getMatrixQ().template cast<T>()),
        R(other.getMatrixR().template cast<T>()),
        P(other.getMatrixP().template cast<T>()),
        pre_state(other.getPreState().template cast<T>()),
        post_state(other.getPostState().template cast<T>())
        {

        }

    KalmanFilter(int states,
                  int measurements,
                  int inputs,
                  const kalmanMatrix &A,
                  const kalmanMatrix &B,
                  const kalmanMatrix &C,
                  const kalmanMatrix &Q,
                  const kalmanMatrix &R,
                  const kalmanMatrix &pre_state):
        states(states),
        measurements(measurements),
        inputs(inputs),
        A(A),
        B(B),
        C(C),
        Q(Q),
        R(R),
        P(kalmanMatrix::Identity(states,states)),
        pre_state(pre_state)
        {
            assert((states>0)&&(measurements>0)&&(inputs>0)&&
                   (A.cols()==states)&&(A.rows()==states)&&
                   (B.rows()==states)&&(B.cols()==inputs)&&
                   (C.cols()==states)&&(C.rows()==measurements)&&
                   (Q.rows()==states)&&(Q.cols()==states)&&
                   (R.cols()==measurements)&&(R.rows()==measurements)&&
                   (pre_state.rows()==states)&&(pre_state.cols()==1));
        }



    int  getNoOfStates()const {return states;}
    int  getNoOfMeasurements()const{return measurements;}
    int  getNoOfInputs()const{return inputs;}
    const kalmanMatrix& getMatrixA()const{return A;}
    const kalmanMatrix& getMatrixB()const {return B;}
    const kalmanMatrix& getMatrixC()const {return C;}
    const kalmanMatrix& getMatrixQ()const{return Q;}
    const kalmanMatrix& getMatrixR()const{return R;}
    const kalmanMatrix& getMatrixP()const{return P;}
    const kalmanMatrix& getPreState()const{return pre_state;}
    const kalmanMatrix& getPostState()const{return post_state;}
    //set the transitional Matrix;
    void setMatrixA(const kalmanMatrix &A_ip)
                    {   assert((A.cols()==states)&&(A.rows()==states));
                        A=A_ip;}
    //set the Input matrix
    void setMatrixB(const kalmanMatrix &B_ip)
                    {   assert((B.cols()==inputs)&&(A.rows()==states));
                        B=B_ip;}
    //Set the Output matrix
    void setMatrixC(const kalmanMatrix &C_ip)
                    {   assert((C.cols()==measurements)&&(C.rows()==states));
                        C=C_ip;}
    //Set the process noise covariance
    void setMatrixQ(const kalmanMatrix &Q_ip)
                    {   assert((Q.cols()==states)&&(Q.rows()==states));
                        Q=Q_ip;}
    //Set the measurement noise covariance
    void setMatrixR(const kalmanMatrix &R_ip)
                    {   assert((R.cols()==measurements)&&(Q.rows()==measurements));
                        R=R_ip;}
    //Set the pre state
    void setMatrixPreState(const kalmanMatrix &pre_state_ip)
                    {   assert((R.cols()==measurements)&&(Q.rows()==measurements));
                        pre_state=pre_state_ip;}

    //Predict the value
    const kalmanMatrix& predict(const kalmanMatrix& input)
                     { assert(input.rows()==inputs);
                       P = A*P*Transpose<kalmanMatrix>(A)+Q;
                       post_state = A*pre_state +B*input;
                       pre_state = post_state;
                       return post_state;
    }
    //Correction steps
    const kalmanMatrix& correct(const kalmanMatrix& measurement,const kalmanMatrix& input)
                     {
                        assert(input.rows()==inputs&&measurement.rows()==measurements);

                        P = A*P*A.transpose()+Q;
                        //Inovation and optimal kalman gain calculation
                        const kalmanMatrix K = P*C.transpose()*(C*P*C.transpose()+R).inverse();
                        //Update or the correction step
                        pre_state = pre_state+K*(measurement-C*pre_state);
                        //Prediction
                        post_state = A*pre_state+B*input;
                        //update covariance step
                        P =(kalmanMatrix::Identity(states,states)-K*C)*P;
                        pre_state=post_state;
                        return post_state;
                        }

    //For displaying the filter parameters, For Why friend please refer to Scott Mayer
    friend std::ostream& operator<<(std::ostream& os,const KalmanFilter<T>& filter)
    {
        printFilter(filter);
        return os;
    }


private:
        int states;
        int measurements;
        int inputs;
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
template<typename W>
void printFilter(const KalmanFilter<W> &filter)
{
    std::cout<<std::endl<<"Transitional matrix:"<<std::endl<<filter.getMatrixA();
    std::cout<<std::endl<<"Input matrix:"<<std::endl<<filter.getMatrixB();
    std::cout<<std::endl<<"Measurement matrix:"<<std::endl<<filter.getMatrixC();
    std::cout<<std::endl<<"Process noise co-variance matrix:"<<std::endl<<filter.getMatrixQ();
    std::cout<<std::endl<<"Measurment noise co-variance matrix:"<<std::endl<<filter.getMatrixR();
}

}

#endif
