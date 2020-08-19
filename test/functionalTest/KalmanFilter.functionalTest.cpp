//This code test the kalmanFilter library
//Developed by Ansu Man Singh
//Date :6/04/2017


#include"kalmanFilter.hpp"
#include<iostream>
#include<vector>
using namespace std;
using namespace kalman;

const double SampleTime =10e-3;
int main()
{
    //A system to test the kalman filter
    Matrix<double,Dynamic,Dynamic> A(4,4);
    A<<0.0212, -0.1072, 0.009423, -0.03216,
          0.3201, -0.1764, -0.02187, -0.0559,
          0.5571,  0.1096, -0.0492,  0.009316,
          0.1125,  0.06765, -0.007619, 0.01181;
    MatrixXd B(4,2);
        B<< -387.7, 193.9,
            1668,  -834.1,
            -2237,   1118,
            -1883,  941.6;

     MatrixXd C(2,4);
        C<< 50.85, 29.79, 16.14, -1.381,
           96.15, 39.02, 10.69, -1.047;
    MatrixXd Q(4,4);
        Q= MatrixXd::Identity(4,4);
    MatrixXd R(MatrixXd::Identity(2,2));
    MatrixXd x_prev(4,1);
    MatrixXd x_next(4,1);
    x_prev<<10,10,10,10;
    MatrixXd x_init(4,1);
    x_init<<0,0,0,0;
    kalman::kalmanFilter<double> Filter(4,2,2,A,B,C,Q,R,x_init);

    cout<<"Filter parameters"<<Filter;
    std::vector<Matrix<double,2,1> > inputArray;
    for(int i=0;i<1000;i++)
        {
         double input1,input2;
         input1 = 5*sin(2*M_PI*5*i*SampleTime);
         input2 = 7*sin(2*M_PI*5*i*SampleTime);
         Matrix<double,2,1> temp;
         temp<<input1,input2;
         inputArray.push_back(temp);

        }

    //Testing the actual response of the system vs the predicted one
    for(int i=0;i<1000;i++)
    {
        C*x_prev;
        cout<<"\ny_hat:"<<C*Filter.correct(C*x_prev,inputArray[i]);
        x_next=A*x_prev +B*(inputArray[i]);
        std::cout<<"\ny:"<<C*x_next;
        x_prev=x_next;

    }


    return 0;
}
