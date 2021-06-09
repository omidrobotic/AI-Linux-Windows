#ifndef FILTER_H
#define FILTER_H

#include "geometry.h"
#include "matrix.h"
using namespace math;
typedef matrix<double> MatrixD;

MatrixD MatrixCreator( int Row, int Col, double *Data );
void Matrix2Array( MatrixD Input, int &Row, int &Col, double *Data );

/*****************************************************************************/
/********************* CLASS Kalman ******************************************/
/*****************************************************************************/
class Kalman
{
    // Wk = A * Wk_1 + B
    // Zk  = H * Wk
public:
    MatrixD Update(MatrixD &Z);
    MatrixD Predict();

    MatrixD A, B, H;
    MatrixD Q, R;
    MatrixD Wk, Wk_1, Wkm;
    MatrixD Pk, Pk_1, Pkm;
    MatrixD Kk;
};

/*****************************************************************************/
/********************* CLASS BallKalmanFilter ********************************/
/*****************************************************************************/
class BallKalmanFilter
{
public:
    BallKalmanFilter();
    void Update( double _TimeCapture, VecPosition _VisionPos, VecPosition &_FilterPos, VecPosition &_FilterVelosity );
    void Predict( double _PredictTime, VecPosition &_PredictPos, VecPosition &_PredictVelosity );

private:
    Kalman XFilter, YFilter;
    double LastTimeCapture;

};

/*****************************************************************************/
/********************* CLASS RobotKalmanFilter *******************************/
/*****************************************************************************/
class RobotKalmanFilter
{
public:
    RobotKalmanFilter();
    void Update( double _TimeCapture, VecPosition _VisionPos, double _VisionAng, VecPosition &_FilterPos, double &_FilterAng, VecPosition &_FilterPosVelosity, double &_FilterAngVelosity );
    void Predict( double _PredictTime, VecPosition &_PredictPos, double &_PredictAng, VecPosition &_PredictPosVelosity, double &_PredictAngVelosity );

private:
   Kalman XFilter, YFilter, AngFilter;
   double LastTimeCapture;

};

#endif // FILTER_H
