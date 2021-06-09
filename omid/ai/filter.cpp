#include "filter.h"

MatrixD MatrixCreator(int Row, int Col, double *Data)
{
    MatrixD temp( Row, Col );
    for( int i = 0; i < Row; i++ )
        for( int j = 0; j < Col; j++ )
            temp( i, j ) = Data[ j + i * Col ];

    return temp;
}

void Matrix2Array(MatrixD Input, int &Row, int &Col, double *Data)
{
    Row = Input.RowNo();
    Col = Input.ColNo();
    for( int i = 0; i < Input.RowNo(); i++ )
        for( int j = 0; j < Input.ColNo(); j++ )
            Data[ j + i * Col ] = Input( i, j );
}

/*****************************************************************************/
/********************* CLASS Kalman ******************************************/
/*****************************************************************************/
MatrixD Kalman::Update(MatrixD &Z)
{
    MatrixD I = Q;
    I.Unit();

    int _r, _c;
    double _data[5];
    Matrix2Array(!A, _r, _c, _data);

    // Time Update
    Wkm = A * Wk_1 + B;
    Pkm = A * Pk_1 * ~A + Q;

    // Measurement Update
    Kk = ( Pkm * ~H ) * !( H * Pkm * ~H  + R );
    Wk = Wkm + Kk * ( Z - H * Wkm );
    Pk = ( I - Kk * H ) * Pkm ;

    // Parameters Update
    Wk_1 = Wk;
    Pk_1 = Pk;

    return Wk;
}

MatrixD Kalman::Predict()
{
    return A * Wk_1 + B;
}

/*****************************************************************************/
/********************* CLASS BallKalmanFilter ********************************/
/*****************************************************************************/
BallKalmanFilter::BallKalmanFilter()
{
    double B[2] = { 0, 0 } ;
    double H[2] = { 1, 0 };
    double Q[4] = { 70.152, 7.108, 4.0990, 0.080092 };
    double R[1] = { 20011 };
    double Pk_1[4] = { 200.1, 0, 0, 200.1 };

    XFilter.B = MatrixCreator( 2, 1, B );
    XFilter.H = MatrixCreator( 1, 2, H );
    XFilter.Q = MatrixCreator( 2, 2, Q );
    XFilter.R = MatrixCreator( 1, 1, R );
    XFilter.Pk_1 = MatrixCreator( 2, 2, Pk_1 );

    YFilter.B = MatrixCreator( 2, 1, B );
    YFilter.H = MatrixCreator( 1, 2, H );
    YFilter.Q = MatrixCreator( 2, 2, Q );
    YFilter.R = MatrixCreator( 1, 1, R );
    YFilter.Pk_1 = MatrixCreator( 2, 2, Pk_1 );

    LastTimeCapture = -1;
}

void BallKalmanFilter::Update(double _TimeCapture, VecPosition _VisionPos, VecPosition &_FilterPos, VecPosition &_FilterVelosity)
{
    if( LastTimeCapture == -1 )
    {
        double Xk_1[2] = { _VisionPos.getX(), 0 };
        XFilter.Wk_1 = MatrixCreator( 2, 1, Xk_1 );

        double Yk_1[2] = { _VisionPos.getY(), 0 };
        YFilter.Wk_1 = MatrixCreator( 2, 1, Yk_1 );

        _FilterPos = _VisionPos;
        _FilterVelosity = VecPosition( 0, 0 );
    }
    else
    {
        double T = 1000 * ( _TimeCapture - LastTimeCapture );

        double _A[4] = { 1, T, 0, 1 };
        XFilter.A = MatrixCreator( 2, 2, _A );
        YFilter.A = MatrixCreator( 2, 2, _A );

        double _Zx[1] = { _VisionPos.getX() };
        MatrixD Zx = MatrixCreator( 1, 1, _Zx );
        MatrixD Fx = XFilter.Update( Zx );

        double _Zy[1] = { _VisionPos.getY() };
        MatrixD Zy = MatrixCreator( 1, 1, _Zy );
        MatrixD Fy = YFilter.Update( Zy );

        VecPosition temp_pos_output( Fx(0, 0), Fy(0, 0) );
        if( temp_pos_output.getDistanceTo( _VisionPos ) > 32 )
        {
            temp_pos_output = _VisionPos;

            double Xk_1[2] = { _VisionPos.getX(), 0 };
            XFilter.Wk_1 = MatrixCreator( 2, 1, Xk_1 );

            double Yk_1[2] = { _VisionPos.getY(), 0 };
            YFilter.Wk_1 = MatrixCreator( 2, 1, Yk_1 );

            double Pk_1[4] = { 200.1, 0, 0, 200.1 };
            XFilter.Pk_1 = MatrixCreator( 2, 2, Pk_1 );
            YFilter.Pk_1 = MatrixCreator( 2, 2, Pk_1 );
        }

        _FilterPos = temp_pos_output;
        _FilterVelosity = VecPosition( Fx(1, 0), Fy(1, 0) );
    }

    LastTimeCapture = _TimeCapture;
}

void BallKalmanFilter::Predict(double _PredictTime, VecPosition &_PredictPos, VecPosition &_PredictVelosity)
{
    if( LastTimeCapture != -1 )
    {
        double T = 1000 * ( _PredictTime - LastTimeCapture );
        double _A[4] = { 1, T, 0, 1 };
        XFilter.A = MatrixCreator( 2, 2, _A );
        YFilter.A = MatrixCreator( 2, 2, _A );

        MatrixD Fx = XFilter.Predict();
        MatrixD Fy = XFilter.Predict();

        _PredictPos = VecPosition( Fx(0, 0), Fy(0, 0) );
        _PredictVelosity = VecPosition( Fx(1, 0), Fy(1, 0) );
    }
    else
        ;//logStatus( " Ball Kalman Filter: You Call Predict Without Input Data ", QColor("red") );
}

/*****************************************************************************/
/********************* CLASS RobotKalmanFilter *******************************/
/*****************************************************************************/
RobotKalmanFilter::RobotKalmanFilter()
{
    double B[2] = { 0, 0 };
    double H[2] = { 1, 0 };
    double Q[4] = { 120.152, 12.108, 9.0990, 0.080092 };
    double R[1] = { 500 };
    double Pk_1[4] = { 210, 0, 0, 210 };

    XFilter.B = MatrixCreator( 2, 1, B );
    XFilter.H = MatrixCreator( 1, 2, H );
    XFilter.Q = MatrixCreator( 2, 2, Q );
    XFilter.R = MatrixCreator( 1, 1, R );
    XFilter.Pk_1 = MatrixCreator( 2, 2, Pk_1 );

    YFilter.B = MatrixCreator( 2, 1, B );
    YFilter.H = MatrixCreator( 1, 2, H );
    YFilter.Q = MatrixCreator( 2, 2, Q );
    YFilter.R = MatrixCreator( 1, 1, R );
    YFilter.Pk_1 = MatrixCreator( 2, 2, Pk_1 );

    double Ang_B[2] = { 0, 0 } ;
    double Ang_H[2] = { 1, 0 };
    double Ang_Q[4] = { 120.152, 12.108, 9.0990, 0.080092 };
    double Ang_R[1] = { 500 };
    double Ang_Pk_1[4] = { 210, 0, 0, 210 };

    AngFilter.B = MatrixCreator( 2, 1, Ang_B );
    AngFilter.H = MatrixCreator( 1, 2, Ang_H );
    AngFilter.Q = MatrixCreator( 2, 2, Ang_Q );
    AngFilter.R = MatrixCreator( 1, 1, Ang_R );
    AngFilter.Pk_1 = MatrixCreator( 2, 2, Ang_Pk_1 );

    LastTimeCapture = -1;
}

void RobotKalmanFilter::Update(double _TimeCapture, VecPosition _VisionPos, double _VisionAng, VecPosition &_FilterPos, double &_FilterAng, VecPosition &_FilterPosVelosity, double &_FilterAngVelosity)
{
    if( LastTimeCapture == -1 )
    {
        double Xk_1[2] = { _VisionPos.getX(), 0 };
        XFilter.Wk_1 = MatrixCreator( 2, 1, Xk_1 );

        double Yk_1[2] = { _VisionPos.getY(), 0 };
        YFilter.Wk_1 = MatrixCreator( 2, 1, Yk_1 );

        double Angk_1[2] = { _VisionAng, 0 };
        AngFilter.Wk_1 = MatrixCreator( 2, 1, Angk_1 );

        _FilterPos = _VisionPos;
        _FilterAng = _VisionAng;
        _FilterPosVelosity = VecPosition( 0, 0 );
        _FilterAngVelosity = 0;
    }
    else
    {
        double T = 1000 * ( _TimeCapture - LastTimeCapture );

        //logStatus(string::number(T),"orange");
        double _A[4] = { 1, T, 0, 1 };
        XFilter.A = MatrixCreator( 2, 2, _A );
        YFilter.A = MatrixCreator( 2, 2, _A );
        AngFilter.A = MatrixCreator( 2, 2, _A );

        double _Zx[1] = { _VisionPos.getX() };
        MatrixD Zx = MatrixCreator( 1, 1, _Zx );
        MatrixD Fx = XFilter.Update( Zx );

        double _Zy[1] = { _VisionPos.getY() };
        MatrixD Zy = MatrixCreator( 1, 1, _Zy );
        MatrixD Fy = YFilter.Update( Zy );

        double _ZAng[1] = { _VisionAng };
        MatrixD ZAng = MatrixCreator( 1, 1, _ZAng );
        MatrixD FAng = AngFilter.Update( ZAng );

        VecPosition temp_pos_output( Fx(0, 0), Fy(0, 0) );
        if( temp_pos_output.getDistanceTo( _VisionPos ) > 32 )
        {
            temp_pos_output = _VisionPos;

            double Xk_1[2] = { _VisionPos.getX(), Fx(1, 0) };
            XFilter.Wk_1 = MatrixCreator( 2, 1, Xk_1 );

            double Yk_1[2] = { _VisionPos.getY(), Fy(1, 0) };
            YFilter.Wk_1 = MatrixCreator( 2, 1, Yk_1 );

            double Pk_1[4] = { 200.1, 0, 0, 200.1 };
            XFilter.Pk_1 = MatrixCreator( 2, 2, Pk_1 );
            YFilter.Pk_1 = MatrixCreator( 2, 2, Pk_1 );
        }

        _FilterPos = temp_pos_output;
        _FilterAng = FAng(0, 0);
        _FilterPosVelosity = VecPosition( Fx(1, 0), Fy(1, 0) );
        _FilterAngVelosity = FAng(1, 0);
    }

    LastTimeCapture = _TimeCapture;
}

void RobotKalmanFilter::Predict(double _PredictTime, VecPosition &_PredictPos, double &_PredictAng, VecPosition &_PredictPosVelosity, double &_PredictAngVelosity)
{
    if( LastTimeCapture != -1 )
    {
        double T = 1000 * ( _PredictTime - LastTimeCapture );
        double _A[4] = { 1, T, 0, 1 };
        XFilter.A = MatrixCreator( 2, 2, _A );
        YFilter.A = MatrixCreator( 2, 2, _A );
        AngFilter.A = MatrixCreator( 2, 2, _A );

        MatrixD Fx = XFilter.Predict();
        MatrixD Fy = XFilter.Predict();
        MatrixD FAng = AngFilter.Predict();

        _PredictPos = VecPosition( Fx(0, 0), Fy(0, 0) );
        _PredictAng = FAng(0, 0);
        _PredictPosVelosity = VecPosition( Fx(1, 0), Fy(1, 0) );
        _PredictAngVelosity = FAng(1, 0);
    }
    else
        ;//logStatus( " Robot Kalman Filter: You Call Predict Without Input Data ", QColor("red") );
}
