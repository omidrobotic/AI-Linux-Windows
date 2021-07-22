#ifndef FIELDGEOMETRY_H
#define FIELDGEOMETRY_H

#include "geometry.h"
#include "Switches.h"

/*Lab Field
	//const float WholeFieldLength = 10400;
	//const float WholeFieldWidth = 7400;
	//const float FieldLength = 8000;
	//const float FieldWidth = 6000;
	//const float FreeBound = 500;
	//const float GoalLength = 1000;
	//const float CornelPoint = 4000;
	//const float PenaltyPoint = 1000;
	//const float PenaltyAreaRadius = 1000;//also knwon as mohavate jarime
	//const float CenterCircleRadius = 500;
	//const float FieldMargin = 700;
	//const float SurroundFieldMargin = 250;
	//const 
	//const float WholeFieldLengthG = 10.400;
	//const float WholeFieldWidthG = 7.400;
	//const float FieldLengthG = 8.000;
	//const float FieldWidthG = 6.000;
	//const float FreeBoundG = 0.500;
	//const float GoalLengthG = 1.000;
	//const float CornelPointG = 4.000;
	//const float PenaltyPointG = 1.000;
	//const float PenaltyAreaRadiusG = 1.000; //also knwon as mohavate jarime
	//const float CenterCircleRadiusG = 0.500;
	//const float WholeFieldMarginG = 0.700;
	//const float SurroundFieldMarginG = 0.250;
	//const float GoalDeep = 0.180;
*/
	


#if DIVISION == 1
const double GlframeScale = 67.0; //if double size=67 and single size=50	//67.0	
const int WholeFieldLength = 13400;	
const int WholeFieldWidth = 10400;	//7400
const int SurroundFieldLength = 12600;
const int SurroundFieldWidth = 9600;
const int FieldLength = 12000;
const int FieldWidth = 9000;
const int FreeBound = 500;	//?
const int GoalLength = 1200;
const int GoalPostLength = 180;
const int CornelPoint = 6000;
const int PenaltyPoint = 1200;
const int PenaltyAreaRadius = 1200;//also knwon as mohavate jarime
const int PenaltyAreaWidth = 1800;
const int PenaltyAreaLength = 3800;
const int CenterCircleRadius = 500;
const int WholeFieldMargin = 700;
const int SurroundFieldMargin = 300;	//250
//const int PenaltyAreaCenterLineLength = 500;

const double WholeFieldLengthG = 13.400;
const double WholeFieldWidthG = 10.400;
const double FieldLengthG = 12.000;
const double FieldWidthG = 9.000;
const double FreeBoundG = 0.500;	//?
const double GoalLengthG = 1.200;
const double CornelPointG = 6.000;
const double PenaltyPointG = 1.200;
//const double PenaltyAreaRadiusG = 1.000; //also knwon as mohavate jarime
const int PenaltyAreaWidthG = 1.200;
const int PenaltyAreaLengthG = 2.400;
const double CenterCircleRadiusG = 0.500;
const double WholeFieldMarginG = 0.700;
const double SurroundFieldMarginG = 0.300;	//250
const float GoalDeep = 0.180;


#elif DIVISION == 2
const double GlframeScale = 67.0; //if double size=67 and single size=50
const int WholeFieldLength = 10400;	//10400
const int WholeFieldWidth = 7400;	//7400
const int SurroundFieldLength = 9000;
const int SurroundFieldWidth = 6000;
const int FieldLength = 9000;
const int FieldWidth = 6000;
const int FreeBound = 300;
const int GoalLength = 1000;
const int GoalPostLength = 180;
const int CornelPoint = 4500;	//4490
const int PenaltyPoint = 1000;
const int PenaltyAreaRadius = 1000;//also knwon as mohavate jarime
const int PenaltyAreaWidth = 1300;
const int PenaltyAreaLength = 2300;
const int CenterCircleRadius = 500;
const int WholeFieldMargin = 700;
const int SurroundFieldMargin = 300;	//250
//const int PenaltyAreaCenterLineLength = 500;

const double WholeFieldLengthG = 10.400;
const double WholeFieldWidthG = 7.400;
const double FieldLengthG = 9.000;
const double FieldWidthG = 6.000;
const double FreeBoundG = 0.300;
const double GoalLengthG = 1.000;
const double CornelPointG = 4.500;	//4.490
const double PenaltyPointG = 1.000;
//const double PenaltyAreaRadiusG = 1.000; //also knwon as mohavate jarime
const int PenaltyAreaWidthG = 1.000;
const int PenaltyAreaLengthG = 2.000;
const double CenterCircleRadiusG = 0.500;
const double WholeFieldMarginG = 0.700;
const double SurroundFieldMarginG = 0.300;	//250
const float GoalDeep = 0.180;
#endif



#if RECTANGULAR_PENALTY_AREA == 0
//**RRT
const VecPosition PALU(-3500, 250);		//penalty area line left up
const VecPosition PALD(-3500, -250);	//penalty area line left down
const VecPosition PARU(3500, 250);		//penalty area line right up
const VecPosition PARD(3500, -250);		//penalty area line right down
//**

#elif DIVISION==3
const double GlframeScale = 67.0; //if double size=67 and single size=50
const int WholeFieldLength = 6000;
const int WholeFieldWidth = 4000;
const int SurroundFieldLength = 6000;
const int SurroundFieldWidth = 4000;
const int FieldLength = 6000;
const int FieldWidth = 4000;
const int FreeBound = 500;
const int GoalLength = 700;
const int GoalPostLength = 200;
const int CornelPoint = 4500;	//4490
const int PenaltyPoint = 1000;
const int PenaltyAreaRadius = 800;//also knwon as mohavate jarime
const int PenaltyAreaWidth = 800;
const int PenaltyAreaLength = 2000;
const int CenterCircleRadius = 500;
const int WholeFieldMargin = 200;
const int SurroundFieldMargin = 200;	//250
//const int PenaltyAreaCenterLineLength = 500;

const double WholeFieldLengthG = 6.000;
const double WholeFieldWidthG = 4.000;
const double FieldLengthG = 6.000;
const double FieldWidthG = 4.000;
const double FreeBoundG = 0.500;
const double GoalLengthG = 0.700;
const double CornelPointG = 4.500;	//4.490
const double PenaltyPointG = 1.000;
//const double PenaltyAreaRadiusG = 1.000; //also knwon as mohavate jarime
const int PenaltyAreaWidthG = 0.800;
const int PenaltyAreaLengthG = 2.000;
const double CenterCircleRadiusG = 0.500;
const double WholeFieldMarginG = 0.700;
const double SurroundFieldMarginG = 0.300;	//250
const float GoalDeep = 0.180;


#endif





////**Strategy
//
//	//**corners
//const VecPosition LeftSideRightCorner(-(FieldLength/2),(FieldWidth/2));
//const VecPosition LeftSideLeftCorner((FieldLength / 2),(FieldWidth/2));
//const VecPosition RightSideRightCorner((FieldLength / 2),-(FieldWidth / 2));
//const VecPosition RightSideLeftCorner((FieldLength / 2), (FieldWidth / 2));
//	//**
//
//	//**center
//const VecPosition CenterCircleCenterPoint(0, 0);
//const VecPosition CenterCircleRightPoint(PenaltyAreaRadius, 0);
//const VecPosition CenterCircleLeftPoint(-PenaltyAreaRadius, 0);
//const VecPosition CenterCircleUpPoint(0, PenaltyAreaRadius);
//const VecPosition CenterCircleDownPoint(0, -PenaltyAreaRadius);
//	//**
//	
//	//**Goal
//const VecPosition RightPenaltyAreaLineRightPoint(FieldLength/2,-(PenaltyAreaCenterLineLength/2 + PenaltyAreaRadius));
//const VecPosition RightPenaltyAreaLineCentePoint(FieldLength / 2,0);
//const VecPosition RightPenaltyAreaLineLeftPoint(FieldLength / 2, PenaltyAreaCenterLineLength / 2 + PenaltyAreaRadius);
//
//const VecPosition LeftPenaltyAreaLineRightPoint(-(FieldLength / 2), PenaltyAreaCenterLineLength / 2 + PenaltyAreaRadius);
//const VecPosition LeftPenaltyAreaLineCenterPoint(-(FieldLength / 2), 0);
//const VecPosition LeftPenaltyAreaLineLeftPoint(-(FieldLength / 2), -(PenaltyAreaCenterLineLength / 2 + PenaltyAreaRadius));
//
//const VecPosition RightPenaltyPoint((FieldLength / 2) - PenaltyAreaRadius, 0);
//const VecPosition LeftPenaltyPoint(-((FieldLength / 2) - PenaltyAreaRadius), 0);
//	//**
//
////**



#endif // FIELD_H