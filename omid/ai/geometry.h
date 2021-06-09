/*
Copyright (c) 2000-2003, Jelle Kok, University of Amsterdam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Amsterdam nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*! \file Geometry.h
<pre>
<b>File:</b>          Geometry.h
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       12/02/2001
<b>Last Revision:</b> $ID$
<b>Contents:</b>      Header file for the classes VecPosition, Geometry, Line,
               Circle and Rectangle. All the member
               data and member method declarations for all these classes can be
               found in this file together with some auxiliary functions for
               numeric and goniometric purposes.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
12/02/2001       Jelle Kok       Initial version created
09/06/2001       Remco de Boer   Version including full documentation completed
</pre>
*/

#ifndef _GEOMETRY_
#define _GEOMETRY_

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>       // needed for string
#include <iostream>
class World;
class Circle;
using namespace std;

typedef double AngRad;  /*!< Type definition for angles in degrees. */
typedef double AngDeg;  /*!< Type definition for angles in radians. */

#define EPSILON 0.0001  /*!< Value used for floating point equality tests. */

// auxiliary numeric functions for determining the
// maximum and minimum of two given double values and the sign of a value
//double max     ( double d1, double d2 );
//double min     ( double d1, double d2 );
int    sign    ( double d1            );

// auxiliary goniometric functions which enable you to
// specify angles in degrees rather than in radians
AngDeg Rad2Deg ( AngRad x             );
AngRad Deg2Rad ( AngDeg x             );
double cosDeg  ( AngDeg x             );
double sinDeg  ( AngDeg x             );
double tanDeg  ( AngDeg x             );
AngDeg atanDeg ( double x             );
double atan2Deg( double x,  double y  );
AngDeg acosDeg ( double x             );
AngDeg asinDeg ( double x             );

// various goniometric functions
bool   isAngInInterval     ( AngDeg ang,    AngDeg angMin,    AngDeg angMax );
AngDeg getBisectorTwoAngles( AngDeg angMin, AngDeg angMax );

/*! CoordSystem is an enumeration of the different specified
    coordinate systems.  The two possibilities are CARTESIAN or
    POLAR. These values are for instance used in the initializing a
    VecPosition. The CoordSystem indicates whether the supplied
    arguments represent the position in cartesian or in polar
    coordinates. */
enum CoordSystemT {
    CARTESIAN,
    POLAR
};

/*****************************************************************************/
/********************   CLASS VECPOSITION   **********************************/
/*****************************************************************************/

/*! This class contains an x- and y-coordinate of a position (x,y) as
    member data and methods which operate on this position. The
    standard arithmetic operators are overloaded and can thus be
    applied to positions (x,y). It is also possible to represent a
    position in polar coordinates (r,phi), since the class contains a
    method to convert these into cartesian coordinates (x,y). */
class VecPosition
{
    // private member data
//me
//private:
public:
//me
    double m_x;   /*!< x-coordinate of this position */
    double m_y;   /*!< y-coordinate of this position */
    // public methods
public:

	///**RRT variables
	int parent;
	int number;
	///**

    // constructor for VecPosition class
    VecPosition (double vx = 0 , double vy = 0 , CoordSystemT cs = CARTESIAN);
    // overloaded arithmetic operators
    VecPosition operator - ();
    VecPosition operator + (const double &d);
    VecPosition operator + (const VecPosition &p);
    VecPosition operator - (const double &d);
    VecPosition operator - (const VecPosition &p) const;
    VecPosition operator * (const double &d);
    VecPosition operator * (const VecPosition &p);
    double InerMultiply    (const VecPosition &p)
    {
        return this->m_x * p.m_x + this->m_y * p.m_y;
    }
    VecPosition operator / (const double &d);
    VecPosition operator / (const VecPosition &p);
    void operator = (const double &d);
    void operator += (const VecPosition &p);
    void operator += (const double &d);
    void operator -= (const VecPosition &p);
    void operator -= (const double &d);
    void operator *= (const VecPosition &p);
    void operator *= (const double &d);
    void operator /= (const VecPosition &p);
    void operator /= (const double &d);
    bool operator != (const VecPosition &p);
    bool operator != (const double &d);
    bool operator == (const VecPosition &p);
    bool operator == (const double &d);

    // methods for producing output
    friend ostream&    operator <<            ( ostream           &os,
                                                VecPosition       p            );
    void               show                   ( CoordSystemT      cs =CARTESIAN);
    string             str                    ( CoordSystemT      cs =CARTESIAN);

    // set- and get methods for private member variables
    bool               setX                   ( double            dX           );
	double             getX					  (                          ) const;
    bool               setY                   ( double            dY           );
    double             getY                   (                          ) const;

    // set- and get methods for derived position information
    void               setVecPosition         ( double            dX = 0,
                                                double            dY = 0,
                                                CoordSystemT      cs =CARTESIAN);
    double             getDistanceTo          ( const VecPosition &p     ) const;
    VecPosition        setMagnitude           ( double            d            );
    double             getMagnitude           (                          ) const;
    AngDeg             getDirection           (                          ) const;
	
    // comparison methods for positions
    bool               isInFrontOf            ( const VecPosition &p           );
    bool               isInFrontOf            ( const double      &d           );
    bool               isBehindOf             ( const VecPosition &p           );
    bool               isBehindOf             ( const double      &d           );
    bool               isLeftOf               ( const VecPosition &p           );
    bool               isLeftOf               ( const double      &d           );
    bool               isRightOf              ( const VecPosition &p           );
    bool               isRightOf              ( const double      &d           );
    bool               isBetween              ( const VecPosition &p1,
                                                const VecPosition &p2          )
    {
        double tt = sqrt(pow(p2.getX()-p1.getX(),2)+pow(p2.getY()-p1.getY(),2));

        if(sqrt(pow(getX()-p1.getX(),2)+pow(getY()-p1.getY(),2)) < tt && sqrt(pow(p2.getX()-getX(),2)+pow(p2.getY()-getY(),2)) < tt)
            return true;
        return false;
        //      double ph1 = ((getX() - p2.getX()) / (p1.getX() - p2.getX()));
        //      double ph2 = ((getY() - p2.getY()) / (p1.getY() - p2.getY()));

        //       return ((ph1 + ph2) / 2.0 > 0 && (ph1 + ph2) / 2.0 < 1);
        //      return ph1 == ph2;
        //       return ( ( getX() <= max(p1.getX(), p2.getX()) ) && ( getX() >= min(p1.getX(), p2.getX())  ) ) && ( ( getY() >= min(p1.getY(), p2.getY()) ) && ( getY() <= max(p2.getY(), p1.getY()) ) );
    }

    bool               isBetweenX             ( const VecPosition &p1,
                                                const VecPosition &p2          );
    bool               isBetweenX             ( const double      &d1,
                                                const double      &d2          );
    bool               isBetweenY             ( const VecPosition &p1,
                                                const VecPosition &p2          );
    bool               isBetweenY             ( const double      &d1,
                                                const double      &d2          );
	bool			   isInCircle             ( const Circle      &c		   ) const;
	

    // conversion methods for positions
    VecPosition        normalize              (                                );
    VecPosition        rotate                 ( AngRad            angle        );
    VecPosition        globalToRelative       ( VecPosition       orig,
                                                AngDeg            ang          );
    VecPosition        relativeToGlobal       ( VecPosition       orig,
                                                AngDeg            ang          );
    VecPosition        getVecPositionOnLineFraction( VecPosition  &p,
                                                     double            dFrac        );
    AngRad             Angle                  ( bool   _positive_angle = false )
    {


        double val;
        if (this->getMagnitude() == 0) return 0;
        if (this->getY() >= 0) val =  acos(this->getX() / this->getMagnitude());
        else val = -acos(this->getX() / this->getMagnitude());
        if(_positive_angle)
        {
            if( val < 0 )
                val += 2 * M_PI;

            //          if(val < 0)
            //              logStatus("This angle must not be less than zero!!! --> " + string::number(val) , QColor("red"));
        }
        return val;
    }
    static VecPosition directVector           ( AngRad            angle        )
    {
        VecPosition v;
        v.setX(cos(angle));
        v.setY(sin(angle));
        return v ;
    }
    static AngRad      AngleBetweenWithSgn    ( VecPosition       v1,
                                                VecPosition       v2           )
    {
        double ccw;
        double angle1, angle2;
        angle1 = v1.Angle();
        angle2 = v2.Angle();
        if (angle1 < 0 && angle2 > 0)
        {
            angle1 += M_PI;
            angle2 -= M_PI;
        }
        if ((angle1 * angle2 >= 0 && angle1 < angle2) || (angle1 * angle2 < 0 && angle1 - M_PI > angle2)) ccw = 1;
        else ccw = -1;
        double abs1 = v1.getMagnitude();
        double abs2 = v2.getMagnitude();
        if (abs1 == 0 || abs2 == 0) return 0;
        double m = v1.InerMultiply(v2);
        if ( (m) / (abs1 * abs2 + 0.0001) > 1 || (m) / (abs1 * abs2 + 0.0001) < -1 )
        {
            cout<<"acos in angle between sgn is going to crash";
            return 0;
        }
        return acos((m) / (abs1 * abs2 + 0.0001)) * ccw;


    }
    AngRad             AngleBetween           ( VecPosition v                  )
    {
        VecPosition thiss;
        thiss.setX(m_x);
        thiss.setY(m_y);
        double b1 = v.getMagnitude();
        double b2 = thiss.getMagnitude();
        if (b1==0 || b2== 0) return 0;
        double m =v .InerMultiply(thiss);

        if ( (m) / (b1 * b2 + 0.0001) > 1 || (m) / (b1 * b2 + 0.0001) < -1 )
        {
           cout<< "acos in angle between is going to crash";
            return 0;
        }

        return acos((m) / (b1 * b2 + 0.0001));//(Why?)
    }

    VecPosition getNearer( VecPosition point1, VecPosition point2 ) const
    {
        return ( getDistanceTo( point1 ) < getDistanceTo( point2 ) ? point1 : point2);
    }
    VecPosition getFarest( VecPosition point1, VecPosition point2 )
    {
        return ( getDistanceTo( point1 ) > getDistanceTo( point2 ) ? point1 : point2);
    }

	///**RRT Methods
	
	

	VecPosition cartesian_to_polar();
	VecPosition polar_to_cartesian();
	VecPosition cartesian_to_polar_from_CP(const VecPosition &cp) const;
	VecPosition getNearestPositionOutsideOfCircle(const Circle &balk);
	VecPosition getPositionToward(const double &angle, const double &distance) const;
	VecPosition getPositionToward(const VecPosition &destination, const double &distance) const;
	double getSlopeToward(const VecPosition &end) const;
	double getAngleToward(const VecPosition &end) const;
	double getAngle();
	///**

    // static class methods
    static VecPosition getVecPositionFromPolar( double            dMag,
                                                AngDeg            ang          );
    static AngDeg      normalizeAngle         ( AngDeg            angle        );

    
    bool isNan( double _w = 0 );
    static bool isInvalidNumber               ( double n1, double n2 = 0, double n3 = 0, double n4 = 0, double n5 = 0 );

};

/*****************************************************************************/
/*********************   CLASS GEOMETRY   ************************************/
/*****************************************************************************/

/*! This class contains several static methods dealing with geometry.*/
class Geometry
{

public:

    // geometric series
    static double getLengthGeomSeries(double dFirst,double dRatio,double dSum  );
    static double getSumGeomSeries   (double dFirst,double dRatio,double dLen  );
    static double getSumInfGeomSeries(double dFirst,double dRatio              );
    static double getFirstGeomSeries (double dSum,  double dRatio,double dLen  );
    static double getFirstInfGeomSeries(double dSum,double dRatio              );

    // abc formula
    static int    abcFormula(double a,double b, double c, double *s1,double *s2);
};

/*****************************************************************************/
/********************** CLASS CIRCLE *****************************************/
/*****************************************************************************/

/*!This class represents a circle. A circle is defined by one VecPosition
   (which denotes the center) and its radius. */

class Circle
{
    VecPosition m_posCenter;            /*!< Center of the circle  */
    double      m_dRadius;              /*!< Radius of the circle  */

public:
    Circle( );
    Circle( VecPosition pos, double dR );

    void        show                  ( ostream& os = cout );

    // get and set methods
    bool        setCircle             ( VecPosition pos,
                                        double      dR  );
    bool        setRadius             ( double dR       );
    double      getRadius             (                 ) const;
    bool        setCenter             ( VecPosition pos );
    VecPosition getCenter             (                 ) const;
    double      getCircumference      (                 );
    double      getArea               (                 );

    // calculate intersection points and area with other circle
    bool        isInside              ( VecPosition pos );
    int         getIntersectionPoints ( Circle      c,
                                        VecPosition *p1,
                                        VecPosition *p2 );
    double      getIntersectionArea   ( Circle c        );
	
	VecPosition getNearestPointOnCircleTo(const VecPosition &point) const;
	VecPosition		   getPositionOnCircleWithAngle(double rad_ang);

}  ;


/*****************************************************************************/
/*********************** CLASS LINE ******************************************/
/*****************************************************************************/

/*!This class contains the representation of a line. A line is defined
   by the formula ay + bx + c = 0. The coefficients a, b and c are stored
   and used in the calculations. */
class Line
{
private:
    // a line is defined by the formula: ay + bx + c = 0
    double m_a; /*!< This is the a coefficient in the line ay + bx + c = 0 */
    double m_b; /*!< This is the b coefficient in the line ay + bx + c = 0 */
    double m_c; /*!< This is the c coefficient in the line ay + bx + c = 0 */

public:
    Line();
    Line( double a, double b, double c );

    // print methods
    void show(ostream& os = cout );
    friend ostream& operator << (ostream & os, Line l);

    bool operator != (const Line &l);
    bool operator == (const Line &l);

    // get intersection points with this line
    VecPosition getIntersection            ( Line        line                  );
    int         getCircleIntersectionPoints( Circle      circle,
                                             VecPosition *posSolution1,
                                             VecPosition *posSolution2         );
    Line        getTangentLine             ( VecPosition pos                   );
    VecPosition getPointOnLineClosestTo    ( VecPosition pos                   );
    double      getDistanceWithPoint       ( VecPosition pos                   );
    bool        isInBetween                ( VecPosition pos,
                                             VecPosition point1,
                                             VecPosition point2                );

    // calculate associated variables in the line
    double      getYGivenX                 ( double      x );
    double      getXGivenY                 ( double      y );
    double      getACoefficient            (               ) const;
    double      getBCoefficient            (               ) const;
    double      getCCoefficient            (               ) const;

    // static methods to make a line using an easier representation.
    static Line makeLineFromTwoPoints      ( VecPosition pos1,
                                             VecPosition pos2                  );
    static Line makeLineFromPositionAndAngle( VecPosition vec,
                                              AngDeg angle                      );
};

/*****************************************************************************/
/********************** CLASS GOAL_SHAPE *****************************************/
/*****************************************************************************/

/*!This class represents a Goal_Shape. A Goal_Shape is defined by one size
   (which denotes the scale) */

//const VecPosition VecPosition((FieldLength/2),  (FreeBound/2)) = VecPosition((FieldLength/2),  (FreeBound/2)) ;
//const VecPosition VecPosition((FieldLength/2),  -(FreeBound/2)) = VecPosition((FieldLength/2), -(FreeBound/2)) ;
//const double front_line_length = 350 ;

class Goal_Shape
{
    double      m_dScale;              /*!< size of the Goal_Shape           */
    Circle      up_circle ;            /*!< Uper Circle of the Goal_Shape    */
    Circle      down_circle ;          /*!< Downer Circle of the Goal_Shape  */
    Line        front_line ;           /*!< Front Line of the Goal_Shape     */

public:
    Goal_Shape( );
    Goal_Shape( double scale );

    void        show                  ( ostream& os = cout );

    // get and set methods
    bool        setGoal_Shape         ( double scale    );
    double      getScale              (                 );
    Circle      get_Up_Circle         (                 );
    Circle      get_Down_Circle       (                 );
    Line        get_Front_Line        (                 );
    double      getCircumference      (                 );
    double      getArea               (                 );

    // calculate intersection points and area with other Goal_Shape
    bool        isInside              ( VecPosition pos );

    // get intersection points with this line
    VecPosition getIntersection       ( Line       line);


}  ;
/***************************************************************************/
/********************** CLASS PARALINE *************************************/
/***************************************************************************/

/*!This class represents a Paraline. A Paraline is defined by two VecPosition
     _first denotes start point & _second denotes next point */

class Paraline
{
    VecPosition      m_dfirst  ;            /*!< Position Of first point        */
    VecPosition      m_dsecond ;            /*!< Position Of second point       */
    double           m_dlength ;            /*!< length Of first the paraline   */

	double m_a; /*!< This is the a coefficient in the Parline ay + bx + c = 0 */
	double m_b; /*!< This is the b coefficient in the Parline ay + bx + c = 0 */
	double m_c; /*!< This is the c coefficient in the Parline ay + bx + c = 0 */

public:
    Paraline() ;
    Paraline(VecPosition _first, VecPosition _second) ;

    void        show                  ( ostream &os = cout );

    // get and set methods
    void          setFirstPoint            ( VecPosition _first                 );
    void          setSecondPoint           ( VecPosition _second                );
    VecPosition   getFirstPoint            (                                    );
    VecPosition   getSecondPoint           (                                    );
    VecPosition   getMidPoint              (                                    );
    double        getLength                (                                    );
    double        getDistanceTo            ( VecPosition _point                 );
	double		  getACoefficient		   (									) const;
	double		  getBCoefficient		   (									) const;
	double		  getCCoefficient		   (									) const;
	double		  getYGivenX			   (double      x						);
	double        getXGivenY			   (double      y						);
    VecPosition   getNearerPoint           ( VecPosition _point                 );
    Paraline      getMaxParaline              ( Paraline* _para_array, int _num );
    Paraline      getMinParaline              ( Paraline* _para_array, int _num );
    VecPosition   getPointOnParalineClosestTo  ( VecPosition _point             );

    // comparison methods for positions
    bool          hasIntersection          ( Circle _object ,double _limit = 22.5);
	bool          hasIntersection	   ( Paraline paraline					 );
	bool		  isPositionInRight		   ( VecPosition point                   );
	bool		  isPositionInLeft         ( VecPosition point                   );
} ;


/***************************************************************************/
/********************** CLASS CONE *****************************************/
/***************************************************************************/

/*!This class represents a Cone. A Cone is defined by one VecPosition
     (which denotes the ball) and two VecPosition (which denotes the bar's) */

class Cone
{
    VecPosition      m_dBall;        	    /*!< Position Of Ball */
    VecPosition      m_dup_bar ;            /*!< Uper Circle of the Cone    */
    VecPosition      m_ddown_bar ;          /*!< Downer Circle of the Cone  */

    AngDeg Max_Bar ;
    AngDeg Min_Bar ;
    AngDeg Alfa ;

    //    struct hole_type{
    //        double first_y ;
    //        double second_y ;
    //    } ;

public:
    Cone()
    {
    }
    Cone( VecPosition Ball, VecPosition up_bar, VecPosition down_bar );

    void        show                  ( ostream& os = cout );

    // get and set methods
    void          set_ball              ( VecPosition Ball     );
    void          set_up_bar            ( VecPosition up_bar   );
    void          set_down_bar          ( VecPosition down_bar );
    VecPosition   get_Ball              (                      );
    VecPosition   get_up_bar            (                      );
    VecPosition   get_down_bar          (                      );

    // calculate intersection points and area with other Cone
    bool        isInside              ( VecPosition pos      );

    // calculate percent of open area in goal
    //    double      get_percent           ( VecPosition* vec_array, int num ) ;
    //    int         get_paraline          ( Circle* vec_array, int num ,Paraline* para_array) ;

    // New Cone -- >
    struct Hole_Type{
        VecPosition Point_1 ;
        VecPosition Point_2 ;
    } ;

    bool        Is_In_Cone            ( Circle Obstacle                       ) ;
    bool        Get_Cover_Area        ( Circle Obstacle , Hole_Type &Output   ) ;

    double      Get_Free_Space_In_Cone(Circle *Obstacle, int Num_of_Obstacle, Hole_Type &Longest_Hole) ;
    double      Get_Free_Space_In_Cone(Circle *Obstacle, int Num_of_Obstacle) ;
    double      Get_Free_Space_In_Cone(Circle *Obstacle, int Num_of_Obstacle, Hole_Type *Holes, int &Num_of_Hole) ;
    double      Get_Free_Space_In_Cone(Circle *Obstacle, int Num_of_Obstacle, Hole_Type *Holes, int &Num_of_Hole, int &Longest_Hole_Index) ;
    double      Get_Free_Space_In_Cone(Hole_Type &longest_hole, int _team_number = 0, int _opp_number = 0, int _non_obs1 = -1, int _non_obs2 = -1, int _non_obs3 = -1, int _non_obs4= -1);
    double      Get_Free_Space_In_Cone(int _team_number = 0, int _opp_number = 0, int _non_obs1 = -1, int _non_obs2 = -1, int _non_obs3 = -1, int _non_obs4= -1);

    double      Get_Free_Surface_In_Cone(Circle *Obstacle, int Num_of_Obstacle) ;
    double      Get_Free_Surface_In_Cone(int _team_number = 0, int _opp_number = 0, int _non_obs1 = -1, int _non_obs2 = -1, int _non_obs3 = -1, int _non_obs4= -1);
}  ;

/*****************************************************************************/
/********************* CLASS Regression **************************************/
/*****************************************************************************/

class Regression
{
    //      VecPosition *input_positions ;
    //      int num_of_input;

public:
    //  Regression                    ( VecPosition* _pos, int _num      );

    static Line getLinearRegression      (  VecPosition* _pos, int _num     );

    Line getNonlinearRegression   (                                   );

    void        show              ( ostream& os = cout                );

    // checks whether point lies inside the rectangle
    bool        isInside          ( VecPosition pos                   );

    // standard get and set methosd
    void        setRectanglePoints( VecPosition pos1,
                                    VecPosition pos2                  );
    bool        setPosLeftTop     ( VecPosition pos                   );
    VecPosition getPosLeftTop     (                                   );
    bool        setPosRightBottom ( VecPosition pos                   );
    VecPosition getPosRightBottom (                                   );

    //double getDistanceTo     ( VecPosition p                     );
};

/*****************************************************************************/
/********************** CLASS RECTANGLE **************************************/
/******************************************************************************/
/*! This class represents a rectangle. A rectangle is defined by two
   VecPositions the one at the upper left corner and the one at the
   right bottom. */
class Rect
{
    VecPosition m_posLeftTop;     /*!< top left position of the rectangle      */
    VecPosition m_posRightBottom; /*!< bottom right position of the rectangle  */

public:
    Rect                          (                                   )
    {
        setPosLeftTop(VecPosition(0,0));
        setPosRightBottom(VecPosition(0,0));
    }

    Rect                          ( VecPosition pos, VecPosition pos2 );

    void        show              ( ostream& os = cout                );

    // checks whether point lies inside the rectangle
    bool        isInside          ( const VecPosition & pos                   ) const;

    // standard get and set methosd
    void        setRectanglePoints( VecPosition pos1,
                                    VecPosition pos2                  );
    bool        setPosLeftTop     ( VecPosition pos                   );
    VecPosition getPosLeftTop     (                                   );
    bool        setPosRightBottom ( VecPosition pos                   );
    VecPosition getPosRightBottom (                                   );

	bool		hasIntersectionWith( Line line                        );
	bool		hasIntersectionWith(Paraline paraline);
	bool		hasIntersectionWith(VecPosition start, VecPosition end);
    //double getDistanceTo     ( VecPosition p                     );
};

/*****************************************************************************/
/********************** CLASS HISTOGRAM **************************************/
/*****************************************************************************/
class Histogram
{
public:
    double minimumlimit;
    double maximumlimit;

    int numofsteps;

    int numofeach[100];
    int numofrecivedata;

    Histogram()
    {
        this->setOption( 0, 1, 10 );
    }

    Histogram( double _minimumlimit, double _maximumlimit, int _numofsteps )
    {
        this->setOption( _minimumlimit, _maximumlimit, _numofsteps );
    }

    void setOption ( double _minimumlimit, double _maximumlimit, int _numofsteps )
    {
        minimumlimit = fmin( _minimumlimit, _maximumlimit ) ;
        maximumlimit = fmax( _minimumlimit, _maximumlimit );

        numofsteps = _numofsteps;

        this->clearData();
    }

    void clearData()
    {
        for ( int i = 0; i < numofsteps; i++ ) numofeach[i] = 0;
        numofrecivedata = 0;
    }

    void setData( double _data )
    {
        double stepmagnetiut = ( maximumlimit - minimumlimit ) / numofsteps;
        for ( int i = 0; i < numofsteps; i++ )
            if ( _data > minimumlimit + i * stepmagnetiut && _data < minimumlimit + ( i + 1 ) * stepmagnetiut )
                numofeach[i] ++;

        numofrecivedata ++;
    }


};

/******************************************************************************/
/************************** CLASS DIAGRAM *************************************/
/******************************************************************************/

const int maxNumOfStack = 2000;

/*! This class represents a diagram. */
class Diagram
{

public:
  Diagram();

  void resetDiagram();
  void setNextItem ( double nextStack1, double nextStack2 = 0, double nextStack3 = 0 );

  int numOfSavedStack;

  double stack1[ maxNumOfStack ];
  double stack2[ maxNumOfStack ];
  double stack3[ maxNumOfStack ];

};
#endif
