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

/*! \file Geometry.cpp
<pre>
<b>File:</b>          Geometry.cpp
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       13/02/2001
<b>Last Revision:</b> $ID$
<b>Contents:</b>      class declarations of different geometry classes:<BR>
                       - VecPosition: representation of a point
                       - Line:        representation of a line
                       - Rectangle:   representation of a rectangle
                       - Circle:      representation of a circle
                       - Geometry:    different geometry methods

Furthermore it contains some goniometric functions to work with sine, cosine
and tangent functions using degrees and some utility functions to return
the maximum and the minimum of two values.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
12/02/2001       Jelle Kok       Initial version created
</pre>
*/

#include "geometry.h"
#include <stdio.h>    // needed for sprintf
#include "world.h"
#include "Switches.h"
/*! This function returns the sign of a give double.
    1 is positive, -1 is negative
    \param d1 first parameter
    \return the sign of this double */
int sign( double d1 )
{
    return (d1>0)?1:-1;
}

/*! This function returns the maximum of two given doubles.
    \param d1 first parameter
    \param d2 second parameter
    \return the maximum of these two parameters */
double max( double d1, double d2 )
{
    return (d1>d2)?d1:d2;
}

///*! This function returns the minimum of two given doubles.
//    \param d1 first parameter
//    \param d2 second parameter
//    \return the minimum of these two parameters */
double min( double d1, double d2 )
{
    return (d1<d2)?d1:d2;
}


/*! This function converts an angle in radians to the corresponding angle in
    degrees.
    \param x an angle in radians
    \return the corresponding angle in degrees */
AngDeg Rad2Deg( AngRad x )
{
    return ( x * 180 / M_PI );
}

/*! This function converts an angle in degrees to the corresponding angle in
    radians.
    \param x an angle in degrees
    \return the corresponding angle in radians */
AngRad Deg2Rad( AngDeg x )
{
    return ( x * M_PI / 180 );
}

/*! This function returns the cosine of a given angle in degrees using the
    built-in cosine function that works with angles in radians.
    \param x an angle in degrees
    \return the cosine of the given angle */
double cosDeg( AngDeg x )
{
    return ( cos( Deg2Rad( x ) ) );
}

/*! This function returns the sine of a given angle in degrees using the
    built-in sine function that works with angles in radians.
    \param x an angle in degrees
    \return the sine of the given angle */
double sinDeg( AngDeg x )
{
    return ( sin( Deg2Rad( x ) ) );
}

/*! This function returns the tangent of a given angle in degrees using the
    built-in tangent function that works with angles in radians.
    \param x an angle in degrees
    \return the tangent of the given angle */
double tanDeg( AngDeg x )
{
    return ( tan( Deg2Rad( x ) ) );
}

/*! This function returns the principal value of the arc tangent of x
    in degrees using the built-in arc tangent function which returns
    this value in radians.
    \param x a double value
    \return the arc tangent of the given value in degrees */
AngDeg atanDeg( double x )
{
    return ( Rad2Deg( atan( x ) ) );
}

/*! This function returns the principal value of the arc tangent of y/x in
    degrees using the signs of both arguments to determine the quadrant of the
    return value. For this the built-in 'atan2' function is used which returns
    this value in radians.
    \param x a double value
    \param y a double value
    \return the arc tangent of y/x in degrees taking the signs of x and y into
    account */
double atan2Deg( double x, double y )
{
    if( fabs( x ) < EPSILON && fabs( y ) < EPSILON )
        return ( 0.0 );

    return ( Rad2Deg( atan2( x, y ) ) );
}

/*! This function returns the principal value of the arc cosine of x in degrees
    using the built-in arc cosine function which returns this value in radians.
    \param x a double value
    \return the arc cosine of the given value in degrees */
AngDeg acosDeg( double x )
{
    if( x >= 1 )
        return ( 0.0 );
    else if( x <= -1 )
        return ( 180.0 );

    return ( Rad2Deg( acos( x ) ) );
}

/*! This function returns the principal value of the arc sine of x in degrees
    using the built-in arc sine function which returns this value in radians.
    \param x a double value
    \return the arc sine of the given value in degrees */
AngDeg asinDeg( double x )
{
    if( x >= 1 )
        return ( 90.0 );
    else if ( x <= -1 )
        return ( -90.0 );

    return ( Rad2Deg( asin( x ) ) );
}

/*! This function returns a boolean value which indicates whether the value
   'ang' (from interval [-180..180] lies in the interval [angMin..angMax].
    Examples: isAngInInterval( -100, 4, -150) returns false
             isAngInInterval(   45, 4, -150) returns true
    \param ang angle that should be checked
    \param angMin minimum angle in interval
    \param angMax maximum angle in interval
    \return boolean indicating whether ang lies in [angMin..angMax] */
bool isAngInInterval( AngDeg ang, AngDeg angMin, AngDeg angMax )
{
    // convert all angles to interval 0..360
    if( ( ang    + 360 ) < 360 ) ang    += 360;
    if( ( angMin + 360 ) < 360 ) angMin += 360;
    if( ( angMax + 360 ) < 360 ) angMax += 360;

    if( angMin < angMax ) // 0 ---false-- angMin ---true-----angMax---false--360
        return angMin < ang && ang < angMax ;
    else                  // 0 ---true--- angMax ---false----angMin---true---360
        return !( angMax < ang && ang < angMin );
}

/*! This method returns the bisector (average) of two angles. It deals
    with the boundary problem, thus when 'angMin' equals 170 and 'angMax'
    equals -100, -145 is returned.
    \param angMin minimum angle [-180,180]
    \param angMax maximum angle [-180,180]
    \return average of angMin and angMax. */
AngDeg getBisectorTwoAngles( AngDeg angMin, AngDeg angMax )
{
    // separate sine and cosine part to circumvent boundary problem
    return VecPosition::normalizeAngle(
            atan2Deg( (sinDeg( angMin) + sinDeg( angMax ) )/2.0,
                      (cosDeg( angMin) + cosDeg( angMax ) )/2.0 ) );
}

/*****************************************************************************/
/*******************   CLASS VECPOSITION   ***********************************/
/*****************************************************************************/

/*! Constructor for the VecPosition class. When the supplied
    Coordinate System type equals CARTESIAN, the arguments x and y
    denote the x- and y-coordinates of the new position. When it
    equals POLAR however, the arguments x and y denote the polar
    coordinates of the new position; in this case x is thus equal to
    the distance r from the origin and y is equal to the angle phi
    that the polar vector makes with the x-axis.
    \param x the x-coordinate of the new position when cs == CARTESIAN; the
    distance of the new position from the origin when cs = POLAR
    \param y the y-coordinate of the new position when cs = CARTESIAN; the
    angle that the polar vector makes with the x-axis when cs = POLAR
    \param cs a CoordSystemT indicating whether x and y denote cartesian
    coordinates or polar coordinates
    \return the VecPosition corresponding to the given arguments */
VecPosition::VecPosition( double x, double y, CoordSystemT cs )
{
    setVecPosition( x, y, cs );
}

/*! Overloaded version of unary minus operator for VecPositions. It returns the
    negative VecPosition, i.e. both the x- and y-coordinates are multiplied by
    -1. The current VecPosition itself is left unchanged.
    \return a negated version of the current VecPosition */
VecPosition VecPosition::operator - ( )
{
    return ( VecPosition( -m_x, -m_y ) );
}

/*! Overloaded version of the binary plus operator for adding a given double
    value to a VecPosition. The double value is added to both the x- and
    y-coordinates of the current VecPosition. The current VecPosition itself is
    left unchanged.
    \param d a double value which has to be added to both the x- and
    y-coordinates of the current VecPosition
    \return the result of adding the given double value to the current
    VecPosition */
VecPosition VecPosition::operator + ( const double &d )
{
    return ( VecPosition( m_x + d, m_y + d ) );
}

/*! Overloaded version of the binary plus operator for VecPositions. It returns
    the sum of the current VecPosition and the given VecPosition by adding their
    x- and y-coordinates. The VecPositions themselves are left unchanged.
    \param p a VecPosition
    \return the sum of the current VecPosition and the given VecPosition */
VecPosition VecPosition::operator + ( const VecPosition &p )
{
    return ( VecPosition( m_x + p.m_x, m_y + p.m_y ) );
}

/*! Overloaded version of the binary minus operator for subtracting a
    given double value from a VecPosition. The double value is
    subtracted from both the x- and y-coordinates of the current
    VecPosition. The current VecPosition itself is left unchanged.
    \param d a double value which has to be subtracted from both the x- and
    y-coordinates of the current VecPosition
    \return the result of subtracting the given double value from the current
    VecPosition */
VecPosition VecPosition::operator - ( const double &d )
{
    return ( VecPosition( m_x - d, m_y - d ) );
}

/*! Overloaded version of the binary minus operator for
    VecPositions. It returns the difference between the current
    VecPosition and the given VecPosition by subtracting their x- and
    y-coordinates. The VecPositions themselves are left unchanged.

    \param p a VecPosition
    \return the difference between the current VecPosition and the given
    VecPosition */
VecPosition VecPosition::operator - ( const VecPosition &p ) const
{
    return ( VecPosition( m_x - p.m_x, m_y - p.m_y ) );
}

/*! Overloaded version of the multiplication operator for multiplying a
    VecPosition by a given double value. Both the x- and y-coordinates of the
    current VecPosition are multiplied by this value. The current VecPosition
    itself is left unchanged.
    \param d the multiplication factor
    \return the result of multiplying the current VecPosition by the given
    double value */
VecPosition VecPosition::operator * ( const double &d  )
{
    return ( VecPosition( m_x * d, m_y * d  ) );
}

/*! Overloaded version of the multiplication operator for
    VecPositions. It returns the product of the current VecPosition
    and the given VecPosition by multiplying their x- and
    y-coordinates. The VecPositions themselves are left unchanged.

    \param p a VecPosition
    \return the product of the current VecPosition and the given VecPosition */
VecPosition VecPosition::operator * ( const VecPosition &p )
{
    return ( VecPosition( m_x * p.m_x, m_y * p.m_y ) );
}

/*! Overloaded version of the division operator for dividing a
    VecPosition by a given double value. Both the x- and y-coordinates
    of the current VecPosition are divided by this value. The current
    VecPosition itself is left unchanged.

    \param d the division factor
    \return the result of dividing the current VecPosition by the given double
    value */
VecPosition VecPosition::operator / ( const double &d )
{
    return ( VecPosition( m_x / d, m_y / d  ) );
}

/*! Overloaded version of the division operator for VecPositions. It
    returns the quotient of the current VecPosition and the given
    VecPosition by dividing their x- and y-coordinates. The
    VecPositions themselves are left unchanged.

    \param p a VecPosition
    \return the quotient of the current VecPosition and the given one */
VecPosition VecPosition::operator / ( const VecPosition &p )
{
    return ( VecPosition( m_x / p.m_x, m_y / p.m_y ) );
}

/*! Overloaded version of the assignment operator for assigning a given double
    value to both the x- and y-coordinates of the current VecPosition. This
    changes the current VecPosition itself.
    \param d a double value which has to be assigned to both the x- and
    y-coordinates of the current VecPosition */
void VecPosition::operator = ( const double &d )
                             {
    m_x = d;
    m_y = d;
}

/*! Overloaded version of the sum-assignment operator for VecPositions. It
    returns the sum of the current VecPosition and the given VecPosition by
    adding their x- and y-coordinates. This changes the current VecPosition
    itself.
    \param p a VecPosition which has to be added to the current VecPosition */
void VecPosition::operator +=( const VecPosition &p )
                             {
    m_x += p.m_x;
    m_y += p.m_y;
}

/*! Overloaded version of the sum-assignment operator for adding a given double
    value to a VecPosition. The double value is added to both the x- and
    y-coordinates of the current VecPosition. This changes the current
    VecPosition itself.
    \param d a double value which has to be added to both the x- and
    y-coordinates of the current VecPosition */
void VecPosition::operator += ( const double &d )
                              {
    m_x += d;
    m_y += d;
}

/*! Overloaded version of the difference-assignment operator for
    VecPositions.  It returns the difference between the current
    VecPosition and the given VecPosition by subtracting their x- and
    y-coordinates. This changes the current VecPosition itself.

    \param p a VecPosition which has to be subtracted from the current
    VecPosition */
void VecPosition::operator -=( const VecPosition &p )
                             {
    m_x -= p.m_x;
    m_y -= p.m_y;
}

/*! Overloaded version of the difference-assignment operator for
    subtracting a given double value from a VecPosition. The double
    value is subtracted from both the x- and y-coordinates of the
    current VecPosition. This changes the current VecPosition itself.

    \param d a double value which has to be subtracted from both the x- and
    y-coordinates of the current VecPosition */
void VecPosition::operator -=( const double &d )
                             {
    m_x -= d;
    m_y -= d;
}

/*! Overloaded version of the multiplication-assignment operator for
    VecPositions. It returns the product of the current VecPosition
    and the given VecPosition by multiplying their x- and
    y-coordinates. This changes the current VecPosition itself.

    \param p a VecPosition by which the current VecPosition has to be
    multiplied */
void VecPosition::operator *=( const VecPosition &p )
                             {
    m_x *= p.m_x;
    m_y *= p.m_y;
}

/*! Overloaded version of the multiplication-assignment operator for
    multiplying a VecPosition by a given double value. Both the x- and
    y-coordinates of the current VecPosition are multiplied by this
    value. This changes the current VecPosition itself.

    \param d a double value by which both the x- and y-coordinates of the
    current VecPosition have to be multiplied */
void VecPosition::operator *=( const double &d )
                             {
    m_x *= d;
    m_y *= d;
}

/*! Overloaded version of the division-assignment operator for
    VecPositions. It returns the quotient of the current VecPosition
    and the given VecPosition by dividing their x- and
    y-coordinates. This changes the current VecPosition itself.

    \param p a VecPosition by which the current VecPosition is divided */
void VecPosition::operator /=( const VecPosition &p )
                             {
    m_x /= p.m_x;
    m_y /= p.m_y;
}

/*! Overloaded version of the division-assignment operator for
    dividing a VecPosition by a given double value. Both the x- and
    y-coordinates of the current VecPosition are divided by this
    value. This changes the current VecPosition itself.

    \param d a double value by which both the x- and y-coordinates of the
    current VecPosition have to be divided */
void VecPosition::operator /=( const double &d )
                             {
    m_x /= d;
    m_y /= d;
}

/*! Overloaded version of the inequality operator for VecPositions. It
    determines whether the current VecPosition is unequal to the given
    VecPosition by comparing their x- and y-coordinates.

    \param p a VecPosition
    \return true when either the x- or y-coordinates of the given VecPosition
    and the current VecPosition are different; false otherwise */
bool VecPosition::operator !=( const VecPosition &p )
{
    return ( ( m_x != p.m_x ) || ( m_y != p.m_y ) );
}

/*! Overloaded version of the inequality operator for comparing a
    VecPosition to a double value. It determines whether either the x-
    or y-coordinate of the current VecPosition is unequal to the given
    double value.

    \param d a double value with which both the x- and y-coordinates of the
    current VecPosition have to be compared.
    \return true when either the x- or y-coordinate of the current VecPosition
    is unequal to the given double value; false otherwise */
bool VecPosition::operator !=( const double &d )
{
    return ( ( m_x != d ) || ( m_y != d ) );
}

/*! Overloaded version of the equality operator for VecPositions. It
    determines whether the current VecPosition is equal to the given
    VecPosition by comparing their x- and y-coordinates.

    \param p a VecPosition
    \return true when both the x- and y-coordinates of the given
    VecPosition and the current VecPosition are equal; false
    otherwise */
bool VecPosition::operator ==( const VecPosition &p )
{
    return ( ( m_x == p.m_x ) && ( m_y == p.m_y ) );
}

/*! Overloaded version of the equality operator for comparing a
    VecPosition to a double value. It determines whether both the x-
    and y-coordinates of the current VecPosition are equal to the
    given double value.

    \param d a double value with which both the x- and y-coordinates of the
    current VecPosition have to be compared.
    \return true when both the x- and y-coordinates of the current VecPosition
    are equal to the given double value; false otherwise */
bool VecPosition::operator ==( const double &d )
{
    return ( ( m_x == d ) && ( m_y == d ) );
}

/*! Overloaded version of the C++ output operator for
    VecPositions. This operator makes it possible to use VecPositions
    in output statements (e.g.  cout << v). The x- and y-coordinates
    of the VecPosition are printed in the format (x,y).

    \param os output stream to which information should be written
    \param v a VecPosition which must be printed
    \return output stream containing (x,y) */
ostream& operator <<( ostream &os, VecPosition v )
{
    return ( os << "( " << v.m_x << ", " << v.m_y << " )" );
}

/*! This method writes the current VecPosition to standard output. It
    can also print a polar representation of the current VecPosition.

    \param cs a CoordSystemtT indicating whether a POLAR or CARTESIAN
     representation of the current VecPosition should be printed */
void VecPosition::show( CoordSystemT cs )
{
    if( cs == CARTESIAN )
        cout << *this << endl;
    else
        cout << "( r: " << getMagnitude( ) << ", phi: " << getDirection( ) << "  )";
}

/*! This method writes the current VecPosition to a string. It can
    also write a polar representation of the current VecPosition.

    \param cs a CoordSystemtT indicating whether a POLAR or CARTESIAN
     representation of the current VecPosition should be written
    \return a string containing a polar or Cartesian representation of the
    current VecPosition depending on the value of the boolean argument */
string VecPosition::str( CoordSystemT cs )
{
    char buf[ 1024 ];

    if( cs == CARTESIAN )
        sprintf( buf, "( %f, %f )", getX( ), getY( ) );
    else
        sprintf( buf, "( r: %f, phi: %f )", getMagnitude( ), getDirection( ) );

    string str( buf );
    return ( str );
}

/*! Set method for the x-coordinate of the current VecPosition.

    \param dX a double value representing a new x-coordinate
    \return a boolean indicating whether the update was successful */
bool VecPosition::setX( double dX )
{
    m_x = dX;
    return ( true );
}

/*! Get method for the x-coordinate of the current VecPosition.

    \return the x-coordinate of the current VecPosition */
double VecPosition::getX( ) const
{
    return ( m_x );
}

/*! Set method for the y-coordinate of the current VecPosition.

    \param dY a double value representing a new y-coordinate
    \return a boolean indicating whether the update was successful */
bool VecPosition::setY( double dY )
{
    m_y = dY;
    return ( true );
}

/*! Get method for the y-coordinate of the current VecPosition.

    \return the y-coordinate of the current VecPosition */
double VecPosition::getY( ) const
{
    return ( m_y );
}

/*! This method (re)sets the coordinates of the current
    VecPosition. The given coordinates can either be polar or
    Cartesian coordinates. This is indicated by the value of the third
    argument.

    \param dX a double value indicating either a new Cartesian
    x-coordinate when cs=CARTESIAN or a new polar r-coordinate
    (distance) when cs=POLAR

    \param dY a double value indicating either a new Cartesian
    y-coordinate when cs=CARTESIAN or a new polar phi-coordinate
    (angle) when cs=POLAR

    \param cs a CoordSystemT indicating whether x and y denote
    cartesian coordinates or polar coordinates */
void VecPosition::setVecPosition( double dX, double dY, CoordSystemT cs)
{
    if( cs == CARTESIAN )
    {
        m_x = dX;
        m_y = dY;
    }
    else
        *this = getVecPositionFromPolar( dX, dY );
}

/*! This method determines the distance between the current
    VecPosition and a given VecPosition. This is equal to the
    magnitude (length) of the vector connecting the two positions
    which is the difference vector between them.

    \param p a Vecposition
    \return the distance between the current VecPosition and the given
    VecPosition */
double VecPosition::getDistanceTo( const VecPosition &p ) const
{
    return ( ( *this - p ).getMagnitude( ) );
}

/*! This method adjusts the coordinates of the current VecPosition in
    such a way that the magnitude of the corresponding vector equals
    the double value which is supplied as an argument. It thus scales
    the vector to a given length by multiplying both the x- and
    y-coordinates by the quotient of the argument and the current
    magnitude. This changes the VecPosition itself.

    \param d a double value representing a new magnitude

    \return the result of scaling the vector corresponding with the
    current VecPosition to the given magnitude thus yielding a
    different VecPosition */
VecPosition VecPosition::setMagnitude( double d )
{
    if( getMagnitude( ) > EPSILON )
        ( *this ) *= ( d / getMagnitude( ) );

    return ( *this );
}

/*! This method determines the magnitude (length) of the vector
    corresponding with the current VecPosition using the formula of
    Pythagoras.

    \return the length of the vector corresponding with the current
    VecPosition */
double VecPosition::getMagnitude( ) const
{
    return ( sqrt( m_x * m_x + m_y * m_y ) );
}

/*! This method determines the direction of the vector corresponding
    with the current VecPosition (the phi-coordinate in polar
    representation) using the arc tangent function. Note that the
    signs of x and y have to be taken into account in order to
    determine the correct quadrant.

    \return the direction in degrees of the vector corresponding with
    the current VecPosition */
AngDeg VecPosition::getDirection( ) const
{
    return ( atan2Deg( m_y, m_x ) );
}

/*! This method determines whether the current VecPosition is in front
    of a given VecPosition, i.e. whether the x-coordinate of the
    current VecPosition is larger than the x-coordinate of the given
    VecPosition.

    \param p a VecPosition to which the current VecPosition must be compared
    \return true when the current VecPosition is in front of the given
    VecPosition; false otherwise */
bool VecPosition::isInFrontOf( const VecPosition &p )
{
    return ( ( m_x > p.getX( ) ) ? true : false );
}

/*! This method determines whether the x-coordinate of the current
    VecPosition is in front of (i.e. larger than) a given double
    value.

    \param d a double value to which the current x-coordinate must be
    compared

    \return true when the current x-coordinate is in front of the
    given value; false otherwise */
bool VecPosition::isInFrontOf( const double &d )
{
    return ( ( m_x > d ) ? true : false );
}

/*! This method determines whether the current VecPosition is behind a
    given VecPosition, i.e. whether the x-coordinate of the current
    VecPosition is smaller than the x-coordinate of the given
    VecPosition.

    \param p a VecPosition to which the current VecPosition must be
    compared

    \return true when the current VecPosition is behind the given
    VecPosition; false otherwise */
bool VecPosition::isBehindOf( const VecPosition &p )
{
    return ( ( m_x < p.getX( ) ) ? true : false );
}

/*! This method determines whether the x-coordinate of the current
    VecPosition is behind (i.e. smaller than) a given double value.

    \param d a double value to which the current x-coordinate must be
    compared

    \return true when the current x-coordinate is behind the given
    value; false otherwise */
bool VecPosition::isBehindOf( const double &d )
{
    return ( ( m_x < d ) ? true : false );
}

/*! This method determines whether the current VecPosition is to the
    left of a given VecPosition, i.e. whether the y-coordinate of the
    current VecPosition is smaller than the y-coordinate of the given
    VecPosition.

    \param p a VecPosition to which the current VecPosition must be
    compared

    \return true when the current VecPosition is to the left of the
    given VecPosition; false otherwise */
bool VecPosition::isLeftOf( const VecPosition &p )
{
    return ( ( m_y < p.getY( ) ) ? true : false );
}

/*! This method determines whether the y-coordinate of the current
    VecPosition is to the left of (i.e. smaller than) a given double
    value.

    \param d a double value to which the current y-coordinate must be
    compared

    \return true when the current y-coordinate is to the left of the
    given value; false otherwise */
bool VecPosition::isLeftOf( const double &d )
{
    return ( ( m_y < d ) ? true : false );
}

/*! This method determines whether the current VecPosition is to the
    right of a given VecPosition, i.e. whether the y-coordinate of the
    current VecPosition is larger than the y-coordinate of the given
    VecPosition.

    \param p a VecPosition to which the current VecPosition must be
    compared

    \return true when the current VecPosition is to the right of the
    given VecPosition; false otherwise */
bool VecPosition::isRightOf( const VecPosition &p )
{
    return ( ( m_y > p.getY( ) ) ? true : false );
}

/*! This method determines whether the y-coordinate of the current
    VecPosition is to the right of (i.e. larger than) a given double
    value.

    \param d a double value to which the current y-coordinate must be
    compared

    \return true when the current y-coordinate is to the right of the
    given value; false otherwise */
bool VecPosition::isRightOf( const double &d )
{
    return ( ( m_y > d ) ? true : false );
}

/*! This method determines whether the current VecPosition is in
    between two given VecPositions when looking in the x-direction,
    i.e. whether the current VecPosition is in front of the first
    argument and behind the second.

    \param p1 a VecPosition to which the current VecPosition must be
    compared

    \param p2 a VecPosition to which the current VecPosition must be
    compared

    \return true when the current VecPosition is in between the two
    given VecPositions when looking in the x-direction; false
    otherwise */
bool VecPosition::isBetweenX( const VecPosition &p1, const VecPosition &p2 )
{
    return ( ( isInFrontOf( p1 ) && isBehindOf( p2 ) ) ? true : false );
}

/*! This method determines whether the x-coordinate of the current
    VecPosition is in between two given double values, i.e. whether
    the x-coordinate of the current VecPosition is in front of the
    first argument and behind the second.

    \param d1 a double value to which the current x-coordinate must be
    compared

    \param d2 a double value to which the current x-coordinate must be
    compared

    \return true when the current x-coordinate is in between the two
    given values; false otherwise */
bool VecPosition::isBetweenX( const double &d1, const double &d2 )
{
    return ( ( isInFrontOf( d1 ) && isBehindOf( d2 ) ) ? true : false );
}

/*! This method determines whether the current VecPosition is in
    between two given VecPositions when looking in the y-direction,
    i.e. whether the current VecPosition is to the right of the first
    argument and to the left of the second.

    \param p1 a VecPosition to which the current VecPosition must be
    compared

    \param p2 a VecPosition to which the current VecPosition must be
    compared

    \return true when the current VecPosition is in between the two
    given VecPositions when looking in the y-direction; false
    otherwise */
bool VecPosition::isBetweenY( const VecPosition &p1, const VecPosition &p2 )
{
    return ( ( isRightOf( p1 ) && isLeftOf( p2 ) ) ? true : false );
}

/*! This method determines whether the y-coordinate of the current
    VecPosition is in between two given double values, i.e. whether
    the y-coordinate of the current VecPosition is to the right of the
    first argument and to the left of the second.

    \param d1 a double value to which the current y-coordinate must be
    compared

    \param d2 a double value to which the current y-coordinate must be
    compared

    \return true when the current y-coordinate is in between the two
    given values; false otherwise */
bool VecPosition::isBetweenY( const double &d1, const double &d2 )
{
    return ( ( isRightOf( d1 ) && isLeftOf( d2 ) ) ? true : false );
}


bool VecPosition::isInCircle( const Circle &c) const 
{
	if (pow(getX() - c.getCenter().getX(), 2) + pow(getY() - c.getCenter().getY(), 2)
		< pow(c.getRadius(), 2))
		return true;
	else
		return false;
}


/*! This method normalizes a VecPosition by setting the magnitude of
    the corresponding vector to 1. This thus changes the VecPosition
    itself.

    \return the result of normalizing the current VecPosition thus
    yielding a different VecPosition */
VecPosition VecPosition::normalize( )
{
    return ( setMagnitude( 1.0 ) );
}

/*! This method rotates the vector corresponding to the current
    VecPosition over a given angle thereby changing the current
    VecPosition itself. This is done by calculating the polar
    coordinates of the current VecPosition and adding the given angle
    to the phi-coordinate in the polar representation. The polar
    coordinates are then converted back to Cartesian coordinates to
    obtain the desired result.

    \param angle an angle in degrees over which the vector
    corresponding to the current VecPosition must be rotated

    \return the result of rotating the vector corresponding to the
    current VecPosition over the given angle thus yielding a different
    VecPosition */
VecPosition VecPosition::rotate( AngRad angle )
{
    VecPosition v1(0,0);
    VecPosition v2 = directVector(this->Angle() + angle);
    v1 =  v2 * this->getMagnitude() ;
    this->m_x = v1.getX();
    this->m_y = v1.getY();
    return v1;

    //  // determine the polar representation of the current VecPosition
    //  double dMag    = this->getMagnitude( );
    //  double dNewDir = this->getDirection( ) + angle;  // add rotation angle to phi
    //  setVecPosition( dMag, dNewDir, POLAR );          // convert back to Cartesian
    //  return ( *this );
}



/*! This method converts the coordinates of the current VecPosition
    (which are represented in an global coordinate system with the
    origin at (0,0)) into relative coordinates in a different
    coordinate system (e.g. relative to a player). The new coordinate
    system is defined by the arguments to the method. The relative
    coordinates are now obtained by aligning the relative coordinate
    system with the global coordinate system using a translation to
    make both origins coincide followed by a rotation to align the
    axes.

    \param origin the origin of the relative coordinate frame

    \param ang the angle between the world frame and the relative
    frame (reasoning from the world frame)

    \return the result of converting the current global VecPosition
    into a relative VecPosition */
VecPosition VecPosition::globalToRelative( VecPosition origin, AngDeg ang )
{
    // convert global coordinates into relative coordinates by aligning
    // relative frame and world frame. First perform translation to make
    // origins of both frames coincide. Then perform rotation to make
    // axes of both frames coincide (use negative angle since you rotate
    // relative frame to world frame).
    *this -= origin;
    return ( rotate( -ang ) );
}

/*! This method converts the coordinates of the current VecPosition
    (which are represented in a relative coordinate system) into
    global coordinates in the world frame (with origin at (0,0)). The
    relative coordinate system is defined by the arguments to the
    method. The global coordinates are now obtained by aligning the
    world frame with the relative frame using a rotation to align the
    axes followed by a translation to make both origins coincide.

    \param origin the origin of the relative coordinate frame

    \param ang the angle between the world frame and the relative
    frame (reasoning from the world frame)

    \return the result of converting the current relative VecPosition
    into an global VecPosition */
VecPosition VecPosition::relativeToGlobal( VecPosition origin, AngDeg ang )
{
    // convert relative coordinates into global coordinates by aligning
    // world frame and relative frame. First perform rotation to make
    // axes of both frames coincide (use positive angle since you rotate
    // world frame to relative frame). Then perform translation to make
    // origins of both frames coincide.
    rotate( ang );
    *this += origin;
    return ( *this );
}

/*! This method returns a VecPosition that lies somewhere on the
    vector between the current VecPosition and a given
    VecPosition. The desired position is specified by a given fraction
    of this vector (e.g. 0.5 means exactly in the middle of the
    vector). The current VecPosition itself is left unchanged.

    \param p a VecPosition which defines the vector to the current
    VecPosition

    \param dFrac double representing the fraction of the connecting
    vector at which the desired VecPosition lies.

    \return the VecPosition which lies at fraction dFrac on the vector
    connecting p and the current VecPosition */
VecPosition VecPosition::getVecPositionOnLineFraction( VecPosition &p,
                                                       double      dFrac )
{
    // determine point on line that lies at fraction dFrac of whole line
    // example: this --- 0.25 ---------  p
    // formula: this + dFrac * ( p - this ) = this - dFrac * this + dFrac * p =
    //          ( 1 - dFrac ) * this + dFrac * p
    return ( ( *this ) * ( 1.0 - dFrac ) + ( p * dFrac ) );
}

/*! This method converts a polar representation of a VecPosition into
    a Cartesian representation.

    \param dMag a double representing the polar r-coordinate, i.e. the
    distance from the point to the origin

    \param ang the angle that the polar vector makes with the x-axis,
    i.e. the polar phi-coordinate

    \return the result of converting the given polar representation
    into a Cartesian representation thus yielding a Cartesian
    VecPosition */
VecPosition VecPosition::getVecPositionFromPolar( double dMag, AngDeg ang )
{
    // cos(phi) = x/r <=> x = r*cos(phi); sin(phi) = y/r <=> y = r*sin(phi)
    return ( VecPosition( dMag * cosDeg( ang ), dMag * sinDeg( ang ) ) );
}

/*! This method normalizes an angle. This means that the resulting
    angle lies between -180 and 180 degrees.

    \param angle the angle which must be normalized

    \return the result of normalizing the given angle */
AngDeg VecPosition::normalizeAngle( AngDeg angle )
{
    while( angle > 180.0  ) angle -= 360.0;
    while( angle < -180.0 ) angle += 360.0;

    return ( angle );
}



bool VecPosition::isNan(double _w)
{
	return (isnan(getX()) || isnan(getY()) || isnan(_w) || isinf(getX()) || isinf(getY()) || isinf(_w));
}

bool VecPosition::isInvalidNumber(double n1, double n2, double n3, double n4, double n5)
{
	return (isnan(n1) || isnan(n2) || isnan(n3) || isnan(n4) || isnan(n5) || isinf(n1) || isinf(n2) || isinf(n3) || isinf(n4) || isinf(n5));
}



///**RRT
VecPosition VecPosition::cartesian_to_polar()
{
	VecPosition PC(POLAR);
	short int airt = Field::detect_airt(*this);
	if (airt == 1)
		PC.setY(atan(getY() / getX()));
	else if (airt == 4)
		PC.setY(atan(getY() / getX()) + 2 * M_PI);
	else
		PC.setY(atan(getY() / getX()) + M_PI);

	if (getX() == 0 && getY() > 0)
		PC.setY(M_PI / 2);
	if (getX() == 0 && getY() < 0)
		PC.setY(3 * M_PI / 2);
	if (getX() == 0 && getY() == 0)
		PC.setY(0);

	PC.setX(sqrt(pow(getX(), 2) + pow(getY(), 2)));
	return PC;
}
VecPosition VecPosition::polar_to_cartesian()
{
	return VecPosition(getX()*cos(getY()) /*+ current_position.x*/, getX()*sin(getY()) /*+ current_position.y*/);
}
VecPosition VecPosition::cartesian_to_polar_from_CP(const VecPosition &cp) const
{
	VecPosition CCFCP;	//cartesian coordinates from current position
	CCFCP.setX(getX() - cp.getX());
	CCFCP.setY(getY() - cp.getY());

	VecPosition PCFCP;	//polar coordinates from current position

	short int airt = Field::detect_airt(CCFCP);
	if (airt == 1)
		PCFCP.setY(atan(CCFCP.getY() / CCFCP.getX()));
	else if (airt == 4)
		PCFCP.setY(atan(CCFCP.getY() / CCFCP.getX()) + 2 * M_PI);
	else
		PCFCP.setY(atan(CCFCP.getY() / CCFCP.getX()) + M_PI);

	if (CCFCP.getX() == 0 && CCFCP.getY() > 0)
		PCFCP.setY(M_PI / 2);
	if (CCFCP.getX() == 0 && CCFCP.getY() < 0)
		PCFCP.setY(3 * M_PI / 2);
	if (CCFCP.getX() == 0 && CCFCP.getY() == 0)
		PCFCP.setY(0);

	PCFCP.setX(sqrt(pow(CCFCP.getX(), 2) + pow(CCFCP.getY(), 2)));
	return PCFCP;
}

VecPosition VecPosition::getNearestPositionOutsideOfCircle(const Circle &c) 	//!!!!!!!! wrong if two robots are near !!!!!!!
{

	/*VecPosition destination_position;
	VecPosition current_position;

	destination_position.setX(destination_position_C.x);
	destination_position.setY(destination_position_C.y);

	current_position.setX(current_position_C.x);
	current_position.setY(current_position_C.y);

	Line temp;
	temp = Line::makeLineFromTwoPoints(current_position, destination_position);

	VecPosition pos_solution1;
	VecPosition pos_solution2;
	temp.getCircleIntersectionPoints(balk, &pos_solution1, &pos_solution2);
	destination_position_C.x = pos_solution1.getX();
	destination_position_C.y = pos_solution1.getY();
	return destination_position_C;
	*/

	if (c.getRadius() <= getDistanceTo(c.getCenter()) )
		return *this;

	Circle second_circle;
	VecPosition pos_solution1;
	VecPosition pos_solution2;

	second_circle.setRadius((c.getRadius() - getDistanceTo(c.getCenter())));
	second_circle.setCenter(*this);

	second_circle.getIntersectionPoints(c, &pos_solution1, &pos_solution2);
	return pos_solution1;
}
VecPosition VecPosition::getPositionToward(const double &angle, const double &distance) const  //distance > 0 ->  x end be rast miravad	//distance < 0 ->  x end be chap miravad	//slope > 0 ->  y end be bala miravad	//slope < 0 ->  y end be paein miravad
{
	/*if (airt == 1)
	{
	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));
	end.x = start.x + sign(distance)*x;
	end.y = start.y + sign(distance)*slope*x;
	}
	else if (airt == 2)
	{
	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));
	end.x = start.x + sign(distance)*x;
	end.y = start.y + sign(distance)*slope*x;
	}
	else if (airt == 3)
	{
	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));
	end.x = start.x - sign(distance)*x;
	end.y = start.y - sign(distance)*slope*x;
	}
	else
	{
	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));
	end.x = start.x - sign(distance)*x;
	end.y = start.y - sign(distance)*slope*x;
	}*/

	/*if (abs(slope) == numeric_limits<double>::infinity())
		return VecPosition(this->getX(), sign(slope)*(this->getY() + abs(distance)));

	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));

	return VecPosition(getX() + sign(distance)*x, getY() + sign(distance)*slope*x);*/

	return VecPosition((distance*cos(angle) + getX()), (distance*sin(angle) + getY()));
}

VecPosition VecPosition::getPositionToward(const VecPosition &destination, const double &distance) const
{
	/*return start.getPositionFromSlopeAndDistance(Bar::line_slope(start, destination), sign(destination.getX() - start.getX()) * distance);*/

	double alfa = getAngleToward(destination);
	return VecPosition((distance*cos(alfa) + getX()), (distance*sin(alfa) + getY()));
}

double VecPosition::getSlopeToward(const VecPosition &destination) const
{
	if ((destination.getX() - getX()) != 0)
		return (destination.getY() - getY()) / (destination.getX() - getX());
	else if (destination.getY() >= getY())
		return std::numeric_limits<double>::infinity();
	else
		return -std::numeric_limits<double>::infinity();
}

double VecPosition::getAngleToward(const VecPosition &destination) const
{
	if (abs(destination.getX() - getX()) < EPSILON && abs(destination.getY() - getY()) < EPSILON)
		return 0;
	double m = getSlopeToward(destination);
	double alfa = atan(m);
	if (destination.getX() < getX())
		alfa += M_PI;
	return alfa;
}

double VecPosition::getAngle()
{
	if (abs(getX()) < EPSILON && abs(getY()) < EPSILON)
		return 0;
	double m = VecPosition(0,0).getSlopeToward(*this);
	double alfa = atan(m);
	if (getX() < 0)
		alfa += M_PI;
	return alfa;
}



///**

/*****************************************************************************/
/********************** CLASS GEOMETRY ***************************************/
/*****************************************************************************/

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the
    length of a geometric series given its first element, the sum of the
    elements in the series and the constant ratio between the elements.
    Normally: s = a + ar + ar^2 + ...  + ar^n
    Now: dSum = dFirst + dFirst*dRatio + dFirst*dRatio^2 + .. + dFist*dRatio^n
    \param dFirst first term of the series
    \param dRatio ratio with which the the first term is multiplied
    \param dSum the total sum of all the serie
    \return the length(n in above example) of the series */
double Geometry::getLengthGeomSeries( double dFirst, double dRatio, double dSum )
{
    if( dRatio < 0 )
        cerr << "(Geometry:getLengthGeomSeries): negative ratio" << endl;

    // s = a + ar + ar^2 + .. + ar^n-1 and thus sr = ar + ar^2 + .. + ar^n
    // subtract: sr - s = - a + ar^n) =>  s(1-r)/a + 1 = r^n = temp
    // log r^n / n = n log r / log r = n = length
    double temp = (dSum * ( dRatio - 1 ) / dFirst) + 1;
    if( temp <= 0 )
        return -1.0;
    return log( temp ) / log( dRatio ) ;
}

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the sum of a
    geometric series given its first element, the ratio and the number of steps
    in the series
    Normally: s = a + ar + ar^2 + ...  + ar^n
    Now: dSum = dFirst + dFirst*dRatio + ... + dFirst*dRatio^dSteps
    \param dFirst first term of the series
    \param dRatio ratio with which the the first term is multiplied
    \param dSum the number of steps to be taken into account
    \return the sum of the series */
double Geometry::getSumGeomSeries( double dFirst, double dRatio, double dLength)
{
    // s = a + ar + ar^2 + .. + ar^n-1 and thus sr = ar + ar^2 + .. + ar^n
    // subtract: s - sr = a - ar^n) =>  s = a(1-r^n)/(1-r)
    return dFirst * ( 1 - pow( dRatio, dLength ) ) / ( 1 - dRatio ) ;
}

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the sum of an
    infinite geometric series given its first element and the constant ratio
    between the elements. Note that such an infinite series will only converge
    when 0<r<1.
    Normally: s = a + ar + ar^2 + ar^3 + ....
    Now: dSum = dFirst + dFirst*dRatio + dFirst*dRatio^2...
    \param dFirst first term of the series
    \param dRatio ratio with which the the first term is multiplied
    \return the sum of the series */
double Geometry::getSumInfGeomSeries( double dFirst, double dRatio )
{
    if( dRatio > 1 )
        cerr << "(Geometry:CalcLengthGeomSeries): series does not converge" <<endl;

    // s = a(1-r^n)/(1-r) with n->inf and 0<r<1 => r^n = 0
    return dFirst / ( 1 - dRatio );
}

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the first element
    of a geometric series given its element, the ratio and the number of steps
    in the series
    Normally: s = a + ar + ar^2 + ...  + ar^n
    Now: dSum = dFirst + dFirst*dRatio + ... + dFirst*dRatio^dSteps
    \param dSum sum of the series
    \param dRatio ratio with which the the first term is multiplied
    \param dSum the number of steps to be taken into account
    \return the first element (a) of a serie */
double Geometry::getFirstGeomSeries( double dSum, double dRatio, double dLength)
{
    // s = a + ar + ar^2 + .. + ar^n-1 and thus sr = ar + ar^2 + .. + ar^n
    // subtract: s - sr = a - ar^n) =>  s = a(1-r^n)/(1-r) => a = s*(1-r)/(1-r^n)
    return dSum *  ( 1 - dRatio )/( 1 - pow( dRatio, dLength ) ) ;
}

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the first element
    of an infinite geometric series given its first element and the constant
    ratio between the elements. Note that such an infinite series will only
    converge when 0<r<1.
    Normally: s = a + ar + ar^2 + ar^3 + ....
    Now: dSum = dFirst + dFirst*dRatio + dFirst*dRatio^2...
    \param dSum sum of the series
    \param dRatio ratio with which the the first term is multiplied
    \return the first term of the series */
double Geometry::getFirstInfGeomSeries( double dSum, double dRatio )
{
    if( dRatio > 1 )
        cerr << "(Geometry:getFirstInfGeomSeries):series does not converge" <<endl;

    // s = a(1-r^n)/(1-r) with r->inf and 0<r<1 => r^n = 0 => a = s ( 1 - r)
    return dSum * ( 1 - dRatio );
}

/*! This method performs the abc formula (Pythagoras' Theorem) on the given
    parameters and puts the result in *s1 en *s2. It returns the number of
    found coordinates.
    \param a a parameter in abc formula
    \param b b parameter in abc formula
    \param c c parameter in abc formula
    \param *s1 first result of abc formula
    \param *s2 second result of abc formula
    \return number of found x-coordinates */
int Geometry::abcFormula(double a, double b, double c, double *s1, double *s2)
{
    double dDiscr = b*b - 4*a*c;       // discriminant is b^2 - 4*a*c
    if (fabs(dDiscr) < EPSILON )       // if discriminant = 0
    {
        *s1 = -b / (2 * a);              //  only one solution
        return 1;
    }
    else if (dDiscr < 0)               // if discriminant < 0
        return 0;                        //  no solutions
    else                               // if discriminant > 0
    {
        dDiscr = sqrt(dDiscr);           //  two solutions
        *s1 = (-b + dDiscr ) / (2 * a);
        *s2 = (-b - dDiscr ) / (2 * a);
        return 2;
    }
}

/*****************************************************************************/
/********************* CLASS CIRCLE ******************************************/
/*****************************************************************************/

/*! This is the constructor of a circle.
    \param pos first point that defines the center of circle
    \param dR the radius of the circle
    \return circle with pos as center and radius as radius*/
Circle::Circle( VecPosition pos, double dR )
{
    setCircle( pos, dR );
}

/*! This is the constructor of a circle which initializes a circle with a
    radius of zero. */
Circle::Circle( )
{
    setCircle( VecPosition(-1000.0,-1000.0), 0);
}

/*! This method prints the circle information to the specified output stream
    in the following format: "c: (c_x,c_y), r: rad" where (c_x,c_y) denotes
    the center of the circle and rad the radius.
    \param os output stream to which output is written. */
void Circle::show( ostream& os)
{
    os << "c:" << m_posCenter << ", r:" << m_dRadius;
}

/*! This method sets the values of the circle.
    \param pos new center of the circle
    \param dR new radius of the circle
    ( > 0 )
    \return bool indicating whether radius was set */
bool Circle::setCircle( VecPosition pos, double dR )
{
    setCenter( pos );
    return setRadius( dR  );
}
/*! This method sets the radius of the circle.
    \param dR new radius of the circle ( > 0 )
    \return bool indicating whether radius was set */
bool Circle::setRadius( double dR )
{
    if( dR > 0 )
    {
        m_dRadius = dR;
        return true;
    }
    else
    {
        m_dRadius = 0.0;
        return false;
    }
}

/*! This method returns the radius of the circle.
    \return radius of the circle */
double Circle::getRadius() const
{
    return m_dRadius;
}

/*! This method sets the center of the circle.
    \param pos new center of the circle```````
    \return bool indicating whether center was set */
bool Circle::setCenter( VecPosition pos )
{
    m_posCenter = pos;
    return true;
}

/*! This method returns the center of the circle.
    \return center of the circle */
VecPosition Circle::getCenter() const
{
    return m_posCenter;
}

/*! This method returns the circumference of the circle.
    \return circumference of the circle */
double Circle::getCircumference()
{
    return 2.0*M_PI*getRadius();
}

/*! This method returns the area inside the circle.
    \return area inside the circle */
double Circle::getArea()
{
    return M_PI*getRadius()*getRadius();
}

/*! This method returns a boolean that indicates whether 'pos' is
    located inside the circle.

   \param pos position of which should be checked whether it is
   located in the circle

   \return bool indicating whether pos lies inside the circle */
bool Circle::isInside( VecPosition pos )
{
    return m_posCenter.getDistanceTo( pos ) < getRadius() ;
}

/*! This method returns the two possible intersection points between two
    circles. This method returns the number of solutions that were found.
    \param c circle with which intersection should be found
    \param p1 will be filled with first solution
    \param p2 will be filled with second solution
    \return number of solutions. */
int Circle::getIntersectionPoints( Circle c, VecPosition *p1, VecPosition *p2)
{
    double x0, y0, r0;
    double x1, y1, r1;

    x0 = getCenter( ).getX();
    y0 = getCenter( ).getY();
    r0 = getRadius( );
    x1 = c.getCenter( ).getX();
    y1 = c.getCenter( ).getY();
    r1 = c.getRadius( );

    double      d, dx, dy, h, a, x, y, p2_x, p2_y;

    // first calculate distance between two centers circles P0 and P1.
    dx = x1 - x0;
    dy = y1 - y0;
    d = sqrt(dx*dx + dy*dy);

    // normalize differences
    dx /= d; dy /= d;

    // a is distance between p0 and point that is the intersection point P2
    // that intersects P0-P1 and the line that crosses the two intersection
    // points P3 and P4.
    // Define two triangles: P0,P2,P3 and P1,P2,P3.
    // with distances a, h, r0 and b, h, r1 with d = a + b
    // We know a^2 + h^2 = r0^2 and b^2 + h^2 = r1^2 which then gives
    // a^2 + r1^2 - b^2 = r0^2 with d = a + b ==> a^2 + r1^2 - (d-a)^2 = r0^2
    // ==> r0^2 + d^2 - r1^2 / 2*d
    a = (r0*r0 + d*d - r1*r1) / (2.0 * d);

    // h is then a^2 + h^2 = r0^2 ==> h = sqrt( r0^2 - a^2 )
    double      arg = r0*r0 - a*a;
    h = (arg > 0.0) ? sqrt(arg) : 0.0;

    // First calculate P2
    p2_x = x0 + a * dx;
    p2_y = y0 + a * dy;

    // and finally the two intersection points
    x =  p2_x - h * dy;
    y =  p2_y + h * dx;
    p1->setVecPosition( x, y );
    x =  p2_x + h * dy;
    y =  p2_y - h * dx;
    p2->setVecPosition( x, y );

    return (arg < 0.0) ? 0 : ((arg == 0.0 ) ? 1 :  2);
}

/*! This method returns the size of the intersection area of two circles.
    \param c circle with which intersection should be determined
    \return size of the intersection area. */
double Circle::getIntersectionArea( Circle c )
{
    VecPosition pos1, pos2, pos3;
    double d, h, dArea;
    AngDeg ang;

    d = getCenter().getDistanceTo( c.getCenter() ); // dist between two centers
    if( d > c.getRadius() + getRadius() )           // larger than sum radii
        return 0.0;                                   // circles do not intersect
    if( d <= fabs(c.getRadius() - getRadius() ) )   // one totally in the other
    {
        double dR = fmin( c.getRadius(), getRadius() );// return area smallest circ
        return M_PI*dR*dR;
    }

    int iNrSol = getIntersectionPoints( c, &pos1, &pos2 );
    if( iNrSol != 2 )
        return 0.0;

    // the intersection area of two circles can be divided into two segments:
    // left and right of the line between the two intersection points p1 and p2.
    // The outside area of each segment can be calculated by taking the part
    // of the circle pie excluding the triangle from the center to the
    // two intersection points.
    // The pie equals pi*r^2 * rad(2*ang) / 2*pi = 0.5*rad(2*ang)*r^2 with ang
    // the angle between the center c of the circle and one of the two
    // intersection points. Thus the angle between c and p1 and c and p3 where
    // p3 is the point that lies halfway between p1 and p2.
    // This can be calculated using ang = asin( d / r ) with d the distance
    // between p1 and p3 and r the radius of the circle.
    // The area of the triangle is 2*0.5*h*d.

    pos3 = pos1.getVecPositionOnLineFraction( pos2, 0.5 );
    d = pos1.getDistanceTo( pos3 );
    h = pos3.getDistanceTo( getCenter() );
    ang = asin( d / getRadius() );

    dArea = ang*getRadius()*getRadius();
    dArea = dArea - d*h;

    // and now for the other segment the same story
    h = pos3.getDistanceTo( c.getCenter() );
    ang = asin( d / c.getRadius() );
    dArea = dArea + ang*c.getRadius()*c.getRadius();
    dArea = dArea - d*h;

    return dArea;
}

VecPosition Circle::getNearestPointOnCircleTo(const VecPosition &point) const
{
	Line center_to_point = Line::makeLineFromTwoPoints(getCenter(),point);
	VecPosition pos_solution1, pos_solution2;
	center_to_point.getCircleIntersectionPoints(*this, &pos_solution1, &pos_solution2);
	return point.getNearer(pos_solution1, pos_solution2);
}

VecPosition	Circle::getPositionOnCircleWithAngle(double rad_ang)
{
	return getCenter().getPositionToward(rad_ang, getRadius());
}

/*****************************************************************************/
/***********************  CLASS LINE *****************************************/
/*****************************************************************************/

/*! This constructor creates a line by given the three coefficents of the line.
    A line is specified by the formula ay + bx + c = 0.
    \param dA a coefficients of the line
    \param dB b coefficients of the line
    \param dC c coefficients of the line */
Line::Line()
{
    m_a = 0;
    m_b = 0;
    m_c = 0;
}
Line::Line( double dA, double dB, double dC )
{
    m_a = dA;
    m_b = dB;
    m_c = dC;
}

/*! This function prints the line to the specified output stream in the
    format y = ax + b.
    \param os output stream to which output is written
    \param l line that is written to output stream
    \return output sream to which output is appended. */
ostream& operator <<(ostream & os, Line l)
{
    double a = l.getACoefficient();
    double b = l.getBCoefficient();
    double c = l.getCCoefficient();

    // ay + bx + c = 0 -> y = -b/a x - c/a
    if( a == 0 )
        os << "x = " << -c/b;
    else
    {
        os << "y = ";
        if( b != 0 )
            os << -b/a << "x ";
        if( c > 0 )
            os << "- " <<  fabs(c/a);
        else if( c < 0 )
            os << "+ " <<  fabs(c/a);
    }
    return os;
}

/*! This method prints the line information to the specified output stream.
    \param os output stream to which output is written. */
void Line::show( ostream& os)
{
    os << *this;
}


bool Line::operator != (const Line &l)
{
    return ( ( getACoefficient() != l.getACoefficient() ) || ( getBCoefficient() != l.getBCoefficient() ) || ( getCCoefficient() != l.getCCoefficient() ) );
}

bool Line::operator == (const Line &l)
{
    return ( ( getACoefficient() == l.getACoefficient() ) && ( getBCoefficient() == l.getBCoefficient() ) && ( getCCoefficient() == l.getCCoefficient() ) );
}



/*! This method returns the intersection point between the current Line and
    the specified line.
    \param line line with which the intersection should be calculated.
    \return VecPosition position that is the intersection point. */
VecPosition Line::getIntersection( Line line )
{
    VecPosition pos;
    double x, y;
    if( ( m_a / m_b ) ==  (line.getACoefficient() / line.getBCoefficient() ))
        return pos; // lines are parallel, no intersection
    if( m_a == 0 )            // bx + c = 0 and a2*y + b2*x + c2 = 0 ==> x = -c/b
    {                          // calculate x using the current line
        x = -m_c/m_b;                // and calculate the y using the second line
        y = line.getYGivenX(x);
    }
    else if( line.getACoefficient() == 0 )
    {                         // ay + bx + c = 0 and b2*x + c2 = 0 ==> x = -c2/b2
        x = -line.getCCoefficient()/line.getBCoefficient(); // calculate x using
        y = getYGivenX(x);       // 2nd line and calculate y using current line
    }
    // ay + bx + c = 0 and a2y + b2*x + c2 = 0
    // y = (-b2/a2)x - c2/a2
    // bx = -a*y - c =>  bx = -a*(-b2/a2)x -a*(-c2/a2) - c ==>
    // ==> a2*bx = a*b2*x + a*c2 - a2*c ==> x = (a*c2 - a2*c)/(a2*b - a*b2)
    // calculate x using the above formula and the y using the current line
    else
    {
        x = (m_a*line.getCCoefficient() - line.getACoefficient()*m_c)/
            (line.getACoefficient()*m_b - m_a*line.getBCoefficient());
        y = getYGivenX(x);
    }

    return VecPosition( x, y );
}


/*! This method calculates the intersection points between the current line
    and the circle specified with as center 'posCenter' and radius 'dRadius'.
    The number of solutions are returned and the corresponding points are put
    in the third and fourth argument of the method
    \param c circle with which intersection points should be found
    \param posSolution1 first intersection (if any)
    \param posSolution2 second intersection (if any) */
int Line::getCircleIntersectionPoints( Circle circle,
                                       VecPosition *posSolution1, VecPosition *posSolution2 )
{
    int    iSol;
    double dSol1=0, dSol2=0;
    double h = circle.getCenter().getX();
    double k = circle.getCenter().getY();

    // line:   x = -c/b (if a = 0)
    // circle: (x-h)^2 + (y-k)^2 = r^2, with h = center.x and k = center.y
    // fill in:(-c/b-h)^2 + y^2 -2ky + k^2 - r^2 = 0
    //         y^2 -2ky + (-c/b-h)^2 + k^2 - r^2 = 0
    // and determine solutions for y using abc-formula
    if( fabs(m_a) < EPSILON )
    {
        iSol = Geometry::abcFormula( 1, -2*k, ((-m_c/m_b) - h)*((-m_c/m_b) - h)
                                     + k*k - circle.getRadius()*circle.getRadius(), &dSol1, &dSol2);
        posSolution1->setVecPosition( (-m_c/m_b), dSol1 );
        posSolution2->setVecPosition( (-m_c/m_b), dSol2 );
        return iSol;
    }

    // ay + bx + c = 0 => y = -b/a x - c/a, with da = -b/a and db = -c/a
    // circle: (x-h)^2 + (y-k)^2 = r^2, with h = center.x and k = center.y
    // fill in:x^2 -2hx + h^2 + (da*x-db)^2 -2k(da*x-db) + k^2 - r^2 = 0
    //         x^2 -2hx + h^2 + da^2*x^2 + 2da*db*x + db^2 -2k*da*x -2k*db
    //                                                         + k^2 - r^2 = 0
    //       (1+da^2)*x^2 + 2(da*db-h-k*da)*x + h2 + db^2  -2k*db + k^2 - r^2 = 0
    // and determine solutions for x using abc-formula
    // fill in x in original line equation to get y coordinate
    double da = -m_b/m_a;
    double db = -m_c/m_a;

    double dA = 1 + da*da;
    double dB = 2*( da*db - h - k*da );
    double dC = h*h + db*db-2*k*db + k*k - circle.getRadius()*circle.getRadius();

    iSol = Geometry::abcFormula( dA, dB, dC, &dSol1, &dSol2 );

    posSolution1->setVecPosition( dSol1, da*dSol1 + db );
    posSolution2->setVecPosition( dSol2, da*dSol2 + db );
    return iSol;

}

/*! This method returns the tangent line to a VecPosition. This is the line
    between the specified position and the closest point on the line to this
    position.
    \param pos VecPosition point with which tangent line is calculated.
    \return Line line tangent to this position */
Line Line::getTangentLine( VecPosition pos )
{
    // ay + bx + c = 0 -> y = (-b/a)x + (-c/a)
    // tangent: y = (a/b)*x + C1 -> by - ax + C2 = 0 => C2 = ax - by
    // with pos.y = y, pos.x = x
    return Line( m_b, -m_a, m_a*pos.getX() - m_b*pos.getY() );
}

/*! This method returns the closest point on a line to a given position.
    \param pos point to which closest point should be determined
    \return VecPosition closest point on line to 'pos'. */
VecPosition Line::getPointOnLineClosestTo( VecPosition pos )
{
    Line l2 = getTangentLine( pos );  // get tangent line
    return getIntersection( l2 );     // and intersection between the two lines
}

/*! This method returns the distance between a specified position and the
    closest point on the given line.
    \param pos position to which distance should be calculated
    \return double indicating the distance to the line. */
double Line::getDistanceWithPoint( VecPosition pos )
{
    return pos.getDistanceTo( getPointOnLineClosestTo( pos ) );
}

/*! This method determines whether the projection of a point on the
    current line lies between two other points ('point1' and 'point2')
    that lie on the same line.

    \param pos point of which projection is checked.
    \param point1 first point on line
    \param point2 second point on line
    \return true when projection of 'pos' lies between 'point1' and 'point2'.*/
bool Line::isInBetween( VecPosition pos, VecPosition point1,VecPosition point2)
{
    pos          = getPointOnLineClosestTo( pos ); // get closest point
    double dDist = point1.getDistanceTo( point2 ); // get distance between 2 pos

    // if the distance from both points to the projection is smaller than this
    // dist, the pos lies in between.
    return pos.getDistanceTo( point1 ) <= dDist &&
            pos.getDistanceTo( point2 ) <= dDist;
}

/*! This method calculates the y coordinate given the x coordinate
    \param x coordinate
    \return y coordinate on this line */
double Line::getYGivenX( double x )
{
    if( m_a == 0 )
    {
        //logStatus("bad input in getYGivenX" , QColor("red") );
        cerr << "(Line::getYGivenX) Cannot calculate Y coordinate: " ;
        show( cerr );
        cerr << endl;
        return 0;
    }
    // ay + bx + c = 0 ==> ay = -(b*x + c)/a
    return - ( m_b * x + m_c ) / m_a ;
}

/*! This method calculates the x coordinate given the x coordinate
    \param y coordinate
    \return x coordinate on this line */
double Line::getXGivenY( double y )
{
    if( m_b == 0 )
    {
        cerr << "(Line::getXGivenY) Cannot calculate X coordinate\n" ;
        return 0;
    }
    // ay + bx + c = 0 ==> bx = -(a*y + c)/a
    return - ( m_a * y + m_c ) / m_b ;
}

/*! This method creates a line given two points.
    \param pos1 first point
    \param pos2 second point
    \return line that passes through the two specified points. */
Line Line::makeLineFromTwoPoints( VecPosition pos1, VecPosition pos2 )
{
    // 1*y + bx + c = 0 => y = -bx - c
    // with -b the direction coefficient (or slope)
    // and c = - y - bx
    double dA, dB, dC;
    double dTemp = pos2.getX() - pos1.getX(); // determine the slope
    if( fabs(dTemp) < EPSILON )
    {
        // ay + bx + c = 0 with vertical slope=> a = 0, b = 1
        dA = 0.0;
        dB = 1.0;
    }
    else
    {
        // y = (-b)x -c with -b the slope of the line
        dA = 1.0;
        dB = -(pos2.getY() - pos1.getY())/dTemp;
    }
    // ay + bx + c = 0 ==> c = -a*y - b*x
    dC =  - dA*pos2.getY()  - dB * pos2.getX();
    return Line( dA, dB, dC );
}

/*! This method creates a line given a position and an angle.
    \param vec position through which the line passes
    \param angle direction of the line.
    \return line that goes through position 'vec' with angle 'angle'. */
Line Line::makeLineFromPositionAndAngle( VecPosition vec, AngDeg angle )
{
    // calculate point somewhat further in direction 'angle' and make
    // line from these two points.
    return makeLineFromTwoPoints( vec, vec+VecPosition(1,angle,POLAR));
}

/*! This method returns the a coefficient from the line ay + bx + c = 0.
    \return a coefficient of the line. */
double Line::getACoefficient() const
{
    return m_a;
}

/*! This method returns the b coefficient from the line ay + bx + c = 0.
    \return b coefficient of the line. */
double Line::getBCoefficient() const
{
    return m_b;
}

/*! This method returns the c coefficient from the line ay + bx + c = 0.
    \return c coefficient of the line. */
double Line::getCCoefficient() const
{
    return m_c;
}

/*****************************************************************************/
/********************* CLASS GOAL_SHAPE **************************************/
/*****************************************************************************/

/*! This is the constructor of a Goal_Shape.
    \param scale first double that defines the size of Goal_Shape
    \return Goal_Shape with scale as size */
Goal_Shape::Goal_Shape( double scale )
{
    setGoal_Shape( scale );
}

/*! This is the constructor of a Goal_Shape which initializes a Goal_Shape with a
    scale of 500. */
Goal_Shape::Goal_Shape( )
{
    setGoal_Shape( PenaltyAreaRadius );//it was 500 before change
}

/*! This method prints the Goal_Shape information to the specified output stream
    in the following format: "s: scale"
    the scale of the Goal_Shape.
    \param os output stream to which output is written. */
void Goal_Shape::show( ostream& os)
{
    os << "s:" << m_dScale;
}

/*! This method sets the values of the Goal_Shape.
    \param scale new scale of the Goal_Shape
    ( > 0 )
    \return bool indicating whether Scale was set */
bool Goal_Shape::setGoal_Shape( double scale )
{
    if( scale > 0 )
    {
        m_dScale = scale ;
        front_line = Line::makeLineFromPositionAndAngle(VecPosition((FieldLength/2)-scale,0),90);
        up_circle = Circle(VecPosition((FieldLength/2),  (FreeBound/2)), scale) ;
        down_circle = Circle(VecPosition((FieldLength/2),  -(FreeBound/2)), scale) ;
        return true;
		
    }
    else
    {
        m_dScale = 0.0;
        up_circle = Circle(VecPosition((FieldLength/2),  (FreeBound/2)), 0.0) ;
        down_circle = Circle(VecPosition((FieldLength/2),  -(FreeBound/2)), 0.0) ;
        return false;
    }
}

/*! This method returns the Scale of the Goal_Shape.
    \return Scale of the Goal_Shape */
double Goal_Shape::getScale()
{
    return m_dScale;
}

/*! This method returns the Up Circle of the Goal_Shape.
    \return Up Circle of the Goal_Shape */
Circle Goal_Shape::get_Up_Circle()
{
    return up_circle ;
}

/*! This method returns the Down Circle of the Goal_Shape.
    \return Down Circle of the Goal_Shape */
Circle Goal_Shape::get_Down_Circle()
{
    return down_circle ;
}

/*! This method returns the Front Line of the Goal_Shape.
    \return Front Line of the Goal_Shape */
Line Goal_Shape::get_Front_Line()
{
    return front_line ;
}

/*! This method returns the circumference of the Goal_Shape.
    \return circumference of the Goal_Shape */
double Goal_Shape::getCircumference()
{
    return M_PI * getScale() + FreeBound ;
}

/*! This method returns the area inside the Goal_Shape.
    \return area inside the Goal_Shape */
double Goal_Shape::getArea()
{
    return 0.5 * M_PI * getScale() * getScale() + FreeBound * getScale() ;
}

/*! This method returns a boolean that indicates whether 'pos' is
    located inside the Goal_Shape.

   \param pos position of which should be checked whether it is
   located in the Goal_Shape

   \return bool indicating whether pos lies inside the Goal_Shape */
bool Goal_Shape::isInside( VecPosition pos )
{
    bool output_1 = (pos.getX() <  VecPosition((FieldLength/2),  (FreeBound/2)).getX()) && (pos.getX() > VecPosition((FieldLength/2),  (FreeBound/2)).getX() - getScale()) ;
    bool output_2 = (pos.getY() < VecPosition((FieldLength/2),  (FreeBound/2)).getY()) && (pos.getY() > VecPosition((FieldLength/2),  -(FreeBound/2)).getY()) ;
    bool output_3 = (pos.getDistanceTo(VecPosition((FieldLength/2),  (FreeBound/2))) < getScale()) || (pos.getDistanceTo(VecPosition((FieldLength/2),  -(FreeBound/2))) < getScale()) ;

    return output_1 && ( output_2 || output_3 ) ;
}

/*! This method calculates the intersection points between the current goal_shape
    and the specified line.
    \param line, line with which the intersection should be calculated.
    \return VecPosition position that is the intersection point. */
VecPosition Goal_Shape::getIntersection(Line line)
{
    VecPosition _temp_intersect_front_line = line.getIntersection(front_line);

    if( (_temp_intersect_front_line.getY() < VecPosition((FieldLength/2),  (FreeBound/2)).getY()) && (_temp_intersect_front_line.getY() > VecPosition((FieldLength/2),  -(FreeBound/2)).getY()) )
        return _temp_intersect_front_line ;
    else
    {
        AngRad temp_angle;
        VecPosition _temp_target;
        VecPosition _temp_intersect_circle[2] ;

        if( line.getCircleIntersectionPoints(up_circle, &(_temp_intersect_circle[0]), &(_temp_intersect_circle[1])) != 0 )
        {
            temp_angle = (_temp_intersect_circle[0] - VecPosition((FieldLength/2),  (FreeBound/2)) ).Angle();
            if( temp_angle >= M_PI / 2 && temp_angle <= M_PI )
                _temp_target = _temp_intersect_circle[0] ;
            else
                _temp_target = _temp_intersect_circle[1] ;

            if( _temp_target.getY() > VecPosition((FieldLength/2),  (FreeBound/2)).getY() )
                return _temp_target;
        }

        if( line.getCircleIntersectionPoints(down_circle, &(_temp_intersect_circle[0]), &(_temp_intersect_circle[1])) != 0 )
        {
            temp_angle = (_temp_intersect_circle[0] - VecPosition((FieldLength/2),  (FreeBound/2)) ).Angle();
            if( temp_angle <= -M_PI / 2 && temp_angle >= -M_PI )
                _temp_target = _temp_intersect_circle[0] ;
            else
                _temp_target = _temp_intersect_circle[1] ;

            if( _temp_target.getY() < VecPosition((FieldLength/2),  -(FreeBound/2)).getY() )
                return _temp_target;
        }
    }
    //logStatus("goal shabe intersection can't find any position" , QColor("red") );
    return VecPosition((FieldLength/2),0) + VecPosition(-100,0);
}


/***********************************************************************/
/********************* CLASS CONE **************************************/
/***********************************************************************/

/*! This is the constructor of a Cone.
    \param Ball first VecPosition that defines the size of Cone
    \param up_bar second VecPosition that defines the size of Cone
    \param down_bar thired VecPosition that defines the size of Cone
    \return Cone with Ball and up_bar and down_bar */
Cone::Cone( VecPosition Ball, VecPosition up_bar, VecPosition down_bar )
{
    set_ball(Ball) ;

    if(up_bar.getY() < down_bar.getY())
    {
        set_up_bar(down_bar) ;
        set_down_bar(up_bar) ;
    }
    else
    {
        set_up_bar(up_bar) ;
        set_down_bar(down_bar) ;
    }

    Max_Bar = Rad2Deg( (m_dup_bar - m_dBall).Angle(true) ) ;
    Min_Bar = Rad2Deg( (m_ddown_bar - m_dBall).Angle(true) ) ;
    if(Min_Bar > Max_Bar)
    {
        swap(Min_Bar, Max_Bar) ;
        swap(m_ddown_bar, m_dup_bar) ;
    }
    Alfa = Max_Bar - Min_Bar ;
}

/*! This method prints the Cone information to the specified output stream
    in the following format:
    "b: Ball_pos" the position of the ball.
    "ub: Up_Bar_pos" the position of the up bar.
    "db: Down_Bar_pos" the position of the down bar.
    \param os output stream to which output is written. */
void Cone::show( ostream& os)
{
    os << "b:"  << m_dBall << endl ;
    os << "ub:" << m_dup_bar << endl ;
    os << "db:" << m_ddown_bar ;
}


/*! This method sets the values of the Cone.
    \param ball new position of ball of the Cone
    \return bool indicating whether ball was set */
void Cone::set_ball(VecPosition Ball)
{
    m_dBall = Ball ;
}

/*! This method sets the values of the Up Bar.
    \param up_bar new position of Up Bar of the Cone
    \return bool indicating whether up_bar was set */
void Cone::set_up_bar(VecPosition up_bar)
{
    m_dup_bar = up_bar ;
}

/*! This method sets the values of the Down Bar.
    \param down_bar new position of Down Bar of the Cone
    \return bool indicating whether down_bar was set */
void Cone::set_down_bar(VecPosition down_bar)
{
    m_ddown_bar = down_bar ;
}

/*! This method returns the position of the Ball.
    \return position of the Ball */
VecPosition Cone::get_Ball()
{
    return m_dBall;
}

/*! This method returns the position of the Up Bar.
    \return position of the up bar */
VecPosition Cone::get_up_bar()
{
    return m_dup_bar;
}

/*! This method returns the position of the Down Bar.
    \return position of the down bar */
VecPosition Cone::get_down_bar()
{
    return m_ddown_bar;
}

/*! This method returns a boolean that indicates whether 'pos' is
    located inside the Cone.

   \param pos position of which should be checked whether it is
   located in the Cone

   \return bool indicating whether pos lies inside the Cone */
bool Cone::isInside( VecPosition pos )
{
    AngRad max   = (m_dup_bar - m_dBall).Angle() ;
    AngRad min   = (m_ddown_bar - m_dBall).Angle() ;
    AngRad ang_pos = (pos - m_dBall).Angle() ;

    if(max < min)
    {
        AngRad _temp = max ;
        max = min ;
        min = _temp ;
    }
    return (ang_pos < max) && (ang_pos > min) ;
}

/*!
    This function get covered area by abstacle robot

    \param center vecposition of center of obstacle robot
    \param hole_output a refrence hole_tapy indicate the covered area by abstacle robot

    \return nothing
*/



bool Cone::Is_In_Cone(Circle Obstacle)
{
    double Distance = Obstacle.getCenter().getDistanceTo(m_dBall) ;
    if( Distance <= Obstacle.getRadius() ) return true ;

    AngDeg Arrow = Rad2Deg( (Obstacle.getCenter() - m_dBall).Angle(true) ) ;
    double _temp = Obstacle.getRadius() / Distance ;
    if( _temp <= -1 || _temp >= 1)
    {
        _temp = 0;
        // logStatus("asin will not work probebly",QColor("red"));
    }
    AngDeg Beta = fabs( asinDeg(_temp) ) ;

    AngDeg Min_Arrow = Arrow - Beta ;
    if(Min_Arrow < 0) Min_Arrow += 360 ;
    AngDeg Max_Arrow = Arrow + Beta ;
    if(Max_Arrow >= 360) Max_Arrow -= 360 ;

    //Alfa*=0.8;
    if( Min_Bar < Alfa && Max_Bar > 360 - Alfa )
    {
        if( !( Min_Bar < Alfa / 2.0 && Max_Bar > 360 - (Alfa / 2.0) ) )
            return false;
        if( Max_Arrow > Max_Bar || Min_Arrow > Max_Bar || Max_Arrow < Min_Bar || Min_Arrow < Min_Bar )
            return true ;
        else if( Min_Arrow < 2 * Beta && Max_Arrow > 360 - 2 * Beta )
        {
            if( Max_Bar > Max_Arrow || Min_Bar > Max_Arrow || Max_Bar < Min_Arrow || Min_Bar < Min_Arrow )
                return true ;
            else
                return false ;
        }
        else if( ( Min_Bar > Min_Arrow && Min_Bar < Max_Arrow ) || ( Max_Bar > Min_Arrow && Max_Bar < Max_Arrow ) )
            return true ;
        else
            return false ;
    }
    else if( ( Min_Arrow > Min_Bar && Min_Arrow < Max_Bar ) || ( Max_Arrow > Min_Bar && Max_Arrow < Max_Bar ) )
        return true ;
    else if( Min_Arrow < 2 * Beta && Max_Arrow > 360 - 2 * Beta )
    {
        if( Max_Bar > Max_Arrow || Min_Bar > Max_Arrow || Max_Bar < Min_Arrow || Min_Bar < Min_Arrow )
            return true ;
        else
            return false ;
    }
    else if( ( Min_Bar > Min_Arrow && Min_Bar < Max_Arrow ) || ( Max_Bar > Min_Arrow && Max_Bar < Max_Arrow ) )
        return true ;
    else
        return false ;
}

bool Cone::Get_Cover_Area(Circle Obstacle, Hole_Type &Output)
{
    Hole_Type out;
    Line Goal_Line = Line::makeLineFromTwoPoints(m_dup_bar, m_ddown_bar) ;
    if( (m_dBall.getY() * Goal_Line.getACoefficient() + m_dBall.getX() * Goal_Line.getBCoefficient() + Goal_Line.getCCoefficient()) * (Obstacle.getCenter().getY() * Goal_Line.getACoefficient() + Obstacle.getCenter().getX() * Goal_Line.getBCoefficient() + Goal_Line.getCCoefficient()) > 0 )
    {
        double distance = m_dBall.getDistanceTo(Obstacle.getCenter()) ;
        double _temp = Obstacle.getRadius() / distance ;
        if( _temp <= -1 || _temp >= 1)
        {
            _temp = 0;
            // logStatus("asin will not work probebly",QColor("red"));
        }
        AngRad angle_of_tangent = asin(_temp) ;

        VecPosition up_vec = Obstacle.getCenter() - m_dBall ;
        VecPosition down_vec = up_vec ;

        up_vec.rotate(angle_of_tangent) ;
        down_vec.rotate(-angle_of_tangent) ;

        Line up_line = Line::makeLineFromPositionAndAngle( m_dBall, Rad2Deg(up_vec.Angle()) ) ;
        Line down_line = Line::makeLineFromPositionAndAngle( m_dBall, Rad2Deg(down_vec.Angle()) ) ;

        out.Point_1 = up_line.getIntersection(Goal_Line) ;
        out.Point_2 = down_line.getIntersection(Goal_Line) ;

        VecPosition Nearest = Goal_Line.getPointOnLineClosestTo(m_dBall) ;
        VecPosition Ball_Tangent_to_Goal_Line = ( Nearest - m_dBall ) ;
        if( Ball_Tangent_to_Goal_Line.AngleBetween(up_vec) >= M_PI/2 )
        {
            if( Nearest.getDistanceTo(m_dup_bar) < Nearest.getDistanceTo(m_ddown_bar) )
            {
                out.Point_1 = m_ddown_bar + (m_ddown_bar - m_dup_bar) ;
            }
            else
            {
                out.Point_1 = m_dup_bar + (m_dup_bar - m_ddown_bar) ;
            }
        }
        if( Ball_Tangent_to_Goal_Line.AngleBetween(down_vec) >= M_PI/2 )
        {
            if( Nearest.getDistanceTo(m_dup_bar) < Nearest.getDistanceTo(m_ddown_bar) )
            {
                out.Point_2 = m_ddown_bar + (m_ddown_bar - m_dup_bar) ;
            }
            else
            {
                out.Point_2 = m_dup_bar + (m_dup_bar - m_ddown_bar) ;
            }
        }

        Output = out ;
        return true ;
    }
    VecPosition point = Goal_Line.getPointOnLineClosestTo(m_dBall) ;
    VecPosition newpoint = point + ( point - m_dBall ) / ( point - m_dBall ).getMagnitude() * Obstacle.getRadius() ;
    Line Back_Goal_Line = Line(Goal_Line.getACoefficient(), Goal_Line.getBCoefficient(), -( newpoint.getY() * Goal_Line.getACoefficient() + newpoint.getX() * Goal_Line.getBCoefficient() ) ) ;
    if( (m_dBall.getY() * Back_Goal_Line.getACoefficient() + m_dBall.getX() * Back_Goal_Line.getBCoefficient() + Back_Goal_Line.getCCoefficient()) * ( Obstacle.getCenter().getY() * Back_Goal_Line.getACoefficient() + Obstacle.getCenter().getX() * Back_Goal_Line.getBCoefficient() + Back_Goal_Line.getCCoefficient()) > 0 )
    {
        if( Goal_Line.getCircleIntersectionPoints(Obstacle, &(out.Point_1), &(out.Point_2)) )
        {
            Output = out;
            return true ;
        }
        else
            return false ;
    }
    return false ;
}

double Cone::Get_Free_Space_In_Cone(Circle *Obstacle, int Num_of_Obstacle, Hole_Type &Longest_Hole)
{
    Hole_Type Holes[11] ;
    int Num_of_Hole ;
    int Longest_Hole_Index;

    Hole_Type _temp ;
    _temp.Point_1 = VecPosition(0,0);
    _temp.Point_2 = VecPosition(0,0);

    double out = Get_Free_Space_In_Cone(Obstacle, Num_of_Obstacle, Holes, Num_of_Hole, Longest_Hole_Index) ;
    if( Longest_Hole_Index != -1 )
        Longest_Hole = Holes[Longest_Hole_Index];
    else Longest_Hole = _temp;
    return out;
}

double Cone::Get_Free_Space_In_Cone(Circle *Obstacle, int Num_of_Obstacle)
{
    Hole_Type Holes[11] ;
    int Longest_Hole_Index ;
    int Num_of_Hole ;

    return Get_Free_Space_In_Cone(Obstacle, Num_of_Obstacle, Holes, Num_of_Hole, Longest_Hole_Index) ;
}

double Cone::Get_Free_Space_In_Cone(Circle *Obstacle, int Num_of_Obstacle, Hole_Type *Holes, int &Num_of_Hole)
{
    int Longest_Hole_Index ;

    return Get_Free_Space_In_Cone(Obstacle, Num_of_Obstacle, Holes, Num_of_Hole, Longest_Hole_Index) ;
}



double Cone::Get_Free_Space_In_Cone(int _team_number, int _opp_number, int _non_obs1, int _non_obs2, int _non_obs3, int _non_obs4)
{
    Hole_Type temp;
    return Get_Free_Space_In_Cone(temp, _team_number,  _opp_number,  _non_obs1,  _non_obs2,  _non_obs3,  _non_obs4);
}

double Cone::Get_Free_Space_In_Cone(Hole_Type &longest_hole, int _team_number, int _opp_number, int _non_obs1, int _non_obs2, int _non_obs3, int _non_obs4)
{
    int opp_non_obs1 = -1, opp_non_obs2 = -1, team_non_obs1 = -1, team_non_obs2 = -1 ;

    switch(_team_number)
    {
    case 0:
        switch(_opp_number)
        {
        case 0:
            break;
        case 1:
            opp_non_obs1 = _non_obs1;
            break;
        case 2:
            opp_non_obs1 = _non_obs1;
            opp_non_obs2 = _non_obs2;
            break;
        }
        break;
    case 1:
        switch(_opp_number)
        {
        case 0:
            team_non_obs1 = _non_obs1;
            break;
        case 1:
            team_non_obs1 = _non_obs1;
            opp_non_obs1 = _non_obs2;
            break;
        case 2:
            team_non_obs1 = _non_obs1;
            opp_non_obs1 = _non_obs2;
            opp_non_obs2 = _non_obs3;
            break;
        }
        break;
    case 2:
        switch(_opp_number)
        {
        case 0:
            team_non_obs1 = _non_obs1;
            team_non_obs2 = _non_obs2;
            break;
        case 1:
            team_non_obs1 = _non_obs1;
            team_non_obs2 = _non_obs2;
            opp_non_obs1 = _non_obs3;
            break;
        case 2:
            team_non_obs1 = _non_obs1;
            team_non_obs2 = _non_obs2;
            opp_non_obs1 = _non_obs3;
            opp_non_obs2 = _non_obs4;
            break;
        }
        break;
    }
    Circle Obstacles[24] ;
    int Num_of_Obstacles = 0 , i ;
    for(i = 0; i < world.numT; i++)
        if(i != team_non_obs1 && i != team_non_obs2)
            Obstacles[Num_of_Obstacles++] = Circle(world.robotT[i].position, 90);
    for(i = 0; i < world.numO; i++)
        if(i != opp_non_obs1 && i != opp_non_obs2)
            Obstacles[Num_of_Obstacles++] = Circle(world.robotO[i].position, 90);

    return Get_Free_Space_In_Cone(Obstacles,Num_of_Obstacles,longest_hole);
}


#include "estimation.h"
double Cone::Get_Free_Surface_In_Cone(Circle *Obstacle, int Num_of_Obstacle)
{
    Hole_Type Longest_hole;
    double percent = Get_Free_Space_In_Cone(Obstacle,Num_of_Obstacle,Longest_hole);
    double angle_between = Rad2Deg((Longest_hole.Point_1 - m_dBall).AngleBetween(Longest_hole.Point_2 - m_dBall)) ;

    //GLFrame::addTextToPainting( World::getInstance().ball.getCurrentBallPosition() + VecPosition(0,-200), string::number( angle_between ),QColor("red") );

    double mark[3] = { 0, 25, 100 };
    double range[3] = { 0, 5, 10 };
    return Estimation::getMarkOfMathematicalEquation( range, mark, 3, angle_between);
//    if(angle_between < 15)
//        percent *= ((angle_between ) / (float)(15 ));
//    return percent;
}

double  Cone::Get_Free_Surface_In_Cone(int _team_number, int _opp_number, int _non_obs1, int _non_obs2, int _non_obs3, int _non_obs4)
{
    int opp_non_obs1 = -1, opp_non_obs2 = -1, team_non_obs1 = -1, team_non_obs2 = -1 ;

    switch(_team_number)
    {
    case 0:
        switch(_opp_number)
        {
        case 0:
            break;
        case 1:
            opp_non_obs1 = _non_obs1;
            break;
        case 2:
            opp_non_obs1 = _non_obs1;
            opp_non_obs2 = _non_obs2;
            break;
        }
        break;
    case 1:
        switch(_opp_number)
        {
        case 0:
            team_non_obs1 = _non_obs1;
            break;
        case 1:
            team_non_obs1 = _non_obs1;
            opp_non_obs1 = _non_obs2;
            break;
        case 2:
            team_non_obs1 = _non_obs1;
            opp_non_obs1 = _non_obs2;
            opp_non_obs2 = _non_obs3;
            break;
        }
        break;
    case 2:
        switch(_opp_number)
        {
        case 0:
            team_non_obs1 = _non_obs1;
            team_non_obs2 = _non_obs2;
            break;
        case 1:
            team_non_obs1 = _non_obs1;
            team_non_obs2 = _non_obs2;
            opp_non_obs1 = _non_obs3;
            break;
        case 2:
            team_non_obs1 = _non_obs1;
            team_non_obs2 = _non_obs2;
            opp_non_obs1 = _non_obs3;
            opp_non_obs2 = _non_obs4;
            break;
        }
        break;
    }
    Circle Obstacles[24] ;
    int Num_of_Obstacles = 0 , i ;
    for(i = 0; i < world.numT; i++)
        if(i != team_non_obs1 && i != team_non_obs2)
            Obstacles[Num_of_Obstacles++] = Circle(world.robotT[i].position, 90);
    for(i = 0; i < world.numO; i++)
        if(i != opp_non_obs1 && i != opp_non_obs2)
            Obstacles[Num_of_Obstacles++] = Circle(world.robotO[i].position, 90);
    return Get_Free_Surface_In_Cone( Obstacles, Num_of_Obstacles );
}



double Cone::Get_Free_Space_In_Cone(Circle *Obstacle, int Num_of_Obstacle, Hole_Type *Holes, int &Num_of_Hole, int &Longest_Hole_Index)
{
    int para_counter = 1 ;
    Hole_Type Covered ;
    Hole_Type open_paraline[11] ;
    open_paraline[0].Point_1 = m_dup_bar ;
    open_paraline[0].Point_2 = m_ddown_bar ;

    for(int i = 0; i < Num_of_Obstacle; i++)
    {
        if(! Is_In_Cone(Obstacle[i]))
        {
            continue ;
        }
        if(Get_Cover_Area(Obstacle[i], Covered))
        {
            for(int j = 0; j < para_counter; j++)
            {
                if(para_counter > 1000 || para_counter < 0)
                {
                    int t = 0;
                    t++;
                }

                if( Covered.Point_1.isBetween(open_paraline[j].Point_1, open_paraline[j].Point_2) && Covered.Point_2.isBetween(open_paraline[j].Point_1, open_paraline[j].Point_2) )
                {
                    if( Covered.Point_1.getDistanceTo(open_paraline[j].Point_1) < Covered.Point_2.getDistanceTo(open_paraline[j].Point_1) )
                    {
                        VecPosition temp = open_paraline[j].Point_2 ;
                        open_paraline[j].Point_2 = Covered.Point_1 ;


                        for(int k = para_counter; k > j+1; k--)
                            open_paraline[k] = open_paraline[k-1] ;
                        para_counter++ ;

                        open_paraline[j+1].Point_1 = Covered.Point_2 ;
                        open_paraline[j+1].Point_2 = temp ;

                        j++ ;
                    }
                    else //if(open_paraline[j].Point_1.getDistanceTo(Covered.Point_1) > open_paraline[j].Point_2.getDistanceTo(Covered.Point_1))
                    {
                        VecPosition temp = open_paraline[j].Point_2 ;
                        open_paraline[j].Point_2 = Covered.Point_2 ;

                        for(int k = para_counter; k > j+1; k--)
                            open_paraline[k] = open_paraline[k-1] ;
                        para_counter++ ;

                        open_paraline[j+1].Point_1 = Covered.Point_1 ;
                        open_paraline[j+1].Point_2 = temp ;

                        j++ ;
                    }
                }
                else if( Covered.Point_1.isBetween(open_paraline[j].Point_1, open_paraline[j].Point_2) )
                {
                    bool ss = open_paraline[j].Point_1.isBetween( Covered.Point_1, Covered.Point_2 ) ;
                    if( ss )
                    {
                        open_paraline[j].Point_1 = Covered.Point_1 ;
                    }
                    else if(!ss)
                    {
                        open_paraline[j].Point_2 = Covered.Point_1 ;
                    }
                }
                else if( Covered.Point_2.isBetween(open_paraline[j].Point_1, open_paraline[j].Point_2) )
                {
                    if( open_paraline[j].Point_1.isBetween( Covered.Point_1, Covered.Point_2 ) )
                    {
                        open_paraline[j].Point_1 = Covered.Point_2 ;
                    }
                    else
                    {
                        open_paraline[j].Point_2 = Covered.Point_2 ;
                    }
                }
                else if( ((Covered.Point_1.isBetween(open_paraline[j].Point_1,Covered.Point_2) && Covered.Point_1.isBetween(open_paraline[j].Point_2,Covered.Point_2)))
                      || ((Covered.Point_2.isBetween(open_paraline[j].Point_1,Covered.Point_1) && Covered.Point_2.isBetween(open_paraline[j].Point_2,Covered.Point_1))) )
                    continue;
                else
                {
                    para_counter-- ;
                    for(int k = j; k < para_counter ; k++)
                        open_paraline[k] = open_paraline[k + 1] ;
                    j--;

                }
            }
        }
    }

    double Sum = 0 ;
    Longest_Hole_Index = -1;
    double Longest_Hole_Length = 0 ;
    double Temp_Distance ;
    for(int i = 0; i < para_counter; i++)
    {
        Holes[i] = open_paraline[i] ;
        Temp_Distance = open_paraline[i].Point_1.getDistanceTo(open_paraline[i].Point_2) ;
        Sum += Temp_Distance ;
        if(Temp_Distance > Longest_Hole_Length)
        {
            Longest_Hole_Length = Temp_Distance ;
            Longest_Hole_Index = i ;
        }
    }
    Num_of_Hole = para_counter ;
    return (Sum / m_dup_bar.getDistanceTo(m_ddown_bar) ) * 100 ;
}

/***********************************************************************/
/********************* CLASS PARALINE **********************************/
/***********************************************************************/

/*! This is the constructor of a Paraline.
    \param _first first VecPosition that defines Position Of first point
    \param _second second VecPosition that defines Position Of second point
    \return Paraline with first and second point and length */
Paraline::Paraline()
{
    setFirstPoint(VecPosition(0, 0)) ;
    setSecondPoint(VecPosition(0, 0)) ;
    m_dlength = 0 ;
	m_a = 0;
	m_b = 0;
	m_c = 0;
}

/*! This is the constructor of a Paraline.
    \param _first first VecPosition that defines Position Of first point
    \param _second second VecPosition that defines Position Of second point
    \return Paraline with first and second point and length */
Paraline::Paraline(VecPosition _first, VecPosition _second)
{

    if(_first.getY() > _second.getY())
    {
        setFirstPoint(_first) ;
        setSecondPoint(_second) ;
    }
    else
    {
        setFirstPoint(_second) ;
        setSecondPoint(_first) ;
    }

    m_dlength = _first.getDistanceTo(_second) ;

	/*this part is to get "a" , "b" and "c" coefficents */

	// 1*y + bx + c = 0 => y = -bx - c
	// with -b the direction coefficient (or slope)
	// and c = - y - bx
	double dA, dB, dC;
	double dTemp = _second.getX() - _first.getX(); // determine the slope
	if (fabs(dTemp) < EPSILON)
	{
		// ay + bx + c = 0 with vertical slope=> a = 0, b = 1
		dA = 0.0;
		dB = 1.0;
	}
	else
	{
		// y = (-b)x -c with -b the slope of the line
		dA = 1.0;
		dB = -(_second.getY() - _first.getY()) / dTemp;
	}
	// ay + bx + c = 0 ==> c = -a*y - b*x
	dC = -dA*_second.getY() - dB * _second.getX();

	m_a = dA;
	m_b = dB;
	m_c = dC;
	
}

/*! This method prints the Paraline information to the specified output stream
    in the following format:
    "fp: first_position" the position of the first point.
    "sp: second_position" the position of the second point.
    "l: length" the length of the paraline.
    \param os output stream to which output is written. */
void Paraline::show( ostream& os)
{
    os << "fp:"  << m_dfirst << endl ;
    os << "sp:" << m_dsecond << endl ;
    os << "l:" << m_dlength ;
}


/*! This method sets the values of the Paraline.
    \param _first new position of first point of the Paraline
    \return bool indicating whether first was set */
void Paraline::setFirstPoint(VecPosition _first)
{
    m_dfirst = _first ;
   /* if( m_dfirst.getY() < m_dsecond.getY() )
    {
        VecPosition _temp = m_dfirst ;
        m_dfirst = m_dsecond ;
        m_dsecond = _temp ;
    }*/
}

/*! This method sets the values of the Paraline.
    \param _second new position of second point of the Paraline
    \return bool indicating whether second was set */
void Paraline::setSecondPoint(VecPosition _second)
{
    m_dsecond = _second ;
    /*if( m_dfirst.getY() < m_dsecond.getY() )
    {
        VecPosition _temp = m_dfirst ;
        m_dfirst = m_dsecond ;
        m_dsecond = _temp ;
    }*/
}

/*! This method returns the position of the first point.
    \return position of the first point */
VecPosition Paraline::getFirstPoint()
{
    if( m_dfirst.getY() < m_dsecond.getY() )
    {
        VecPosition _temp = m_dfirst ;
        m_dfirst = m_dsecond ;
        m_dsecond = _temp ;
    }
    return m_dfirst ;
}

/*! This method returns the position of the second point.
    \return position of the second point */
VecPosition Paraline::getSecondPoint()
{
    if( m_dfirst.getY() < m_dsecond.getY() )
    {
        VecPosition _temp = m_dfirst ;
        m_dfirst = m_dsecond ;
        m_dsecond = _temp ;
    }
    return m_dsecond ;
}

VecPosition Paraline::getMidPoint()
{
    return ( ( m_dfirst + m_dsecond ) / 2 ) ;
}

/*! This method returns the values of length.
    \return values of length */
double Paraline::getLength()
{
    return m_dlength ;
}

/*! This method return the distance between the current
    Paraline and a given VecPosition.

    \param _point a Vecposition
    \return the distance between the current Paraline and the given
    VecPosition */
double Paraline::getDistanceTo(VecPosition _point)
{
    Line my_line = Line::makeLineFromTwoPoints(m_dfirst, m_dsecond) ;
    VecPosition closest = my_line.getPointOnLineClosestTo(_point) ;
    if(closest.getDistanceTo(m_dfirst) < m_dlength && closest.getDistanceTo(m_dsecond) < m_dlength)
        return  closest.getDistanceTo(_point) ;
    else
        return min(_point.getDistanceTo(m_dfirst), _point.getDistanceTo(m_dsecond)) ;
}

/*! This method returns the a coefficient from the Paraline ay + bx + c = 0.
\return a coefficient of the line. */
double Paraline::getACoefficient() const
{
	return m_a;
}

/*! This method returns the b coefficient from the Paraline ay + bx + c = 0.
\return b coefficient of the line. */
double Paraline::getBCoefficient() const
{
	return m_b;
}

/*! This method returns the c coefficient from the Paraline ay + bx + c = 0.
\return c coefficient of the line. */
double Paraline::getCCoefficient() const
{
	return m_c;
}

/*! This method calculates the y coordinate given the x coordinate
\param x coordinate
\return y coordinate on this line */
double Paraline::getYGivenX(double x)
{
	if (m_a == 0)
	{
		//logStatus("bad input in getYGivenX" , QColor("red") );
		cerr << "(Line::getYGivenX) Cannot calculate Y coordinate: ";
		show(cerr);
		cerr << endl;
		return 0;
	}
	// ay + bx + c = 0 ==> ay = -(b*x + c)/a
	return -(m_b * x + m_c) / m_a;
}

/*! This method calculates the x coordinate given the x coordinate
\param y coordinate
\return x coordinate on this line */
double Paraline::getXGivenY(double y)
{
	if (m_b == 0)
	{
		cerr << "(Line::getXGivenY) Cannot calculate X coordinate\n";
		return 0;
	}
	// ay + bx + c = 0 ==> bx = -(a*y + c)/a
	return -(m_a * y + m_c) / m_b;
}

/*! This method return the Nearest Point (First point or Second point)
    to a given VecPosition .

    \param _point a Vecposition
    \return the VecPosition indicate Nearest Point to a given VecPosition.
*/
VecPosition Paraline::getNearerPoint(VecPosition _point)
{
    if(m_dfirst.getDistanceTo(_point) < m_dsecond.getDistanceTo(_point))
        return m_dfirst ;
    else
        return m_dsecond ;
}

/*! This method return the maximum length paraline .
    \param _para_array[] a array of input Paraline .
    \param _int a int indicate length of input _para_array .
    \return the Paraline indicate maximum length paraline.
*/
Paraline Paraline::getMaxParaline(Paraline* _para_array, int _num)
{
    int output_index = 0 ;
    for(int i = 0; i < _num; i++)
        if(_para_array[i].getLength() > _para_array[output_index].getLength())
            output_index = i ;

    return _para_array[output_index] ;
}

/*! This method return the minimum length paraline .
    \param _para_array[] a array of input Paraline .
    \param _int a int indicate length of input _para_array .
    \return the Paraline indicate minimum length paraline.
*/
Paraline Paraline::getMinParaline(Paraline* _para_array, int _num)
{
    int output_index = 0 ;
    for(int i = 0; i < _num; i++)
        if(_para_array[i].getLength() < _para_array[output_index].getLength())
            output_index = i ;

    return _para_array[output_index] ;
}

/*! This method returns the closest point on a paraline to a given position.
    \param pos point to which closest point should be determined
    \return VecPosition closest point on paraline to 'pos'. */

VecPosition Paraline::getPointOnParalineClosestTo(VecPosition _point)
{
    Line my_line = Line::makeLineFromTwoPoints(m_dfirst, m_dsecond) ;
    VecPosition closest = my_line.getPointOnLineClosestTo(_point) ;
    if(closest.getDistanceTo(m_dfirst) < m_dfirst.getDistanceTo(m_dsecond) && closest.getDistanceTo(m_dsecond) < m_dfirst.getDistanceTo(m_dsecond))
        return closest ;
    else
        return this->getNearerPoint(_point) ;
}

/*! This method determines whether a given VecPosition is in
    between two point of current Paraline (first point and second point).

    \param _point a VecPosition to which the current Paraline must be
    compared
    \param _limit a double value which has indecated the maximum
    distance to current paraline .
   \return bool indicating whether _point lies inside the Paraline */
bool Paraline::hasIntersection(Circle _object, double _limit)
{
	double a = this->getDistanceTo(_object.getCenter());
    if(this->getDistanceTo(_object.getCenter()) < _object.getRadius() + _limit)
        return true ;
    else
        return false ;
}

bool Paraline::hasIntersection(Paraline paraline)
{
	VecPosition pos;
	double x, y;
	if ((m_a / m_b) == (paraline.getACoefficient() / paraline.getBCoefficient()))
		return false; // lines are parallel, no intersection
	if (m_a == 0)            // bx + c = 0 and a2*y + b2*x + c2 = 0 ==> x = -c/b
	{                          // calculate x using the current line
		x = -m_c / m_b;                // and calculate the y using the second line
		y = paraline.getYGivenX(x);
	}
	else if (paraline.getACoefficient() == 0)
	{                         // ay + bx + c = 0 and b2*x + c2 = 0 ==> x = -c2/b2
		x = -paraline.getCCoefficient() / paraline.getBCoefficient(); // calculate x using
		y = getYGivenX(x);       // 2nd line and calculate y using current line
	}
	// ay + bx + c = 0 and a2y + b2*x + c2 = 0
	// y = (-b2/a2)x - c2/a2
	// bx = -a*y - c =>  bx = -a*(-b2/a2)x -a*(-c2/a2) - c ==>
	// ==> a2*bx = a*b2*x + a*c2 - a2*c ==> x = (a*c2 - a2*c)/(a2*b - a*b2)
	// calculate x using the above formula and the y using the current line
	else
	{
		x = (m_a*paraline.getCCoefficient() - paraline.getACoefficient()*m_c) /
			(paraline.getACoefficient()*m_b - m_a*paraline.getBCoefficient());
		y = getYGivenX(x);
	}
	VecPosition answer(x, y);
	if ((max(answer.getDistanceTo(getFirstPoint()), answer.getDistanceTo(getSecondPoint())) <= getLength()) &&
		(max(answer.getDistanceTo(paraline.getFirstPoint()), answer.getDistanceTo(paraline.getSecondPoint())) <= paraline.getLength()))
		return true;
	else
		return false;
}

bool Paraline::isPositionInRight(VecPosition point)
{
	if (min(getFirstPoint().getX(), getSecondPoint().getX() < point.getX()))
		return true;
	else
		return false;
}

bool Paraline::isPositionInLeft(VecPosition point)
{
	if (max(getFirstPoint().getX(), getSecondPoint().getX() > point.getX()))
		return true;
	else
		return false;
}

/*****************************************************************************/
/********************* CLASS Regression **************************************/
/*****************************************************************************/

//Regression::Regression(VecPosition* _pos, int _num)
//{
//    _num = _num ;
//    for(int i = 0; i < _num; i++)
//        _pos[i] = _pos[i] ;
//}

Line Regression::getLinearRegression(VecPosition* _pos, int _num)
{
    //Regression Algorithm
    double sigmaX  = 0 ;
    double sigmaY  = 0 ;
    double sigmaXY = 0 ;
    double sigmaXX = 0 ;

    for(int i = 0; i < _num; i++)
    {
        sigmaX += _pos[i].getX() ;
        sigmaY += _pos[i].getY() ;
        sigmaXY += _pos[i].getX() * _pos[i].getY() ;
        sigmaXX += _pos[i].getX() * _pos[i].getX() ;
    }

    double b = ( ( sigmaXY ) - ( sigmaX * sigmaY ) / _num) / (sigmaXX - sigmaX * sigmaX / _num) ;
    double a = (sigmaY / _num) - b * (sigmaX / _num) ;

    return Line(1.0, -b, -a) ;
}

/*****************************************************************************/
/********************* CLASS RECTANGLE ***************************************/
/*****************************************************************************/

/*! This is the constructor of a Rectangle. Two points will be given. The
    order does not matter as long as two opposite points are given (left
    top and right bottom or right top and left bottom).
    \param pos first point that defines corner of rectangle
    \param pos2 second point that defines other corner of rectangle
    \return rectangle with 'pos' and 'pos2' as opposite corners. */
Rect::Rect( VecPosition pos, VecPosition pos2 )
{
    setRectanglePoints( pos, pos2 );
}

/*! This method sets the upper left and right bottom point of the current
    rectangle.
    \param pos first point that defines corner of rectangle
    \param pos2 second point that defines other corner of rectangle */
void Rect::setRectanglePoints( VecPosition pos1, VecPosition pos2 )
{
    m_posLeftTop.setX    ( min( pos1.getX(), pos2.getX() ) );
    m_posLeftTop.setY    ( max( pos1.getY(), pos2.getY() ) );
    m_posRightBottom.setX( max( pos1.getX(), pos2.getX() ) );
    m_posRightBottom.setY( min( pos1.getY(), pos2.getY() ) );
}

/*! This method prints the rectangle to the specified output stream in the
    format rect( top_left_point, bottom_right_point ).
    \param os output stream to which rectangle is written. */
void Rect::show( ostream& os )
{
    os << "rect(" << m_posLeftTop << " " << m_posRightBottom << ")";
}

/*! This method determines whether the given position lies inside the current
    rectangle.
    \param pos position which is checked whether it lies in rectangle
    \return true when 'pos' lies in the rectangle, false otherwise */
bool Rect::isInside( const VecPosition &pos ) const
{
    //  return pos.isBetweenX( m_posRightBottom.getX(), m_posLeftTop.getX() ) &&
    //         pos.isBetweenY( m_posLeftTop.getY(),     m_posRightBottom.getY() );
    return ( ( pos.getX() <= m_posRightBottom.getX() ) && ( pos.getX() >= m_posLeftTop.getX() ) ) && ( ( pos.getY() >= m_posRightBottom.getY() ) && ( pos.getY() <= m_posLeftTop.getY() ) );


}

/*! This method sets the top left position of the rectangle
    \param pos new top left position of the rectangle
    \return true when update was successful */
bool Rect::setPosLeftTop( VecPosition pos )
{
    m_posLeftTop = pos;
    return true;
}

/*! This method returns the top left position of the rectangle
    \return top left position of the rectangle */
VecPosition Rect::getPosLeftTop(  )
{
    return m_posLeftTop;
}

/*! This method sets the right bottom position of the rectangle
    \param pos new right bottom position of the rectangle
    \return true when update was succesfull */
bool Rect::setPosRightBottom( VecPosition pos )
{
    m_posRightBottom = pos;
    return true;
}

/*! This method returns the right bottom position of the rectangle
    \return top right bottom of the rectangle */
VecPosition Rect::getPosRightBottom(  )
{
    return m_posRightBottom;
}

bool Rect::hasIntersectionWith(VecPosition start, VecPosition end)
{
	Paraline StoE (start, end);
	Paraline penaltyAreaLine (Field::getUpLeft_LeftPenaltyArea(), Field::getUpRight_LeftPenaltyArea());
	if (Paraline(Field::getUpLeft_LeftPenaltyArea(), Field::getUpRight_LeftPenaltyArea()).hasIntersection(StoE) ||
		Paraline(Field::getUpRight_LeftPenaltyArea(), Field::getDownRight_LeftPenaltyArea()).hasIntersection(StoE) ||
		Paraline(Field::getDownLeft_LeftPenaltyArea(), Field::getDownRight_LeftPenaltyArea()).hasIntersection(StoE) ||

		Paraline(Field::getUpLeft_RightPenaltyArea(), Field::getUpRight_RightPenaltyArea()).hasIntersection(StoE) ||
		Paraline(Field::getUpLeft_RightPenaltyArea(), Field::getDownLeft_RightPenaltyArea()).hasIntersection(StoE) ||
		Paraline(Field::getDownLeft_RightPenaltyArea(), Field::getDownRight_RightPenaltyArea()).hasIntersection(StoE))
		return true;
	else
		return false;


}

/*! This method determines the distance between the current
    VecPosition and a given VecPosition. This is equal to the
    magnitude (length) of the vector connecting the two positions
    which is the difference vector between them.

    \param p a Vecposition
    \return the distance between the current VecPosition and the given
    VecPosition */
//double Rect::getDistanceTo()


/*****************************************************************************/
/********************* CLASS RECTANGLE ***************************************/
/*****************************************************************************/

Diagram::Diagram()
{
    resetDiagram();
}

void Diagram::resetDiagram()
{
    numOfSavedStack = 0;
}

void Diagram::setNextItem(double nextStack1, double nextStack2, double nextStack3)
{
    for ( int i = maxNumOfStack - 1; i > 0; i-- )
    {
        stack1[ i ] = stack1[ i - 1 ];
        stack2[ i ] = stack2[ i - 1 ];
        stack3[ i ] = stack3[ i - 1 ];
    }

    stack1[0] = nextStack1;
    stack2[0] = nextStack2;
    stack3[0] = nextStack3;

    if ( numOfSavedStack < maxNumOfStack ) numOfSavedStack++;
}

/*****************************************************************************/
/********************** TESTING PURPOSES *************************************/
/*****************************************************************************/

/*
#include<iostream.h>

int main( void )
{
  double dFirst = 1.0;
  double dRatio = 2.5;
  double dSum   = 63.4375;
  double dLength = 4.0;

  printf( "sum: %f\n", Geometry::getSumGeomSeries( dFirst, dRatio, dLength));
  printf( "length: %f\n", Geometry::getLengthGeomSeries( dFirst, dRatio, dSum));
}

int main( void )
{
  Line l1(1,-1,3 );
  Line l2(1,-0.2,10 );
 Line l3 = Line::makeLineFromTwoPoints( VecPosition(1,-1), VecPosition(2,-2) );
 l3.show();
 cout << endl;
 l1.show();
 l2.show();
  l1.getIntersection( l2 ).show();
}


int main( void )
{
  Line l( 1, -1, 0 );
  VecPosition s1, s2;
  int i = l.getCircleIntersectionPoints( Circle( VecPosition(1,1),1) &s1,&s2 );
  printf( "number of solutions: %d\n", i );
  if( i == 2 )
  {
    cout << s1 << " " << s2 ;
  }
  else if( i == 1 )
  {
    cout << s1;
  }
  cout << "line: " << l;
}

int main( void )
{
  Circle c11( VecPosition( 10, 0 ), 10);
  Circle c12( VecPosition( 40, 3 ), 40 );
  Circle c21( VecPosition(  0,0 ), 5);
  Circle c22( VecPosition(  3,0 ), 40 );

  VecPosition p1, p2;

  cout << c11.getIntersectionArea( c21 ) << endl;
  cout << c12.getIntersectionArea( c21 ) << endl;
  cout << c22.getIntersectionArea( c11 ) << endl;
  cout << c12.getIntersectionArea( c22 ) << endl;
  return 0;
}

int main( void )
{
  cout << getBisectorTwoAngles( -155.3, 179.0 ) << endl;
  cout << getBisectorTwoAngles( -179.3, 179.0 ) << endl;
}
*/
