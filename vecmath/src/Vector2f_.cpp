#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "Vector2f_.h"
#include "Vector3f_.h"

//////////////////////////////////////////////////////////////////////////
// Public
//////////////////////////////////////////////////////////////////////////

// static
const Vector2f_ Vector2f_::ZERO = Vector2f_( 0, 0 );

// static
const Vector2f_ Vector2f_::UP = Vector2f_( 0, 1 );

// static
const Vector2f_ Vector2f_::RIGHT = Vector2f_( 1, 0 );

Vector2f_::Vector2f_( float f )
{
    m_elements[0] = f;
    m_elements[1] = f;
}

Vector2f_::Vector2f_( float x, float y )
{
    m_elements[0] = x;
    m_elements[1] = y;
}

Vector2f_::Vector2f_( const Vector2f_& rv )
{
    m_elements[0] = rv[0];
    m_elements[1] = rv[1];
}

Vector2f_& Vector2f_::operator = ( const Vector2f_& rv )
{
 	if( this != &rv )
	{
        m_elements[0] = rv[0];
        m_elements[1] = rv[1];
    }
    return *this;
}

const float& Vector2f_::operator [] ( int i ) const
{
    return m_elements[i];
}

float& Vector2f_::operator [] ( int i )
{
    return m_elements[i];
}

float& Vector2f_::x()
{
    return m_elements[0];
}

float& Vector2f_::y()
{
    return m_elements[1];
}

float Vector2f_::x() const
{
    return m_elements[0];
}	

float Vector2f_::y() const
{
    return m_elements[1];
}

Vector2f_ Vector2f_::xy() const
{
    return *this;
}

Vector2f_ Vector2f_::yx() const
{
    return Vector2f_( m_elements[1], m_elements[0] );
}

Vector2f_ Vector2f_::xx() const
{
    return Vector2f_( m_elements[0], m_elements[0] );
}

Vector2f_ Vector2f_::yy() const
{
    return Vector2f_( m_elements[1], m_elements[1] );
}

Vector2f_ Vector2f_::normal() const
{
    return Vector2f_( -m_elements[1], m_elements[0] );
}

float Vector2f_::abs() const
{
    return sqrt(absSquared());
}

float Vector2f_::absSquared() const
{
    return m_elements[0] * m_elements[0] + m_elements[1] * m_elements[1];
}

void Vector2f_::normalize()
{
    float norm = abs();
    m_elements[0] /= norm;
    m_elements[1] /= norm;
}

Vector2f_ Vector2f_::normalized() const
{
    float norm = abs();
    return Vector2f_( m_elements[0] / norm, m_elements[1] / norm );
}

void Vector2f_::negate()
{
    m_elements[0] = -m_elements[0];
    m_elements[1] = -m_elements[1];
}

Vector2f_::operator const float* () const
{
    return m_elements;
}

Vector2f_::operator float* ()
{
    return m_elements;
}

void Vector2f_::print() const
{
	printf( "< %.4f, %.4f >\n",
		m_elements[0], m_elements[1] );
}

Vector2f_& Vector2f_::operator += ( const Vector2f_& v )
{
	m_elements[ 0 ] += v.m_elements[ 0 ];
	m_elements[ 1 ] += v.m_elements[ 1 ];
	return *this;
}

Vector2f_& Vector2f_::operator -= ( const Vector2f_& v )
{
	m_elements[ 0 ] -= v.m_elements[ 0 ];
	m_elements[ 1 ] -= v.m_elements[ 1 ];
	return *this;
}

Vector2f_& Vector2f_::operator *= ( float f )
{
	m_elements[ 0 ] *= f;
	m_elements[ 1 ] *= f;
	return *this;
}

// static
float Vector2f_::dot( const Vector2f_& v0, const Vector2f_& v1 )
{
    return v0[0] * v1[0] + v0[1] * v1[1];
}

// static
Vector3f_ Vector2f_::cross( const Vector2f_& v0, const Vector2f_& v1 )
{
	return Vector3f_
		(
			0,
			0,
			v0.x() * v1.y() - v0.y() * v1.x()
		);
}

// static
Vector2f_ Vector2f_::lerp( const Vector2f_& v0, const Vector2f_& v1, float alpha )
{
	return alpha * ( v1 - v0 ) + v0;
}

//////////////////////////////////////////////////////////////////////////
// Operator overloading
//////////////////////////////////////////////////////////////////////////

Vector2f_ operator + ( const Vector2f_& v0, const Vector2f_& v1 )
{
    return Vector2f_( v0.x() + v1.x(), v0.y() + v1.y() );
}

Vector2f_ operator - ( const Vector2f_& v0, const Vector2f_& v1 )
{
    return Vector2f_( v0.x() - v1.x(), v0.y() - v1.y() );
}

Vector2f_ operator * ( const Vector2f_& v0, const Vector2f_& v1 )
{
    return Vector2f_( v0.x() * v1.x(), v0.y() * v1.y() );
}

Vector2f_ operator / ( const Vector2f_& v0, const Vector2f_& v1 )
{
    return Vector2f_( v0.x() * v1.x(), v0.y() * v1.y() );
}

Vector2f_ operator - ( const Vector2f_& v )
{
    return Vector2f_( -v.x(), -v.y() );
}

Vector2f_ operator * ( float f, const Vector2f_& v )
{
    return Vector2f_( f * v.x(), f * v.y() );
}

Vector2f_ operator * ( const Vector2f_& v, float f )
{
    return Vector2f_( f * v.x(), f * v.y() );
}

Vector2f_ operator / ( const Vector2f_& v, float f )
{
    return Vector2f_( v.x() / f, v.y() / f );
}

bool operator == ( const Vector2f_& v0, const Vector2f_& v1 )
{
    return( v0.x() == v1.x() && v0.y() == v1.y() );
}

bool operator != ( const Vector2f_& v0, const Vector2f_& v1 )
{
    return !( v0 == v1 );
}
