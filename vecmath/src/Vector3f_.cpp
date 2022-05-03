#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "Vector3f_.h"
#include "Vector2f_.h"

//////////////////////////////////////////////////////////////////////////
// Public
//////////////////////////////////////////////////////////////////////////

// static
const Vector3f_ Vector3f_::ZERO = Vector3f_( 0, 0, 0 );

// static
const Vector3f_ Vector3f_::UP = Vector3f_( 0, 1, 0 );

// static
const Vector3f_ Vector3f_::RIGHT = Vector3f_( 1, 0, 0 );

// static
const Vector3f_ Vector3f_::FORWARD = Vector3f_( 0, 0, -1 );

Vector3f_::Vector3f_( float f )
{
    m_elements[0] = f;
    m_elements[1] = f;
    m_elements[2] = f;
}

Vector3f_::Vector3f_( float x, float y, float z )
{
    m_elements[0] = x;
    m_elements[1] = y;
    m_elements[2] = z;
}

Vector3f_::Vector3f_( const Vector2f_& xy, float z )
{
	m_elements[0] = xy.x();
	m_elements[1] = xy.y();
	m_elements[2] = z;
}

Vector3f_::Vector3f_( float x, const Vector2f_& yz )
{
	m_elements[0] = x;
	m_elements[1] = yz.x();
	m_elements[2] = yz.y();
}

Vector3f_::Vector3f_( const Vector3f_& rv )
{
    m_elements[0] = rv[0];
    m_elements[1] = rv[1];
    m_elements[2] = rv[2];
}

Vector3f_& Vector3f_::operator = ( const Vector3f_& rv )
{
    if( this != &rv )
    {
        m_elements[0] = rv[0];
        m_elements[1] = rv[1];
        m_elements[2] = rv[2];
    }
    return *this;
}

const float& Vector3f_::operator [] ( int i ) const
{
    return m_elements[i];
}

float& Vector3f_::operator [] ( int i )
{
    return m_elements[i];
}

float& Vector3f_::x()
{
    return m_elements[0];
}

float& Vector3f_::y()
{
    return m_elements[1];
}

float& Vector3f_::z()
{
    return m_elements[2];
}

float Vector3f_::x() const
{
    return m_elements[0];
}

float Vector3f_::y() const
{
    return m_elements[1];
}

float Vector3f_::z() const
{
    return m_elements[2];
}

Vector2f_ Vector3f_::xy() const
{
	return Vector2f_( m_elements[0], m_elements[1] );
}

Vector2f_ Vector3f_::xz() const
{
	return Vector2f_( m_elements[0], m_elements[2] );
}

Vector2f_ Vector3f_::yz() const
{
	return Vector2f_( m_elements[1], m_elements[2] );
}

Vector3f_ Vector3f_::xyz() const
{
	return Vector3f_( m_elements[0], m_elements[1], m_elements[2] );
}

Vector3f_ Vector3f_::yzx() const
{
	return Vector3f_( m_elements[1], m_elements[2], m_elements[0] );
}

Vector3f_ Vector3f_::zxy() const
{
	return Vector3f_( m_elements[2], m_elements[0], m_elements[1] );
}

float Vector3f_::abs() const
{
	return sqrt( m_elements[0] * m_elements[0] + m_elements[1] * m_elements[1] + m_elements[2] * m_elements[2] );
}

float Vector3f_::absSquared() const
{
    return
        (
            m_elements[0] * m_elements[0] +
            m_elements[1] * m_elements[1] +
            m_elements[2] * m_elements[2]
        );
}

void Vector3f_::normalize()
{
	float norm = abs();
	m_elements[0] /= norm;
	m_elements[1] /= norm;
	m_elements[2] /= norm;
}

Vector3f_ Vector3f_::normalized() const
{
	float norm = abs();
	return Vector3f_
		(
			m_elements[0] / norm,
			m_elements[1] / norm,
			m_elements[2] / norm
		);
}

Vector2f_ Vector3f_::homogenized() const
{
	return Vector2f_
		(
			m_elements[ 0 ] / m_elements[ 2 ],
			m_elements[ 1 ] / m_elements[ 2 ]
		);
}

void Vector3f_::negate()
{
	m_elements[0] = -m_elements[0];
	m_elements[1] = -m_elements[1];
	m_elements[2] = -m_elements[2];
}

Vector3f_::operator const float* () const
{
    return m_elements;
}

Vector3f_::operator float* ()
{
    return m_elements;
}

void Vector3f_::print() const
{
	printf( "< %.4f, %.4f, %.4f >\n",
		m_elements[0], m_elements[1], m_elements[2] );
}

Vector3f_& Vector3f_::operator += ( const Vector3f_& v )
{
	m_elements[ 0 ] += v.m_elements[ 0 ];
	m_elements[ 1 ] += v.m_elements[ 1 ];
	m_elements[ 2 ] += v.m_elements[ 2 ];
	return *this;
}

Vector3f_& Vector3f_::operator -= ( const Vector3f_& v )
{
	m_elements[ 0 ] -= v.m_elements[ 0 ];
	m_elements[ 1 ] -= v.m_elements[ 1 ];
	m_elements[ 2 ] -= v.m_elements[ 2 ];
	return *this;
}

Vector3f_& Vector3f_::operator *= ( float f )
{
	m_elements[ 0 ] *= f;
	m_elements[ 1 ] *= f;
	m_elements[ 2 ] *= f;
	return *this;
}

// static
float Vector3f_::dot( const Vector3f_& v0, const Vector3f_& v1 )
{
    return v0[0] * v1[0] + v0[1] * v1[1] + v0[2] * v1[2];
}

// static
Vector3f_ Vector3f_::cross( const Vector3f_& v0, const Vector3f_& v1 )
{
    return Vector3f_
        (
            v0.y() * v1.z() - v0.z() * v1.y(),
            v0.z() * v1.x() - v0.x() * v1.z(),
            v0.x() * v1.y() - v0.y() * v1.x()
        );
}

// static
Vector3f_ Vector3f_::lerp( const Vector3f_& v0, const Vector3f_& v1, float alpha )
{
	return alpha * ( v1 - v0 ) + v0;
}

// static
Vector3f_ Vector3f_::cubicInterpolate( const Vector3f_& p0, const Vector3f_& p1, const Vector3f_& p2, const Vector3f_& p3, float t )
{
	// geometric construction:
	//            t
	//   (t+1)/2     t/2
	// t+1        t	        t-1

	// bottom level
	Vector3f_ p0p1 = Vector3f_::lerp( p0, p1, t + 1 );
	Vector3f_ p1p2 = Vector3f_::lerp( p1, p2, t );
	Vector3f_ p2p3 = Vector3f_::lerp( p2, p3, t - 1 );

	// middle level
	Vector3f_ p0p1_p1p2 = Vector3f_::lerp( p0p1, p1p2, 0.5f * ( t + 1 ) );
	Vector3f_ p1p2_p2p3 = Vector3f_::lerp( p1p2, p2p3, 0.5f * t );

	// top level
	return Vector3f_::lerp( p0p1_p1p2, p1p2_p2p3, t );
}

Vector3f_ operator + ( const Vector3f_& v0, const Vector3f_& v1 )
{
    return Vector3f_( v0[0] + v1[0], v0[1] + v1[1], v0[2] + v1[2] );
}

Vector3f_ operator - ( const Vector3f_& v0, const Vector3f_& v1 )
{
    return Vector3f_( v0[0] - v1[0], v0[1] - v1[1], v0[2] - v1[2] );
}

Vector3f_ operator * ( const Vector3f_& v0, const Vector3f_& v1 )
{
    return Vector3f_( v0[0] * v1[0], v0[1] * v1[1], v0[2] * v1[2] );
}

Vector3f_ operator / ( const Vector3f_& v0, const Vector3f_& v1 )
{
    return Vector3f_( v0[0] / v1[0], v0[1] / v1[1], v0[2] / v1[2] );
}

Vector3f_ operator - ( const Vector3f_& v )
{
    return Vector3f_( -v[0], -v[1], -v[2] );
}

Vector3f_ operator * ( float f, const Vector3f_& v )
{
    return Vector3f_( v[0] * f, v[1] * f, v[2] * f );
}

Vector3f_ operator * ( const Vector3f_& v, float f )
{
    return Vector3f_( v[0] * f, v[1] * f, v[2] * f );
}

Vector3f_ operator / ( const Vector3f_& v, float f )
{
    return Vector3f_( v[0] / f, v[1] / f, v[2] / f );
}

bool operator == ( const Vector3f_& v0, const Vector3f_& v1 )
{
    return( v0.x() == v1.x() && v0.y() == v1.y() && v0.z() == v1.z() );
}

bool operator != ( const Vector3f_& v0, const Vector3f_& v1 )
{
    return !( v0 == v1 );
}
