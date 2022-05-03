#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "Vector4f_.h"
#include "Vector2f_.h"
#include "Vector3f_.h"

Vector4f_::Vector4f_( float f )
{
	m_elements[ 0 ] = f;
	m_elements[ 1 ] = f;
	m_elements[ 2 ] = f;
	m_elements[ 3 ] = f;
}

Vector4f_::Vector4f_( float fx, float fy, float fz, float fw )
{
	m_elements[0] = fx;
	m_elements[1] = fy;
	m_elements[2] = fz;
	m_elements[3] = fw;
}

Vector4f_::Vector4f_( float buffer[ 4 ] )
{
	m_elements[ 0 ] = buffer[ 0 ];
	m_elements[ 1 ] = buffer[ 1 ];
	m_elements[ 2 ] = buffer[ 2 ];
	m_elements[ 3 ] = buffer[ 3 ];
}

Vector4f_::Vector4f_( const Vector2f_& xy, float z, float w )
{
	m_elements[0] = xy.x();
	m_elements[1] = xy.y();
	m_elements[2] = z;
	m_elements[3] = w;
}

Vector4f_::Vector4f_( float x, const Vector2f_& yz, float w )
{
	m_elements[0] = x;
	m_elements[1] = yz.x();
	m_elements[2] = yz.y();
	m_elements[3] = w;
}

Vector4f_::Vector4f_( float x, float y, const Vector2f_& zw )
{
	m_elements[0] = x;
	m_elements[1] = y;
	m_elements[2] = zw.x();
	m_elements[3] = zw.y();
}

Vector4f_::Vector4f_( const Vector2f_& xy, const Vector2f_& zw )
{
	m_elements[0] = xy.x();
	m_elements[1] = xy.y();
	m_elements[2] = zw.x();
	m_elements[3] = zw.y();
}

Vector4f_::Vector4f_( const Vector3f_& xyz, float w )
{
	m_elements[0] = xyz.x();
	m_elements[1] = xyz.y();
	m_elements[2] = xyz.z();
	m_elements[3] = w;
}

Vector4f_::Vector4f_( float x, const Vector3f_& yzw )
{
	m_elements[0] = x;
	m_elements[1] = yzw.x();
	m_elements[2] = yzw.y();
	m_elements[3] = yzw.z();
}

Vector4f_::Vector4f_( const Vector4f_& rv )
{
	m_elements[0] = rv.m_elements[0];
	m_elements[1] = rv.m_elements[1];
	m_elements[2] = rv.m_elements[2];
	m_elements[3] = rv.m_elements[3];
}

Vector4f_& Vector4f_::operator = ( const Vector4f_& rv )
{
	if( this != &rv )
	{
		m_elements[0] = rv.m_elements[0];
		m_elements[1] = rv.m_elements[1];
		m_elements[2] = rv.m_elements[2];
		m_elements[3] = rv.m_elements[3];
	}
	return *this;
}

const float& Vector4f_::operator [] ( int i ) const
{
	return m_elements[ i ];
}

float& Vector4f_::operator [] ( int i )
{
	return m_elements[ i ];
}

float& Vector4f_::x()
{
	return m_elements[ 0 ];
}

float& Vector4f_::y()
{
	return m_elements[ 1 ];
}

float& Vector4f_::z()
{
	return m_elements[ 2 ];
}

float& Vector4f_::w()
{
	return m_elements[ 3 ];
}

float Vector4f_::x() const
{
	return m_elements[0];
}

float Vector4f_::y() const
{
	return m_elements[1];
}

float Vector4f_::z() const
{
	return m_elements[2];
}

float Vector4f_::w() const
{
	return m_elements[3];
}

Vector2f_ Vector4f_::xy() const
{
	return Vector2f_( m_elements[0], m_elements[1] );
}

Vector2f_ Vector4f_::yz() const
{
	return Vector2f_( m_elements[1], m_elements[2] );
}

Vector2f_ Vector4f_::zw() const
{
	return Vector2f_( m_elements[2], m_elements[3] );
}

Vector2f_ Vector4f_::wx() const
{
	return Vector2f_( m_elements[3], m_elements[0] );
}

Vector3f_ Vector4f_::xyz() const
{
	return Vector3f_( m_elements[0], m_elements[1], m_elements[2] );
}

Vector3f_ Vector4f_::yzw() const
{
	return Vector3f_( m_elements[1], m_elements[2], m_elements[3] );
}

Vector3f_ Vector4f_::zwx() const
{
	return Vector3f_( m_elements[2], m_elements[3], m_elements[0] );
}

Vector3f_ Vector4f_::wxy() const
{
	return Vector3f_( m_elements[3], m_elements[0], m_elements[1] );
}

Vector3f_ Vector4f_::xyw() const
{
	return Vector3f_( m_elements[0], m_elements[1], m_elements[3] );
}

Vector3f_ Vector4f_::yzx() const
{
	return Vector3f_( m_elements[1], m_elements[2], m_elements[0] );
}

Vector3f_ Vector4f_::zwy() const
{
	return Vector3f_( m_elements[2], m_elements[3], m_elements[1] );
}

Vector3f_ Vector4f_::wxz() const
{
	return Vector3f_( m_elements[3], m_elements[0], m_elements[2] );
}

float Vector4f_::abs() const
{
	return sqrt( m_elements[0] * m_elements[0] + m_elements[1] * m_elements[1] + m_elements[2] * m_elements[2] + m_elements[3] * m_elements[3] );
}

float Vector4f_::absSquared() const
{
	return( m_elements[0] * m_elements[0] + m_elements[1] * m_elements[1] + m_elements[2] * m_elements[2] + m_elements[3] * m_elements[3] );
}

void Vector4f_::normalize()
{
	float norm = sqrt( m_elements[0] * m_elements[0] + m_elements[1] * m_elements[1] + m_elements[2] * m_elements[2] + m_elements[3] * m_elements[3] );
	m_elements[0] = m_elements[0] / norm;
	m_elements[1] = m_elements[1] / norm;
	m_elements[2] = m_elements[2] / norm;
	m_elements[3] = m_elements[3] / norm;
}

Vector4f_ Vector4f_::normalized() const
{
	float length = abs();
	return Vector4f_
		(
			m_elements[0] / length,
			m_elements[1] / length,
			m_elements[2] / length,
			m_elements[3] / length
		);
}

void Vector4f_::homogenize()
{
	if( m_elements[3] != 0 )
	{
		m_elements[0] /= m_elements[3];
		m_elements[1] /= m_elements[3];
		m_elements[2] /= m_elements[3];
		m_elements[3] = 1;
	}
}

Vector4f_ Vector4f_::homogenized() const
{
	if( m_elements[3] != 0 )
	{
		return Vector4f_
			(
				m_elements[0] / m_elements[3],
				m_elements[1] / m_elements[3],
				m_elements[2] / m_elements[3],
				1
			);
	}
	else
	{
		return Vector4f_
			(
				m_elements[0],
				m_elements[1],
				m_elements[2],
				m_elements[3]
			);
	}
}

void Vector4f_::negate()
{
	m_elements[0] = -m_elements[0];
	m_elements[1] = -m_elements[1];
	m_elements[2] = -m_elements[2];
	m_elements[3] = -m_elements[3];
}

Vector4f_::operator const float* () const
{
	return m_elements;
}

Vector4f_::operator float* ()
{
	return m_elements;
}

void Vector4f_::print() const
{
	printf( "< %.4f, %.4f, %.4f, %.4f >\n",
		m_elements[0], m_elements[1], m_elements[2], m_elements[3] );
}

// static
float Vector4f_::dot( const Vector4f_& v0, const Vector4f_& v1 )
{
	return v0.x() * v1.x() + v0.y() * v1.y() + v0.z() * v1.z() + v0.w() * v1.w();
}

// static
Vector4f_ Vector4f_::lerp( const Vector4f_& v0, const Vector4f_& v1, float alpha )
{
	return alpha * ( v1 - v0 ) + v0;
}

//////////////////////////////////////////////////////////////////////////
// Operators
//////////////////////////////////////////////////////////////////////////

Vector4f_ operator + ( const Vector4f_& v0, const Vector4f_& v1 )
{
	return Vector4f_( v0.x() + v1.x(), v0.y() + v1.y(), v0.z() + v1.z(), v0.w() + v1.w() );
}

Vector4f_ operator - ( const Vector4f_& v0, const Vector4f_& v1 )
{
	return Vector4f_( v0.x() - v1.x(), v0.y() - v1.y(), v0.z() - v1.z(), v0.w() - v1.w() );
}

Vector4f_ operator * ( const Vector4f_& v0, const Vector4f_& v1 )
{
	return Vector4f_( v0.x() * v1.x(), v0.y() * v1.y(), v0.z() * v1.z(), v0.w() * v1.w() );
}

Vector4f_ operator / ( const Vector4f_& v0, const Vector4f_& v1 )
{
	return Vector4f_( v0.x() / v1.x(), v0.y() / v1.y(), v0.z() / v1.z(), v0.w() / v1.w() );
}

Vector4f_ operator - ( const Vector4f_& v )
{
	return Vector4f_( -v.x(), -v.y(), -v.z(), -v.w() );
}

Vector4f_ operator * ( float f, const Vector4f_& v )
{
	return Vector4f_( f * v.x(), f * v.y(), f * v.z(), f * v.w() );
}

Vector4f_ operator * ( const Vector4f_& v, float f )
{
	return Vector4f_( f * v.x(), f * v.y(), f * v.z(), f * v.w() );
}

Vector4f_ operator / ( const Vector4f_& v, float f )
{
    return Vector4f_( v[0] / f, v[1] / f, v[2] / f, v[3] / f );
}

bool operator == ( const Vector4f_& v0, const Vector4f_& v1 )
{
    return( v0.x() == v1.x() && v0.y() == v1.y() && v0.z() == v1.z() && v0.w() == v1.w() );
}

bool operator != ( const Vector4f_& v0, const Vector4f_& v1 )
{
    return !( v0 == v1 );
}
