#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>

#include "Quat4f_.h"
#include "Vector3f_.h"
#include "Vector4f_.h"

//////////////////////////////////////////////////////////////////////////
// Public
//////////////////////////////////////////////////////////////////////////

// static
const Quat4f_ Quat4f_::ZERO = Quat4f_( 0, 0, 0, 0 );

// static
const Quat4f_ Quat4f_::IDENTITY = Quat4f_( 1, 0, 0, 0 );

Quat4f_::Quat4f_()
{
	m_elements[ 0 ] = 0;
	m_elements[ 1 ] = 0;
	m_elements[ 2 ] = 0;
	m_elements[ 3 ] = 0;
}

Quat4f_::Quat4f_( float w, float x, float y, float z )
{
	m_elements[ 0 ] = w;
	m_elements[ 1 ] = x;
	m_elements[ 2 ] = y;
	m_elements[ 3 ] = z;
}

Quat4f_::Quat4f_( const Quat4f_& rq )
{
	m_elements[ 0 ] = rq.m_elements[ 0 ];
	m_elements[ 1 ] = rq.m_elements[ 1 ];
	m_elements[ 2 ] = rq.m_elements[ 2 ];
	m_elements[ 3 ] = rq.m_elements[ 3 ];
}

Quat4f_& Quat4f_::operator = ( const Quat4f_& rq )
{
	if( this != ( &rq ) )
	{
		m_elements[ 0 ] = rq.m_elements[ 0 ];
		m_elements[ 1 ] = rq.m_elements[ 1 ];
		m_elements[ 2 ] = rq.m_elements[ 2 ];
		m_elements[ 3 ] = rq.m_elements[ 3 ];
	}
    return( *this );
}

Quat4f_::Quat4f_( const Vector3f_& v )
{
	m_elements[ 0 ] = 0;
	m_elements[ 1 ] = v[ 0 ];
	m_elements[ 2 ] = v[ 1 ];
	m_elements[ 3 ] = v[ 2 ];
}

Quat4f_::Quat4f_( const Vector4f_& v )
{
	m_elements[ 0 ] = v[ 0 ];
	m_elements[ 1 ] = v[ 1 ];
	m_elements[ 2 ] = v[ 2 ];
	m_elements[ 3 ] = v[ 3 ];
}

const float& Quat4f_::operator [] ( int i ) const
{
	return m_elements[ i ];
}

float& Quat4f_::operator [] ( int i )
{
	return m_elements[ i ];
}

float Quat4f_::w() const
{
	return m_elements[ 0 ];
}

float Quat4f_::x() const
{
	return m_elements[ 1 ];
}

float Quat4f_::y() const
{
	return m_elements[ 2 ];
}

float Quat4f_::z() const
{
	return m_elements[ 3 ];
}

Vector3f_ Quat4f_::xyz() const
{
	return Vector3f_
	(
		m_elements[ 1 ],
		m_elements[ 2 ],
		m_elements[ 3 ]
	);
}

Vector4f_ Quat4f_::wxyz() const
{
	return Vector4f_
	(
		m_elements[ 0 ],
		m_elements[ 1 ],
		m_elements[ 2 ],
		m_elements[ 3 ]
	);
}

float Quat4f_::abs() const
{
	return sqrt( absSquared() );	
}

float Quat4f_::absSquared() const
{
	return
	(
		m_elements[ 0 ] * m_elements[ 0 ] +
		m_elements[ 1 ] * m_elements[ 1 ] +
		m_elements[ 2 ] * m_elements[ 2 ] +
		m_elements[ 3 ] * m_elements[ 3 ]
	);
}

void Quat4f_::normalize()
{
	float reciprocalAbs = 1.f / abs();

	m_elements[ 0 ] *= reciprocalAbs;
	m_elements[ 1 ] *= reciprocalAbs;
	m_elements[ 2 ] *= reciprocalAbs;
	m_elements[ 3 ] *= reciprocalAbs;
}

Quat4f_ Quat4f_::normalized() const
{
	Quat4f_ q( *this );
	q.normalize();
	return q;
}

void Quat4f_::conjugate()
{
	m_elements[ 1 ] = -m_elements[ 1 ];
	m_elements[ 2 ] = -m_elements[ 2 ];
	m_elements[ 3 ] = -m_elements[ 3 ];
}

Quat4f_ Quat4f_::conjugated() const
{
	return Quat4f_
	(
		 m_elements[ 0 ],
		-m_elements[ 1 ],
		-m_elements[ 2 ],
		-m_elements[ 3 ]
	);
}

void Quat4f_::invert()
{
	Quat4f_ inverse = conjugated() * ( 1.0f / absSquared() );

	m_elements[ 0 ] = inverse.m_elements[ 0 ];
	m_elements[ 1 ] = inverse.m_elements[ 1 ];
	m_elements[ 2 ] = inverse.m_elements[ 2 ];
	m_elements[ 3 ] = inverse.m_elements[ 3 ];
}

Quat4f_ Quat4f_::inverse() const
{
	return conjugated() * ( 1.0f / absSquared() );
}


Quat4f_ Quat4f_::log() const
{
	float len =
		sqrt
		(
			m_elements[ 1 ] * m_elements[ 1 ] +
			m_elements[ 2 ] * m_elements[ 2 ] +
			m_elements[ 3 ] * m_elements[ 3 ]
		);

	if( len < 1e-6 )
	{
		return Quat4f_( 0, m_elements[ 1 ], m_elements[ 2 ], m_elements[ 3 ] );
	}
	else
	{
		float coeff = acos( m_elements[ 0 ] ) / len;
		return Quat4f_( 0, m_elements[ 1 ] * coeff, m_elements[ 2 ] * coeff, m_elements[ 3 ] * coeff );
	}
}

Quat4f_ Quat4f_::exp() const
{
	float theta =
		sqrt
		(
			m_elements[ 1 ] * m_elements[ 1 ] +
			m_elements[ 2 ] * m_elements[ 2 ] +
			m_elements[ 3 ] * m_elements[ 3 ]
		);

	if( theta < 1e-6 )
	{
		return Quat4f_( cos( theta ), m_elements[ 1 ], m_elements[ 2 ], m_elements[ 3 ] );
	}
	else
	{
		float coeff = sin( theta ) / theta;
		return Quat4f_( cos( theta ), m_elements[ 1 ] * coeff, m_elements[ 2 ] * coeff, m_elements[ 3 ] * coeff );
	}
}

Vector3f_ Quat4f_::getAxisAngle( float* radiansOut )
{
	float theta = acos( w() ) * 2;
	float vectorNorm = sqrt( x() * x() + y() * y() + z() * z() );
	float reciprocalVectorNorm = 1.f / vectorNorm;

	*radiansOut = theta;
	return Vector3f_
	(
		x() * reciprocalVectorNorm,
		y() * reciprocalVectorNorm,
		z() * reciprocalVectorNorm
	);
}

void Quat4f_::setAxisAngle( float radians, const Vector3f_& axis )
{
	m_elements[ 0 ] = cos( radians / 2 );

	float sinHalfTheta = sin( radians / 2 );
	float vectorNorm = axis.abs();
	float reciprocalVectorNorm = 1.f / vectorNorm;

	m_elements[ 1 ] = axis.x() * sinHalfTheta * reciprocalVectorNorm;
	m_elements[ 2 ] = axis.y() * sinHalfTheta * reciprocalVectorNorm;
	m_elements[ 3 ] = axis.z() * sinHalfTheta * reciprocalVectorNorm;
}

void Quat4f_::print()
{
	printf( "< %.4f + %.4f i + %.4f j + %.4f k >\n",
		m_elements[ 0 ], m_elements[ 1 ], m_elements[ 2 ], m_elements[ 3 ] );
}

// static
float Quat4f_::dot( const Quat4f_& q0, const Quat4f_& q1 )
{
	return
	(
		q0.w() * q1.w() +
		q0.x() * q1.x() +
		q0.y() * q1.y() +
		q0.z() * q1.z()
	);
}

// static
Quat4f_ Quat4f_::lerp( const Quat4f_& q0, const Quat4f_& q1, float alpha )
{
	return( ( q0 + alpha * ( q1 - q0 ) ).normalized() );
}

// static
Quat4f_ Quat4f_::slerp( const Quat4f_& a, const Quat4f_& b, float t, bool allowFlip )
{
	float cosAngle = Quat4f_::dot( a, b );

	float c1;
	float c2;

	// Linear interpolation for close orientations
	if( ( 1.0f - fabs( cosAngle ) ) < 0.01f )
	{
		c1 = 1.0f - t;
		c2 = t;
	}
	else
	{
		// Spherical interpolation
		float angle = acos( fabs( cosAngle ) );
		float sinAngle = sin( angle );
		c1 = sin( angle * ( 1.0f - t ) ) / sinAngle;
		c2 = sin( angle * t ) / sinAngle;
	}

	// Use the shortest path
	if( allowFlip && ( cosAngle < 0.0f ) )
	{
		c1 = -c1;
	}

	return Quat4f_( c1 * a[ 0 ] + c2 * b[ 0 ], c1 * a[ 1 ] + c2 * b[ 1 ], c1 * a[ 2 ] + c2 * b[ 2 ], c1 * a[ 3 ] + c2 * b[ 3 ] );
}

// static
Quat4f_ Quat4f_::squad( const Quat4f_& a, const Quat4f_& tanA, const Quat4f_& tanB, const Quat4f_& b, float t )
{
	Quat4f_ ab = Quat4f_::slerp( a, b, t );
	Quat4f_ tangent = Quat4f_::slerp( tanA, tanB, t, false );
	return Quat4f_::slerp( ab, tangent, 2.0f * t * ( 1.0f - t ), false );
}

// static
Quat4f_ Quat4f_::cubicInterpolate( const Quat4f_& q0, const Quat4f_& q1, const Quat4f_& q2, const Quat4f_& q3, float t )
{
	// geometric construction:
	//            t
	//   (t+1)/2     t/2
	// t+1        t	        t-1

	// bottom level
	Quat4f_ q0q1 = Quat4f_::slerp( q0, q1, t + 1 );
	Quat4f_ q1q2 = Quat4f_::slerp( q1, q2, t );
	Quat4f_ q2q3 = Quat4f_::slerp( q2, q3, t - 1 );

	// middle level
	Quat4f_ q0q1_q1q2 = Quat4f_::slerp( q0q1, q1q2, 0.5f * ( t + 1 ) );
	Quat4f_ q1q2_q2q3 = Quat4f_::slerp( q1q2, q2q3, 0.5f * t );

	// top level
	return Quat4f_::slerp( q0q1_q1q2, q1q2_q2q3, t );
}

// static
Quat4f_ Quat4f_::logDifference( const Quat4f_& a, const Quat4f_& b )
{
	Quat4f_ diff = a.inverse() * b;
	diff.normalize();
	return diff.log();
}

// static
Quat4f_ Quat4f_::squadTangent( const Quat4f_& before, const Quat4f_& center, const Quat4f_& after )
{
	Quat4f_ l1 = Quat4f_::logDifference( center, before );
	Quat4f_ l2 = Quat4f_::logDifference( center, after );
	
	Quat4f_ e;
	for( int i = 0; i < 4; ++i )
	{
		e[ i ] = -0.25f * ( l1[ i ] + l2[ i ] );
	}
	e = center * ( e.exp() );

	return e;
}

// static
Quat4f_ Quat4f_::fromRotationMatrix( const Matrix3f_& m )
{
	float x;
	float y;
	float z;
	float w;

	// Compute one plus the trace of the matrix
	float onePlusTrace = 1.0f + m( 0, 0 ) + m( 1, 1 ) + m( 2, 2 );

	if( onePlusTrace > 1e-5 )
	{
		// Direct computation
		float s = sqrt( onePlusTrace ) * 2.0f;
		x = ( m( 2, 1 ) - m( 1, 2 ) ) / s;
		y = ( m( 0, 2 ) - m( 2, 0 ) ) / s;
		z = ( m( 1, 0 ) - m( 0, 1 ) ) / s;
		w = 0.25f * s;
	}
	else
	{
		// Computation depends on major diagonal term
		if( ( m( 0, 0 ) > m( 1, 1 ) ) & ( m( 0, 0 ) > m( 2, 2 ) ) )
		{
			float s = sqrt( 1.0f + m( 0, 0 ) - m( 1, 1 ) - m( 2, 2 ) ) * 2.0f;
			x = 0.25f * s;
			y = ( m( 0, 1 ) + m( 1, 0 ) ) / s;
			z = ( m( 0, 2 ) + m( 2, 0 ) ) / s;
			w = ( m( 1, 2 ) - m( 2, 1 ) ) / s;
		}
		else if( m( 1, 1 ) > m( 2, 2 ) )
		{
			float s = sqrt( 1.0f + m( 1, 1 ) - m( 0, 0 ) - m( 2, 2 ) ) * 2.0f;
			x = ( m( 0, 1 ) + m( 1, 0 ) ) / s;
			y = 0.25f * s;
			z = ( m( 1, 2 ) + m( 2, 1 ) ) / s;
			w = ( m( 0, 2 ) - m( 2, 0 ) ) / s;
		}
		else
		{
			float s = sqrt( 1.0f + m( 2, 2 ) - m( 0, 0 ) - m( 1, 1 ) ) * 2.0f;
			x = ( m( 0, 2 ) + m( 2, 0 ) ) / s;
			y = ( m( 1, 2 ) + m( 2, 1 ) ) / s;
			z = 0.25f * s;
			w = ( m( 0, 1 ) - m( 1, 0 ) ) / s;
		}
	}

	Quat4f_ q( w, x, y, z );
	return q.normalized();
}

// static
Quat4f_ Quat4f_::fromRotatedBasis( const Vector3f_& x, const Vector3f_& y, const Vector3f_& z )
{
	return fromRotationMatrix( Matrix3f_( x, y, z ) );
}

// static
Quat4f_ Quat4f_::randomRotation( float u0, float u1, float u2 )
{
	float z = u0;
	float theta = static_cast< float >( 2.f * M_PI * u1 );
	float r = sqrt( 1.f - z * z );
	float w = static_cast< float >( M_PI * u2 );

	return Quat4f_
	(
		cos( w ),
		sin( w ) * cos( theta ) * r,
		sin( w ) * sin( theta ) * r,
		sin( w ) * z
	);
}

//////////////////////////////////////////////////////////////////////////
// Operators
//////////////////////////////////////////////////////////////////////////

Quat4f_ operator + ( const Quat4f_& q0, const Quat4f_& q1 )
{
	return Quat4f_
	(
		q0.w() + q1.w(),
		q0.x() + q1.x(),
		q0.y() + q1.y(),
		q0.z() + q1.z()
	);
}

Quat4f_ operator - ( const Quat4f_& q0, const Quat4f_& q1 )
{
	return Quat4f_
	(
		q0.w() - q1.w(),
		q0.x() - q1.x(),
		q0.y() - q1.y(),
		q0.z() - q1.z()
	);
}

Quat4f_ operator * ( const Quat4f_& q0, const Quat4f_& q1 )
{
	return Quat4f_
	(
		q0.w() * q1.w() - q0.x() * q1.x() - q0.y() * q1.y() - q0.z() * q1.z(),
		q0.w() * q1.x() + q0.x() * q1.w() + q0.y() * q1.z() - q0.z() * q1.y(),
		q0.w() * q1.y() - q0.x() * q1.z() + q0.y() * q1.w() + q0.z() * q1.x(),
		q0.w() * q1.z() + q0.x() * q1.y() - q0.y() * q1.x() + q0.z() * q1.w()
	);
}

Quat4f_ operator * ( float f, const Quat4f_& q )
{
	return Quat4f_
	(
		f * q.w(),
		f * q.x(),
		f * q.y(),
		f * q.z()
	);
}

Quat4f_ operator * ( const Quat4f_& q, float f )
{
	return Quat4f_
	(
		f * q.w(),
		f * q.x(),
		f * q.y(),
		f * q.z()
	);
}
