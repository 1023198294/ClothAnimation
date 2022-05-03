#include "Matrix2f_.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "Vector2f_.h"

Matrix2f_::Matrix2f_( float fill )
{
	for( int i = 0; i < 4; ++i )
	{
		m_elements[ i ] = fill;
	}
}

Matrix2f_::Matrix2f_( float m00, float m01,
				   float m10, float m11 )
{
	m_elements[ 0 ] = m00;
	m_elements[ 1 ] = m10;

	m_elements[ 2 ] = m01;
	m_elements[ 3 ] = m11;
}

Matrix2f_::Matrix2f_( const Vector2f_& v0, const Vector2f_& v1, bool setColumns )
{
	if( setColumns )
	{
		setCol( 0, v0 );
		setCol( 1, v1 );
	}
	else
	{
		setRow( 0, v0 );
		setRow( 1, v1 );
	}
}

Matrix2f_::Matrix2f_( const Matrix2f_& rm )
{
	memcpy( m_elements, rm.m_elements, 2 * sizeof( float ) );
}

Matrix2f_& Matrix2f_::operator = ( const Matrix2f_& rm )
{
	if( this != &rm )
	{
		memcpy( m_elements, rm.m_elements, 2 * sizeof( float ) );
	}
	return *this;
}

const float& Matrix2f_::operator () ( int i, int j ) const
{
	return m_elements[ j * 2 + i ];
}

float& Matrix2f_::operator () ( int i, int j )
{
	return m_elements[ j * 2 + i ];
}

Vector2f_ Matrix2f_::getRow( int i ) const
{
	return Vector2f_
	(
		m_elements[ i ],
		m_elements[ i + 2 ]
	);
}

void Matrix2f_::setRow( int i, const Vector2f_& v )
{
	m_elements[ i ] = v.x();
	m_elements[ i + 2 ] = v.y();
}

Vector2f_ Matrix2f_::getCol( int j ) const
{
	int colStart = 2 * j;

	return Vector2f_
	(
		m_elements[ colStart ],
		m_elements[ colStart + 1 ]
	);
}

void Matrix2f_::setCol( int j, const Vector2f_& v )
{
	int colStart = 2 * j;

	m_elements[ colStart ] = v.x();
	m_elements[ colStart + 1 ] = v.y();
}

float Matrix2f_::determinant()
{
	return Matrix2f_::determinant2x2
	(
		m_elements[ 0 ], m_elements[ 2 ],
		m_elements[ 1 ], m_elements[ 3 ]
	);
}

Matrix2f_ Matrix2f_::inverse( bool* pbIsSingular, float epsilon )
{
	float determinant = m_elements[ 0 ] * m_elements[ 3 ] - m_elements[ 2 ] * m_elements[ 1 ];

	bool isSingular = ( fabs( determinant ) < epsilon );
	if( isSingular )
	{
		if( pbIsSingular != NULL )
		{
			*pbIsSingular = true;
		}
		return Matrix2f_();
	}
	else
	{
		if( pbIsSingular != NULL )
		{
			*pbIsSingular = false;
		}

		float reciprocalDeterminant = 1.0f / determinant;

		return Matrix2f_
		(
			m_elements[ 3 ] * reciprocalDeterminant, -m_elements[ 2 ] * reciprocalDeterminant,
			-m_elements[ 1 ] * reciprocalDeterminant, m_elements[ 0 ] * reciprocalDeterminant
		);
	}
}

void Matrix2f_::transpose()
{
	float m01 = ( *this )( 0, 1 );
	float m10 = ( *this )( 1, 0 );

	( *this )( 0, 1 ) = m10;
	( *this )( 1, 0 ) = m01;
}

Matrix2f_ Matrix2f_::transposed() const
{
	return Matrix2f_
	(
		( *this )( 0, 0 ), ( *this )( 1, 0 ),
		( *this )( 0, 1 ), ( *this )( 1, 1 )
	);

}

Matrix2f_::operator float* ()
{
	return m_elements;
}

void Matrix2f_::print()
{
	printf( "[ %.4f %.4f ]\n[ %.4f %.4f ]\n",
		m_elements[ 0 ], m_elements[ 2 ],
		m_elements[ 1 ], m_elements[ 3 ] );
}

// static
float Matrix2f_::determinant2x2( float m00, float m01,
							   float m10, float m11 )
{
	return( m00 * m11 - m01 * m10 );
}

// static
Matrix2f_ Matrix2f_::ones()
{
	Matrix2f_ m;
	for( int i = 0; i < 4; ++i )
	{
		m.m_elements[ i ] = 1;
	}

	return m;
}

// static
Matrix2f_ Matrix2f_::identity()
{
	Matrix2f_ m;

	m( 0, 0 ) = 1;
	m( 1, 1 ) = 1;

	return m;
}

// static
Matrix2f_ Matrix2f_::rotation( float degrees )
{
	float c = cos( degrees );
	float s = sin( degrees );

	return Matrix2f_
	(
		c, -s,
		s, c
	);
}

//////////////////////////////////////////////////////////////////////////
// Operators
//////////////////////////////////////////////////////////////////////////

Matrix2f_ operator * ( float f, const Matrix2f_& m )
{
	Matrix2f_ output;

	for( int i = 0; i < 2; ++i )
	{
		for( int j = 0; j < 2; ++j )
		{
			output( i, j ) = f * m( i, j );
		}
	}

	return output;
}

Matrix2f_ operator * ( const Matrix2f_& m, float f )
{
	return f * m;
}

Vector2f_ operator * ( const Matrix2f_& m, const Vector2f_& v )
{
	Vector2f_ output( 0, 0 );

	for( int i = 0; i < 2; ++i )
	{
		for( int j = 0; j < 2; ++j )
		{
			output[ i ] += m( i, j ) * v[ j ];
		}
	}

	return output;
}

Matrix2f_ operator * ( const Matrix2f_& x, const Matrix2f_& y )
{
	Matrix2f_ product; // zeroes

	for( int i = 0; i < 2; ++i )
	{
		for( int j = 0; j < 2; ++j )
		{
			for( int k = 0; k < 2; ++k )
			{
				product( i, k ) += x( i, j ) * y( j, k );
			}
		}
	}

	return product;
}
