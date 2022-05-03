#include "Matrix4f_.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "Matrix2f_.h"
#include "Matrix3f_.h"
#include "Quat4f_.h"
#include "Vector3f_.h"
#include "Vector4f_.h"

Matrix4f_::Matrix4f_( float fill )
{
	for( int i = 0; i < 16; ++i )
	{
		m_elements[ i ] = fill;
	}
}

Matrix4f_::Matrix4f_( float m00, float m01, float m02, float m03,
				   float m10, float m11, float m12, float m13,
				   float m20, float m21, float m22, float m23,
				   float m30, float m31, float m32, float m33 )
{
	m_elements[ 0 ] = m00;
	m_elements[ 1 ] = m10;
	m_elements[ 2 ] = m20;
	m_elements[ 3 ] = m30;
	
	m_elements[ 4 ] = m01;
	m_elements[ 5 ] = m11;
	m_elements[ 6 ] = m21;
	m_elements[ 7 ] = m31;

	m_elements[ 8 ] = m02;
	m_elements[ 9 ] = m12;
	m_elements[ 10 ] = m22;
	m_elements[ 11 ] = m32;

	m_elements[ 12 ] = m03;
	m_elements[ 13 ] = m13;
	m_elements[ 14 ] = m23;
	m_elements[ 15 ] = m33;
}

Matrix4f_& Matrix4f_::operator/=(float d)
{
	for(int ii=0;ii<16;ii++){
		m_elements[ii]/=d;
	}
	return *this;
}

Matrix4f_::Matrix4f_( const Vector4f_& v0, const Vector4f_& v1, const Vector4f_& v2, const Vector4f_& v3, bool setColumns )
{
	if( setColumns )
	{
		setCol( 0, v0 );
		setCol( 1, v1 );
		setCol( 2, v2 );
		setCol( 3, v3 );
	}
	else
	{
		setRow( 0, v0 );
		setRow( 1, v1 );
		setRow( 2, v2 );
		setRow( 3, v3 );
	}
}

Matrix4f_::Matrix4f_( const Matrix4f_& rm )
{
	memcpy( m_elements, rm.m_elements, 16 * sizeof( float ) );
}

Matrix4f_& Matrix4f_::operator = ( const Matrix4f_& rm )
{
	if( this != &rm )
	{
		memcpy( m_elements, rm.m_elements, 16 * sizeof( float ) );
	}
	return *this;
}

const float& Matrix4f_::operator () ( int i, int j ) const
{
	return m_elements[ j * 4 + i ];
}

float& Matrix4f_::operator () ( int i, int j )
{
	return m_elements[ j * 4 + i ];
}

Vector4f_ Matrix4f_::getRow( int i ) const
{
	return Vector4f_
	(
		m_elements[ i ],
		m_elements[ i + 4 ],
		m_elements[ i + 8 ],
		m_elements[ i + 12 ]
	);
}

void Matrix4f_::setRow( int i, const Vector4f_& v )
{
	m_elements[ i ] = v.x();
	m_elements[ i + 4 ] = v.y();
	m_elements[ i + 8 ] = v.z();
	m_elements[ i + 12 ] = v.w();
}

Vector4f_ Matrix4f_::getCol( int j ) const
{
	int colStart = 4 * j;

	return Vector4f_
	(
		m_elements[ colStart ],
		m_elements[ colStart + 1 ],
		m_elements[ colStart + 2 ],
		m_elements[ colStart + 3 ]
	);
}

void Matrix4f_::setCol( int j, const Vector4f_& v )
{
	int colStart = 4 * j;

	m_elements[ colStart ] = v.x();
	m_elements[ colStart + 1 ] = v.y();
	m_elements[ colStart + 2 ] = v.z();
	m_elements[ colStart + 3 ] = v.w();
}

Matrix2f_ Matrix4f_::getSubmatrix2x2( int i0, int j0 ) const
{
	Matrix2f_ out;

	for( int i = 0; i < 2; ++i )
	{
		for( int j = 0; j < 2; ++j )
		{
			out( i, j ) = ( *this )( i + i0, j + j0 );
		}
	}

	return out;
}

Matrix3f_ Matrix4f_::getSubmatrix3x3( int i0, int j0 ) const
{
	Matrix3f_ out;

	for( int i = 0; i < 3; ++i )
	{
		for( int j = 0; j < 3; ++j )
		{
			out( i, j ) = ( *this )( i + i0, j + j0 );
		}
	}

	return out;
}

void Matrix4f_::setSubmatrix2x2( int i0, int j0, const Matrix2f_& m )
{
	for( int i = 0; i < 2; ++i )
	{
		for( int j = 0; j < 2; ++j )
		{
			( *this )( i + i0, j + j0 ) = m( i, j );
		}
	}
}

void Matrix4f_::setSubmatrix3x3( int i0, int j0, const Matrix3f_& m )
{
	for( int i = 0; i < 3; ++i )
	{
		for( int j = 0; j < 3; ++j )
		{
			( *this )( i + i0, j + j0 ) = m( i, j );
		}
	}
}

float Matrix4f_::determinant() const
{
	float m00 = m_elements[ 0 ];
	float m10 = m_elements[ 1 ];
	float m20 = m_elements[ 2 ];
	float m30 = m_elements[ 3 ];

	float m01 = m_elements[ 4 ];
	float m11 = m_elements[ 5 ];
	float m21 = m_elements[ 6 ];
	float m31 = m_elements[ 7 ];

	float m02 = m_elements[ 8 ];
	float m12 = m_elements[ 9 ];
	float m22 = m_elements[ 10 ];
	float m32 = m_elements[ 11 ];

	float m03 = m_elements[ 12 ];
	float m13 = m_elements[ 13 ];
	float m23 = m_elements[ 14 ];
	float m33 = m_elements[ 15 ];

	float cofactor00 =  Matrix3f_::determinant3x3( m11, m12, m13, m21, m22, m23, m31, m32, m33 );
	float cofactor01 = -Matrix3f_::determinant3x3( m12, m13, m10, m22, m23, m20, m32, m33, m30 );
	float cofactor02 =  Matrix3f_::determinant3x3( m13, m10, m11, m23, m20, m21, m33, m30, m31 );
	float cofactor03 = -Matrix3f_::determinant3x3( m10, m11, m12, m20, m21, m22, m30, m31, m32 );

	return( m00 * cofactor00 + m01 * cofactor01 + m02 * cofactor02 + m03 * cofactor03 );
}

Matrix4f_ Matrix4f_::inverse( bool* pbIsSingular, float epsilon ) const
{
	float m00 = m_elements[ 0 ];
	float m10 = m_elements[ 1 ];
	float m20 = m_elements[ 2 ];
	float m30 = m_elements[ 3 ];

	float m01 = m_elements[ 4 ];
	float m11 = m_elements[ 5 ];
	float m21 = m_elements[ 6 ];
	float m31 = m_elements[ 7 ];

	float m02 = m_elements[ 8 ];
	float m12 = m_elements[ 9 ];
	float m22 = m_elements[ 10 ];
	float m32 = m_elements[ 11 ];

	float m03 = m_elements[ 12 ];
	float m13 = m_elements[ 13 ];
	float m23 = m_elements[ 14 ];
	float m33 = m_elements[ 15 ];

    float cofactor00 =  Matrix3f_::determinant3x3( m11, m12, m13, m21, m22, m23, m31, m32, m33 );
    float cofactor01 = -Matrix3f_::determinant3x3( m12, m13, m10, m22, m23, m20, m32, m33, m30 );
    float cofactor02 =  Matrix3f_::determinant3x3( m13, m10, m11, m23, m20, m21, m33, m30, m31 );
    float cofactor03 = -Matrix3f_::determinant3x3( m10, m11, m12, m20, m21, m22, m30, m31, m32 );
    
    float cofactor10 = -Matrix3f_::determinant3x3( m21, m22, m23, m31, m32, m33, m01, m02, m03 );
    float cofactor11 =  Matrix3f_::determinant3x3( m22, m23, m20, m32, m33, m30, m02, m03, m00 );
    float cofactor12 = -Matrix3f_::determinant3x3( m23, m20, m21, m33, m30, m31, m03, m00, m01 );
    float cofactor13 =  Matrix3f_::determinant3x3( m20, m21, m22, m30, m31, m32, m00, m01, m02 );
    
    float cofactor20 =  Matrix3f_::determinant3x3( m31, m32, m33, m01, m02, m03, m11, m12, m13 );
    float cofactor21 = -Matrix3f_::determinant3x3( m32, m33, m30, m02, m03, m00, m12, m13, m10 );
    float cofactor22 =  Matrix3f_::determinant3x3( m33, m30, m31, m03, m00, m01, m13, m10, m11 );
    float cofactor23 = -Matrix3f_::determinant3x3( m30, m31, m32, m00, m01, m02, m10, m11, m12 );
    
    float cofactor30 = -Matrix3f_::determinant3x3( m01, m02, m03, m11, m12, m13, m21, m22, m23 );
    float cofactor31 =  Matrix3f_::determinant3x3( m02, m03, m00, m12, m13, m10, m22, m23, m20 );
    float cofactor32 = -Matrix3f_::determinant3x3( m03, m00, m01, m13, m10, m11, m23, m20, m21 );
    float cofactor33 =  Matrix3f_::determinant3x3( m00, m01, m02, m10, m11, m12, m20, m21, m22 );

	float determinant = m00 * cofactor00 + m01 * cofactor01 + m02 * cofactor02 + m03 * cofactor03;

	bool isSingular = ( fabs( determinant ) < epsilon );
	if( isSingular )
	{
		if( pbIsSingular != NULL )
		{
			*pbIsSingular = true;
		}
		return Matrix4f_();
	}
	else
	{
		if( pbIsSingular != NULL )
		{
			*pbIsSingular = false;
		}

		float reciprocalDeterminant = 1.0f / determinant;

		return Matrix4f_
			(
				cofactor00 * reciprocalDeterminant, cofactor10 * reciprocalDeterminant, cofactor20 * reciprocalDeterminant, cofactor30 * reciprocalDeterminant,
				cofactor01 * reciprocalDeterminant, cofactor11 * reciprocalDeterminant, cofactor21 * reciprocalDeterminant, cofactor31 * reciprocalDeterminant,
				cofactor02 * reciprocalDeterminant, cofactor12 * reciprocalDeterminant, cofactor22 * reciprocalDeterminant, cofactor32 * reciprocalDeterminant,
				cofactor03 * reciprocalDeterminant, cofactor13 * reciprocalDeterminant, cofactor23 * reciprocalDeterminant, cofactor33 * reciprocalDeterminant
			);
	}
}

void Matrix4f_::transpose()
{
	float temp;

	for( int i = 0; i < 3; ++i )
	{
		for( int j = i + 1; j < 4; ++j )
		{
			temp = ( *this )( i, j );
			( * this )( i, j ) = ( *this )( j, i );
			( *this )( j, i ) = temp;
		}
	}
}

Matrix4f_ Matrix4f_::transposed() const
{
	Matrix4f_ out;
	for( int i = 0; i < 4; ++i )
	{
		for( int j = 0; j < 4; ++j )
		{
			out( j, i ) = ( *this )( i, j );
		}
	}

	return out;
}

Matrix4f_::operator float* ()
{
	return m_elements;
}

Matrix4f_::operator const float* ()const
{
	return m_elements;
}


void Matrix4f_::print()
{
	printf( "[ %.4f %.4f %.4f %.4f ]\n[ %.4f %.4f %.4f %.4f ]\n[ %.4f %.4f %.4f %.4f ]\n[ %.4f %.4f %.4f %.4f ]\n",
		m_elements[ 0 ], m_elements[ 4 ], m_elements[ 8 ], m_elements[ 12 ],
		m_elements[ 1 ], m_elements[ 5 ], m_elements[ 9 ], m_elements[ 13 ],
		m_elements[ 2 ], m_elements[ 6 ], m_elements[ 10], m_elements[ 14 ],
		m_elements[ 3 ], m_elements[ 7 ], m_elements[ 11], m_elements[ 15 ] );
}

// static
Matrix4f_ Matrix4f_::ones()
{
	Matrix4f_ m;
	for( int i = 0; i < 16; ++i )
	{
		m.m_elements[ i ] = 1;
	}

	return m;
}

// static
Matrix4f_ Matrix4f_::identity()
{
	Matrix4f_ m;
	
	m( 0, 0 ) = 1;
	m( 1, 1 ) = 1;
	m( 2, 2 ) = 1;
	m( 3, 3 ) = 1;

	return m;
}

// static
Matrix4f_ Matrix4f_::translation( float x, float y, float z )
{
	return Matrix4f_
	(
		1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1
	);
}

// static
Matrix4f_ Matrix4f_::translation( const Vector3f_& rTranslation )
{
	return Matrix4f_
	(
		1, 0, 0, rTranslation.x(),
		0, 1, 0, rTranslation.y(),
		0, 0, 1, rTranslation.z(),
		0, 0, 0, 1
	);
}

// static
Matrix4f_ Matrix4f_::rotateX( float radians )
{
	float c = cos( radians );
	float s = sin( radians );

	return Matrix4f_
	(
		1, 0, 0, 0,
		0, c, -s, 0,
		0, s, c, 0,
		0, 0, 0, 1
	);
}

// static
Matrix4f_ Matrix4f_::rotateY( float radians )
{
	float c = cos( radians );
	float s = sin( radians );

	return Matrix4f_
	(
		c, 0, s, 0,
		0, 1, 0, 0,
		-s, 0, c, 0,
		0, 0, 0, 1
	);
}

// static
Matrix4f_ Matrix4f_::rotateZ( float radians )
{
	float c = cos( radians );
	float s = sin( radians );

	return Matrix4f_
	(
		c, -s, 0, 0,
		s, c, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	);
}

// static
Matrix4f_ Matrix4f_::rotation( const Vector3f_& rDirection, float radians )
{
	Vector3f_ normalizedDirection = rDirection.normalized();
	
	float cosTheta = cos( radians );
	float sinTheta = sin( radians );

	float x = normalizedDirection.x();
	float y = normalizedDirection.y();
	float z = normalizedDirection.z();

	return Matrix4f_
	(
		x * x * ( 1.0f - cosTheta ) + cosTheta,			y * x * ( 1.0f - cosTheta ) - z * sinTheta,		z * x * ( 1.0f - cosTheta ) + y * sinTheta,		0.0f,
		x * y * ( 1.0f - cosTheta ) + z * sinTheta,		y * y * ( 1.0f - cosTheta ) + cosTheta,			z * y * ( 1.0f - cosTheta ) - x * sinTheta,		0.0f,
		x * z * ( 1.0f - cosTheta ) - y * sinTheta,		y * z * ( 1.0f - cosTheta ) + x * sinTheta,		z * z * ( 1.0f - cosTheta ) + cosTheta,			0.0f,
		0.0f,											0.0f,											0.0f,											1.0f
	);
}

// static
Matrix4f_ Matrix4f_::rotation( const Quat4f_& q )
{
	Quat4f_ qq = q.normalized();

	float xx = qq.x() * qq.x();
	float yy = qq.y() * qq.y();
	float zz = qq.z() * qq.z();

	float xy = qq.x() * qq.y();
	float zw = qq.z() * qq.w();

	float xz = qq.x() * qq.z();
	float yw = qq.y() * qq.w();

	float yz = qq.y() * qq.z();
	float xw = qq.x() * qq.w();

	return Matrix4f_
	(
		1.0f - 2.0f * ( yy + zz ),		2.0f * ( xy - zw ),				2.0f * ( xz + yw ),				0.0f,
		2.0f * ( xy + zw ),				1.0f - 2.0f * ( xx + zz ),		2.0f * ( yz - xw ),				0.0f,
		2.0f * ( xz - yw ),				2.0f * ( yz + xw ),				1.0f - 2.0f * ( xx + yy ),		0.0f,
		0.0f,							0.0f,							0.0f,							1.0f
	);
}

// static
Matrix4f_ Matrix4f_::scaling( float sx, float sy, float sz )
{
	return Matrix4f_
	(
		sx, 0, 0, 0,
		0, sy, 0, 0,
		0, 0, sz, 0,
		0, 0, 0, 1
	);
}

// static
Matrix4f_ Matrix4f_::uniformScaling( float s )
{
	return Matrix4f_
	(
		s, 0, 0, 0,
		0, s, 0, 0,
		0, 0, s, 0,
		0, 0, 0, 1
	);
}

// static
Matrix4f_ Matrix4f_::randomRotation( float u0, float u1, float u2 )
{
	return Matrix4f_::rotation( Quat4f_::randomRotation( u0, u1, u2 ) );
}

// static
Matrix4f_ Matrix4f_::lookAt( const Vector3f_& eye, const Vector3f_& center, const Vector3f_& up )
{
	// z is negative forward
	Vector3f_ z = ( eye - center ).normalized();
	Vector3f_ y = up;
	Vector3f_ x = Vector3f_::cross( y, z );

	// the x, y, and z vectors define the orthonormal coordinate system
	// the affine part defines the overall translation
	Matrix4f_ view;

	view.setRow( 0, Vector4f_( x, -Vector3f_::dot( x, eye ) ) );
	view.setRow( 1, Vector4f_( y, -Vector3f_::dot( y, eye ) ) );
	view.setRow( 2, Vector4f_( z, -Vector3f_::dot( z, eye ) ) );
	view.setRow( 3, Vector4f_( 0, 0, 0, 1 ) );

	return view;
}

// static
Matrix4f_ Matrix4f_::orthographicProjection( float width, float height, float zNear, float zFar, bool directX )
{
	Matrix4f_ m;

	m( 0, 0 ) = 2.0f / width;
	m( 1, 1 ) = 2.0f / height;
	m( 3, 3 ) = 1.0f;

	m( 0, 3 ) = -1;
	m( 1, 3 ) = -1;

	if( directX )
	{
		m( 2, 2 ) = 1.0f / ( zNear - zFar );
		m( 2, 3 ) = zNear / ( zNear - zFar );
	}
	else
	{
		m( 2, 2 ) = 2.0f / ( zNear - zFar );
		m( 2, 3 ) = ( zNear + zFar ) / ( zNear - zFar );
	}

	return m;
}

// static
Matrix4f_ Matrix4f_::orthographicProjection( float left, float right, float bottom, float top, float zNear, float zFar, bool directX )
{
	Matrix4f_ m;

	m( 0, 0 ) = 2.0f / ( right - left );
	m( 1, 1 ) = 2.0f / ( top - bottom );
	m( 3, 3 ) = 1.0f;

	m( 0, 3 ) = ( left + right ) / ( left - right );
	m( 1, 3 ) = ( top + bottom ) / ( bottom - top );

	if( directX )
	{
		m( 2, 2 ) = 1.0f / ( zNear - zFar );
		m( 2, 3 ) = zNear / ( zNear - zFar );
	}
	else
	{
		m( 2, 2 ) = 2.0f / ( zNear - zFar );
		m( 2, 3 ) = ( zNear + zFar ) / ( zNear - zFar );
	}

	return m;
}

// static
Matrix4f_ Matrix4f_::perspectiveProjection( float fLeft, float fRight,
										 float fBottom, float fTop,
										 float fZNear, float fZFar,
										 bool directX )
{
	Matrix4f_ projection; // zero matrix

	projection( 0, 0 ) = ( 2.0f * fZNear ) / ( fRight - fLeft );
	projection( 1, 1 ) = ( 2.0f * fZNear ) / ( fTop - fBottom );
	projection( 0, 2 ) = ( fRight + fLeft ) / ( fRight - fLeft );
	projection( 1, 2 ) = ( fTop + fBottom ) / ( fTop - fBottom );
	projection( 3, 2 ) = -1;

	if( directX )
	{
		projection( 2, 2 ) = fZFar / ( fZNear - fZFar);
		projection( 2, 3 ) = ( fZNear * fZFar ) / ( fZNear - fZFar );
	}
	else
	{
		projection( 2, 2 ) = ( fZNear + fZFar ) / ( fZNear - fZFar );
		projection( 2, 3 ) = ( 2.0f * fZNear * fZFar ) / ( fZNear - fZFar );
	}

	return projection;
}

// static
Matrix4f_ Matrix4f_::perspectiveProjection( float fovYRadians, float aspect, float zNear, float zFar, bool directX )
{
	Matrix4f_ m; // zero matrix

	float yScale = 1.f / tanf( 0.5f * fovYRadians );
	float xScale = yScale / aspect;

	m( 0, 0 ) = xScale;
	m( 1, 1 ) = yScale;
	m( 3, 2 ) = -1;

	if( directX )
	{
		m( 2, 2 ) = zFar / ( zNear - zFar );
		m( 2, 3 ) = zNear * zFar / ( zNear - zFar );
	}
	else
	{
		m( 2, 2 ) = ( zFar + zNear ) / ( zNear - zFar );
		m( 2, 3 ) = 2.f * zFar * zNear / ( zNear - zFar );
	}

	return m;
}

// static
Matrix4f_ Matrix4f_::infinitePerspectiveProjection( float fLeft, float fRight,
												 float fBottom, float fTop,
												 float fZNear, bool directX )
{
	Matrix4f_ projection;

	projection( 0, 0 ) = ( 2.0f * fZNear ) / ( fRight - fLeft );
	projection( 1, 1 ) = ( 2.0f * fZNear ) / ( fTop - fBottom );
	projection( 0, 2 ) = ( fRight + fLeft ) / ( fRight - fLeft );
	projection( 1, 2 ) = ( fTop + fBottom ) / ( fTop - fBottom );
	projection( 3, 2 ) = -1;

	// infinite view frustum
	// just take the limit as far --> inf of the regular frustum
	if( directX )
	{
		projection( 2, 2 ) = -1.0f;
		projection( 2, 3 ) = -fZNear;		
	}
	else
	{
		projection( 2, 2 ) = -1.0f;
		projection( 2, 3 ) = -2.0f * fZNear;
	}

	return projection;
}

//////////////////////////////////////////////////////////////////////////
// Operators
//////////////////////////////////////////////////////////////////////////

Vector4f_ operator * ( const Matrix4f_& m, const Vector4f_& v )
{
	Vector4f_ output( 0, 0, 0, 0 );

	for( int i = 0; i < 4; ++i )
	{
		for( int j = 0; j < 4; ++j )
		{
			output[ i ] += m( i, j ) * v[ j ];
		}
	}

	return output;
}

Matrix4f_ operator * ( const Matrix4f_& x, const Matrix4f_& y )
{
	Matrix4f_ product; // zeroes

	for( int i = 0; i < 4; ++i )
	{
		for( int j = 0; j < 4; ++j )
		{
			for( int k = 0; k < 4; ++k )
			{
				product( i, k ) += x( i, j ) * y( j, k );
			}
		}
	}

	return product;
}
