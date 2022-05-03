#ifndef MATRIX3F_H
#define MATRIX3F_H

#include <cstdio>

class Matrix2f_;
class Quat4f_;
class Vector3f_;

// 3x3 Matrix, stored in column major order (OpenGL style)
class Matrix3f_
{
public:

    // Fill a 3x3 matrix with "fill", default to 0.
	Matrix3f_( float fill = 0.f );
	Matrix3f_( float m00, float m01, float m02,
		float m10, float m11, float m12,
		float m20, float m21, float m22 );

	// setColumns = true ==> sets the columns of the matrix to be [v0 v1 v2]
	// otherwise, sets the rows
	Matrix3f_( const Vector3f_& v0, const Vector3f_& v1, const Vector3f_& v2, bool setColumns = true );

	Matrix3f_( const Matrix3f_& rm ); // copy constructor
	Matrix3f_& operator = ( const Matrix3f_& rm ); // assignment operator
	// no destructor necessary

	const float& operator () ( int i, int j ) const;
	float& operator () ( int i, int j );

	Vector3f_ getRow( int i ) const;
	void setRow( int i, const Vector3f_& v );

	Vector3f_ getCol( int j ) const;
	void setCol( int j, const Vector3f_& v );

	// gets the 2x2 submatrix of this matrix to m
	// starting with upper left corner at (i0, j0)
	Matrix2f_ getSubmatrix2x2( int i0, int j0 ) const;

	// sets a 2x2 submatrix of this matrix to m
	// starting with upper left corner at (i0, j0)
	void setSubmatrix2x2( int i0, int j0, const Matrix2f_& m );

	float determinant() const;
	Matrix3f_ inverse( bool* pbIsSingular = NULL, float epsilon = 0.f ) const; // TODO: invert in place as well

	void transpose();
	Matrix3f_ transposed() const;

	// ---- Utility ----
	operator float* (); // automatic type conversion for GL
	void print();

	static float determinant3x3( float m00, float m01, float m02,
		float m10, float m11, float m12,
		float m20, float m21, float m22 );

	static Matrix3f_ ones();
	static Matrix3f_ identity();
	static Matrix3f_ rotateX( float radians );
	static Matrix3f_ rotateY( float radians );
	static Matrix3f_ rotateZ( float radians );
	static Matrix3f_ scaling( float sx, float sy, float sz );
	static Matrix3f_ uniformScaling( float s );
	static Matrix3f_ rotation( const Vector3f_& rDirection, float radians );

	// Returns the rotation matrix represented by a unit quaternion
	// if q is not normalized, it it normalized first
	static Matrix3f_ rotation( const Quat4f_& rq );

private:

	float m_elements[ 9 ];

};

// Matrix-Vector multiplication
// 3x3 * 3x1 ==> 3x1
Vector3f_ operator * ( const Matrix3f_& m, const Vector3f_& v );

// Matrix-Matrix multiplication
Matrix3f_ operator * ( const Matrix3f_& x, const Matrix3f_& y );

#endif // MATRIX3F_H
