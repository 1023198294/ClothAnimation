#ifndef MATRIX2F_H
#define MATRIX2F_H

#include <cstdio>

class Vector2f_;

// 2x2 Matrix, stored in column major order (OpenGL style)
class Matrix2f_
{
public:

    // Fill a 2x2 matrix with "fill", default to 0.
	Matrix2f_( float fill = 0.f );
	Matrix2f_( float m00, float m01,
		float m10, float m11 );

	// setColumns = true ==> sets the columns of the matrix to be [v0 v1]
	// otherwise, sets the rows
	Matrix2f_( const Vector2f_& v0, const Vector2f_& v1, bool setColumns = true );

	Matrix2f_( const Matrix2f_& rm ); // copy constructor
	Matrix2f_& operator = ( const Matrix2f_& rm ); // assignment operator
	// no destructor necessary

	const float& operator () ( int i, int j ) const;
	float& operator () ( int i, int j );

	Vector2f_ getRow( int i ) const;
	void setRow( int i, const Vector2f_& v );

	Vector2f_ getCol( int j ) const;
	void setCol( int j, const Vector2f_& v );

	float determinant();
	Matrix2f_ inverse( bool* pbIsSingular = NULL, float epsilon = 0.f );

	void transpose();
	Matrix2f_ transposed() const;

	// ---- Utility ----
	operator float* (); // automatic type conversion for GL
	void print();

	static float determinant2x2( float m00, float m01,
		float m10, float m11 );

	static Matrix2f_ ones();
	static Matrix2f_ identity();
	static Matrix2f_ rotation( float degrees );

private:

	float m_elements[ 4 ];

};

// Scalar-Matrix multiplication
Matrix2f_ operator * ( float f, const Matrix2f_& m );
Matrix2f_ operator * ( const Matrix2f_& m, float f );

// Matrix-Vector multiplication
// 2x2 * 2x1 ==> 2x1
Vector2f_ operator * ( const Matrix2f_& m, const Vector2f_& v );

// Matrix-Matrix multiplication
Matrix2f_ operator * ( const Matrix2f_& x, const Matrix2f_& y );

#endif // MATRIX2F_H
