#ifndef MATRIX4F_H
#define MATRIX4F_H

#include <cstdio>

class Matrix2f_;
class Matrix3f_;
class Quat4f_;
class Vector3f_;
class Vector4f_;

// 4x4 Matrix, stored in column major order (OpenGL style)
class Matrix4f_
{
public:

    // Fill a 4x4 matrix with "fill".  Default to 0.
	Matrix4f_( float fill = 0.f );
	Matrix4f_( float m00, float m01, float m02, float m03,
		float m10, float m11, float m12, float m13,
		float m20, float m21, float m22, float m23,
		float m30, float m31, float m32, float m33 );
	
	// setColumns = true ==> sets the columns of the matrix to be [v0 v1 v2 v3]
	// otherwise, sets the rows
	Matrix4f_( const Vector4f_& v0, const Vector4f_& v1, const Vector4f_& v2, const Vector4f_& v3, bool setColumns = true );
	
	Matrix4f_( const Matrix4f_& rm ); // copy constructor
	Matrix4f_& operator = ( const Matrix4f_& rm ); // assignment operator
	Matrix4f_& operator/=(float d);
	// no destructor necessary

	const float& operator () ( int i, int j ) const;
	float& operator () ( int i, int j );

	Vector4f_ getRow( int i ) const;
	void setRow( int i, const Vector4f_& v );

	// get column j (mod 4)
	Vector4f_ getCol( int j ) const;
	void setCol( int j, const Vector4f_& v );

	// gets the 2x2 submatrix of this matrix to m
	// starting with upper left corner at (i0, j0)
	Matrix2f_ getSubmatrix2x2( int i0, int j0 ) const;

	// gets the 3x3 submatrix of this matrix to m
	// starting with upper left corner at (i0, j0)
	Matrix3f_ getSubmatrix3x3( int i0, int j0 ) const;

	// sets a 2x2 submatrix of this matrix to m
	// starting with upper left corner at (i0, j0)
	void setSubmatrix2x2( int i0, int j0, const Matrix2f_& m );

	// sets a 3x3 submatrix of this matrix to m
	// starting with upper left corner at (i0, j0)
	void setSubmatrix3x3( int i0, int j0, const Matrix3f_& m );

	float determinant() const;
	Matrix4f_ inverse( bool* pbIsSingular = NULL, float epsilon = 0.f ) const;

	void transpose();
	Matrix4f_ transposed() const;

	// ---- Utility ----
	operator float* (); // automatic type conversion for GL
	operator const float* () const; // automatic type conversion for GL
	
	void print();

	static Matrix4f_ ones();
	static Matrix4f_ identity();
	static Matrix4f_ translation( float x, float y, float z );
	static Matrix4f_ translation( const Vector3f_& rTranslation );
	static Matrix4f_ rotateX( float radians );
	static Matrix4f_ rotateY( float radians );
	static Matrix4f_ rotateZ( float radians );
	static Matrix4f_ rotation( const Vector3f_& rDirection, float radians );
	static Matrix4f_ scaling( float sx, float sy, float sz );
	static Matrix4f_ uniformScaling( float s );
	static Matrix4f_ lookAt( const Vector3f_& eye, const Vector3f_& center, const Vector3f_& up );
	static Matrix4f_ orthographicProjection( float width, float height, float zNear, float zFar, bool directX );
	static Matrix4f_ orthographicProjection( float left, float right, float bottom, float top, float zNear, float zFar, bool directX );
	static Matrix4f_ perspectiveProjection( float fLeft, float fRight, float fBottom, float fTop, float fZNear, float fZFar, bool directX );
	static Matrix4f_ perspectiveProjection( float fovYRadians, float aspect, float zNear, float zFar, bool directX );
	static Matrix4f_ infinitePerspectiveProjection( float fLeft, float fRight, float fBottom, float fTop, float fZNear, bool directX );

	// Returns the rotation matrix represented by a quaternion
	// uses a normalized version of q
	static Matrix4f_ rotation( const Quat4f_& q );

	// returns an orthogonal matrix that's a uniformly distributed rotation
	// given u[i] is a uniformly distributed random number in [0,1]
	static Matrix4f_ randomRotation( float u0, float u1, float u2 );

private:

	float m_elements[ 16 ];

};

// Matrix-Vector multiplication
// 4x4 * 4x1 ==> 4x1
Vector4f_ operator * ( const Matrix4f_& m, const Vector4f_& v );

// Matrix-Matrix multiplication
Matrix4f_ operator * ( const Matrix4f_& x, const Matrix4f_& y );

#endif // MATRIX4F_H
