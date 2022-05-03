#ifndef QUAT4F_H
#define QUAT4F_H

class Vector3f_;
class Vector4f_;

#include "Matrix3f_.h"

class Quat4f_
{
public:

	static const Quat4f_ ZERO;
	static const Quat4f_ IDENTITY;

	Quat4f_();

	// q = w + x * i + y * j + z * k
	Quat4f_( float w, float x, float y, float z );
		
	Quat4f_( const Quat4f_& rq ); // copy constructor
	Quat4f_& operator = ( const Quat4f_& rq ); // assignment operator
	// no destructor necessary

	// returns a quaternion with 0 real part
	Quat4f_( const Vector3f_& v );

	// copies the components of a Vector4f_ directly into this quaternion
	Quat4f_( const Vector4f_& v );

	// returns the ith element
	const float& operator [] ( int i ) const;
	float& operator [] ( int i );

	float w() const;
	float x() const;
	float y() const;
	float z() const;
	Vector3f_ xyz() const;
	Vector4f_ wxyz() const;

	float abs() const;
	float absSquared() const;
	void normalize();
	Quat4f_ normalized() const;

	void conjugate();
	Quat4f_ conjugated() const;

	void invert();
	Quat4f_ inverse() const;

	// log and exponential maps
	Quat4f_ log() const;
	Quat4f_ exp() const;
	
	// returns unit vector for rotation and radians about the unit vector
	Vector3f_ getAxisAngle( float* radiansOut );

	// sets this quaternion to be a rotation of fRadians about v = < fx, fy, fz >, v need not necessarily be unit length
	void setAxisAngle( float radians, const Vector3f_& axis );

	// ---- Utility ----
	void print();
 
	 // quaternion dot product (a la vector)
	static float dot( const Quat4f_& q0, const Quat4f_& q1 );
	
	// linear (stupid) interpolation
	static Quat4f_ lerp( const Quat4f_& q0, const Quat4f_& q1, float alpha );

	// spherical linear interpolation
	static Quat4f_ slerp( const Quat4f_& a, const Quat4f_& b, float t, bool allowFlip = true );
	
	// spherical quadratic interoplation between a and b at point t
	// given quaternion tangents tanA and tanB (can be computed using squadTangent)	
	static Quat4f_ squad( const Quat4f_& a, const Quat4f_& tanA, const Quat4f_& tanB, const Quat4f_& b, float t );

	static Quat4f_ cubicInterpolate( const Quat4f_& q0, const Quat4f_& q1, const Quat4f_& q2, const Quat4f_& q3, float t );

	// Log-difference between a and b, used for squadTangent
	// returns log( a^-1 b )	
	static Quat4f_ logDifference( const Quat4f_& a, const Quat4f_& b );

	// Computes a tangent at center, defined by the before and after quaternions
	// Useful for squad()
	static Quat4f_ squadTangent( const Quat4f_& before, const Quat4f_& center, const Quat4f_& after );

	static Quat4f_ fromRotationMatrix( const Matrix3f_& m );

	static Quat4f_ fromRotatedBasis( const Vector3f_& x, const Vector3f_& y, const Vector3f_& z );

	// returns a unit quaternion that's a uniformly distributed rotation
	// given u[i] is a uniformly distributed random number in [0,1]
	// taken from Graphics Gems II
	static Quat4f_ randomRotation( float u0, float u1, float u2 );

private:

	float m_elements[ 4 ];

};

Quat4f_ operator + ( const Quat4f_& q0, const Quat4f_& q1 );
Quat4f_ operator - ( const Quat4f_& q0, const Quat4f_& q1 );
Quat4f_ operator * ( const Quat4f_& q0, const Quat4f_& q1 );
Quat4f_ operator * ( float f, const Quat4f_& q );
Quat4f_ operator * ( const Quat4f_& q, float f );

#endif // QUAT4F_H
