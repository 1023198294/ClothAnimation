#ifndef VECTOR_3F_H
#define VECTOR_3F_H

class Vector2f_;

class Vector3f_
{
public:

	static const Vector3f_ ZERO;
	static const Vector3f_ UP;
	static const Vector3f_ RIGHT;
	static const Vector3f_ FORWARD;

    Vector3f_( float f = 0.f );
    Vector3f_( float x, float y, float z );

	Vector3f_( const Vector2f_& xy, float z );
	Vector3f_( float x, const Vector2f_& yz );

	// copy constructors
    Vector3f_( const Vector3f_& rv );

	// assignment operators
    Vector3f_& operator = ( const Vector3f_& rv );

	// no destructor necessary

	// returns the ith element
    const float& operator [] ( int i ) const;
    float& operator [] ( int i );

    float& x();
	float& y();
	float& z();

	float x() const;
	float y() const;
	float z() const;

	Vector2f_ xy() const;
	Vector2f_ xz() const;
	Vector2f_ yz() const;

	Vector3f_ xyz() const;
	Vector3f_ yzx() const;
	Vector3f_ zxy() const;

	float abs() const;
    float absSquared() const;

	void normalize();
	Vector3f_ normalized() const;

	Vector2f_ homogenized() const;

	void negate();

	// ---- Utility ----
    operator const float* () const; // automatic type conversion for OpenGL
    operator float* (); // automatic type conversion for OpenGL 
	void print() const;	

	Vector3f_& operator += ( const Vector3f_& v );
	Vector3f_& operator -= ( const Vector3f_& v );
    Vector3f_& operator *= ( float f );

    static float dot( const Vector3f_& v0, const Vector3f_& v1 );
	static Vector3f_ cross( const Vector3f_& v0, const Vector3f_& v1 );
    
    // computes the linear interpolation between v0 and v1 by alpha \in [0,1]
	// returns v0 * ( 1 - alpha ) * v1 * alpha
	static Vector3f_ lerp( const Vector3f_& v0, const Vector3f_& v1, float alpha );

	// computes the cubic catmull-rom interpolation between p0, p1, p2, p3
    // by t \in [0,1].  Guarantees that at t = 0, the result is p0 and
    // at p1, the result is p2.
	static Vector3f_ cubicInterpolate( const Vector3f_& p0, const Vector3f_& p1, const Vector3f_& p2, const Vector3f_& p3, float t );

private:

	float m_elements[ 3 ];

};

// component-wise operators
Vector3f_ operator + ( const Vector3f_& v0, const Vector3f_& v1 );
Vector3f_ operator - ( const Vector3f_& v0, const Vector3f_& v1 );
Vector3f_ operator * ( const Vector3f_& v0, const Vector3f_& v1 );
Vector3f_ operator / ( const Vector3f_& v0, const Vector3f_& v1 );

// unary negation
Vector3f_ operator - ( const Vector3f_& v );

// multiply and divide by scalar
Vector3f_ operator * ( float f, const Vector3f_& v );
Vector3f_ operator * ( const Vector3f_& v, float f );
Vector3f_ operator / ( const Vector3f_& v, float f );

bool operator == ( const Vector3f_& v0, const Vector3f_& v1 );
bool operator != ( const Vector3f_& v0, const Vector3f_& v1 );

#endif // VECTOR_3F_H
