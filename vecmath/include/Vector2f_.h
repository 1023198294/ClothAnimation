#ifndef VECTOR_2F_H
#define VECTOR_2F_H

#include <cmath>

class Vector3f_;

class Vector2f_
{
public:
    
    static const Vector2f_ ZERO;
	static const Vector2f_ UP;
	static const Vector2f_ RIGHT;

    Vector2f_( float f = 0.f );
    Vector2f_( float x, float y );

	// copy constructors
    Vector2f_( const Vector2f_& rv );

	// assignment operators
	Vector2f_& operator = ( const Vector2f_& rv );

	// no destructor necessary

	// returns the ith element
    const float& operator [] ( int i ) const;
	float& operator [] ( int i );

    float& x();
	float& y();

	float x() const;
	float y() const;

    Vector2f_ xy() const;
	Vector2f_ yx() const;
	Vector2f_ xx() const;
	Vector2f_ yy() const;

	// returns ( -y, x )
    Vector2f_ normal() const;

    float abs() const;
    float absSquared() const;
    void normalize();
    Vector2f_ normalized() const;

    void negate();

	// ---- Utility ----
    operator const float* () const; // automatic type conversion for OpenGL 
    operator float* (); // automatic type conversion for OpenGL 
	void print() const;

	Vector2f_& operator += ( const Vector2f_& v );
	Vector2f_& operator -= ( const Vector2f_& v );
	Vector2f_& operator *= ( float f );

    static float dot( const Vector2f_& v0, const Vector2f_& v1 );

	static Vector3f_ cross( const Vector2f_& v0, const Vector2f_& v1 );

	// returns v0 * ( 1 - alpha ) * v1 * alpha
	static Vector2f_ lerp( const Vector2f_& v0, const Vector2f_& v1, float alpha );

private:

	float m_elements[2];

};

// component-wise operators
Vector2f_ operator + ( const Vector2f_& v0, const Vector2f_& v1 );
Vector2f_ operator - ( const Vector2f_& v0, const Vector2f_& v1 );
Vector2f_ operator * ( const Vector2f_& v0, const Vector2f_& v1 );
Vector2f_ operator / ( const Vector2f_& v0, const Vector2f_& v1 );

// unary negation
Vector2f_ operator - ( const Vector2f_& v );

// multiply and divide by scalar
Vector2f_ operator * ( float f, const Vector2f_& v );
Vector2f_ operator * ( const Vector2f_& v, float f );
Vector2f_ operator / ( const Vector2f_& v, float f );

bool operator == ( const Vector2f_& v0, const Vector2f_& v1 );
bool operator != ( const Vector2f_& v0, const Vector2f_& v1 );

#endif // VECTOR_2F_H
