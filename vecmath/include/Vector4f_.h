#ifndef VECTOR_4F_H
#define VECTOR_4F_H

class Vector2f_;
class Vector3f_;

class Vector4f_
{
public:

	Vector4f_( float f = 0.f );
	Vector4f_( float fx, float fy, float fz, float fw );
	Vector4f_( float buffer[ 4 ] );

	Vector4f_( const Vector2f_& xy, float z, float w );
	Vector4f_( float x, const Vector2f_& yz, float w );
	Vector4f_( float x, float y, const Vector2f_& zw );
	Vector4f_( const Vector2f_& xy, const Vector2f_& zw );

	Vector4f_( const Vector3f_& xyz, float w );
	Vector4f_( float x, const Vector3f_& yzw );

	// copy constructors
	Vector4f_( const Vector4f_& rv );

	// assignment operators
	Vector4f_& operator = ( const Vector4f_& rv );

	// no destructor necessary

	// returns the ith element
	const float& operator [] ( int i ) const;
	float& operator [] ( int i );

	float& x();
	float& y();
	float& z();
	float& w();

	float x() const;
	float y() const;
	float z() const;
	float w() const;

	Vector2f_ xy() const;
	Vector2f_ yz() const;
	Vector2f_ zw() const;
	Vector2f_ wx() const;

	Vector3f_ xyz() const;
	Vector3f_ yzw() const;
	Vector3f_ zwx() const;
	Vector3f_ wxy() const;

	Vector3f_ xyw() const;
	Vector3f_ yzx() const;
	Vector3f_ zwy() const;
	Vector3f_ wxz() const;

	float abs() const;
	float absSquared() const;
	void normalize();
	Vector4f_ normalized() const;

	// if v.z != 0, v = v / v.w
	void homogenize();
	Vector4f_ homogenized() const;

	void negate();

	// ---- Utility ----
	operator const float* () const; // automatic type conversion for OpenGL
	operator float* (); // automatic type conversion for OpenG
	void print() const; 

	static float dot( const Vector4f_& v0, const Vector4f_& v1 );
	static Vector4f_ lerp( const Vector4f_& v0, const Vector4f_& v1, float alpha );

private:

	float m_elements[ 4 ];

};

// component-wise operators
Vector4f_ operator + ( const Vector4f_& v0, const Vector4f_& v1 );
Vector4f_ operator - ( const Vector4f_& v0, const Vector4f_& v1 );
Vector4f_ operator * ( const Vector4f_& v0, const Vector4f_& v1 );
Vector4f_ operator / ( const Vector4f_& v0, const Vector4f_& v1 );

// unary negation
Vector4f_ operator - ( const Vector4f_& v );

// multiply and divide by scalar
Vector4f_ operator * ( float f, const Vector4f_& v );
Vector4f_ operator * ( const Vector4f_& v, float f );
Vector4f_ operator / ( const Vector4f_& v, float f );

bool operator == ( const Vector4f_& v0, const Vector4f_& v1 );
bool operator != ( const Vector4f_& v0, const Vector4f_& v1 );

#endif // VECTOR_4F_H
