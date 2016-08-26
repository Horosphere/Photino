#ifndef PHOTINO_MATH_VECTOR3_H_
#define PHOTINO_MATH_VECTOR3_H_

#include <cstdint>
#include <cassert>

/*
 * Benchmarking details: (Apply for quaternion and matrices as well)
 * T vs T const&: No performance difference
 *
 * operator[] vs .x .y .z: No performance difference in Release with
 *	optimisation. operator[] is a little slower in Debug.
 *	3.98s (operator[]) vs 3.76s (.x.y.z) for 4097152 samples
 *
 * triple division vs inverse multiplication: No performance difference in
 *	Release. In Debug, triple division (2.63s) is slower than inverse mult.
 *	(2.59s)
 */

namespace photino
{

template <typename T>
struct vector3
{
	vector3(): x(0), y(0), z(0)
	{
	}

	vector3(T const& x, T const& y, T const& z): x(x), y(y), z(z)
	{
	}

	T operator[](std::size_t i) const
	{
		assert(i < 3);
		return i == 0 ? x :
		       i == 1 ? y :
		       z;
	}
	T& operator[](std::size_t i)
	{
		assert(i < 3);
		return i == 0 ? x :
		       i == 1 ? y :
		       z;
	}

	vector3<T>& operator+=(vector3<T> const&);
	vector3<T>& operator-=(vector3<T> const&);
	vector3<T>& operator*=(T const& s);
	vector3<T>& operator/=(T const& s);

	T x, y, z;
};

template <typename T> vector3<T>
operator+(vector3<T> const&, vector3<T> const&);
template <typename T> vector3<T>
operator-(vector3<T> const&, vector3<T> const&);
template <typename T> vector3<T>
operator*(vector3<T> const&, T const&);
template <typename T> vector3<T>
operator*(T const&, vector3<T> const&);
template <typename T> vector3<T>
operator/(vector3<T> const&, T const&);
template <typename T> vector3<T>
operator/(T const&, vector3<T> const&);

template <typename T> T norm(vector3<T> const&);
template <typename T> T normSq(vector3<T> const&);
template <typename T> vector3<T> unit(vector3<T> const&);
template <typename T> T dot(vector3<T> const&, vector3<T> const&);
template <typename T> vector3<T> cross(vector3<T> const&, vector3<T> const&);


// Implementations

template <typename T> inline vector3<T>&
vector3<T>::operator+=(vector3<T> const& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}
template <typename T> inline vector3<T>&
vector3<T>::operator-=(vector3<T> const& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}
template <typename T> inline vector3<T>&
vector3<T>::operator*=(T const& s)
{
	x *= s;
	y *= s;
	z *= s;
	return *this;
}
template <typename T> inline vector3<T>&
vector3<T>::operator/=(T const& s)
{
	x /= s;
	y /= s;
	z /= s;
	return *this;
}

template <typename T> inline vector3<T>
operator+(vector3<T> const& v0, vector3<T> const& v1)
{
	return vector3<T>(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z);
}
template <typename T> inline vector3<T>
operator-(vector3<T> const& v0, vector3<T> const& v1)
{
	return vector3<T>(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}
template <typename T> inline vector3<T>
operator*(vector3<T> const& v, T const& s)
{
	return vector3<T>(v.x * s, v.y * s, v.z * s);
}
template <typename T> inline vector3<T>
operator*(T const& s, vector3<T> const& v)
{
	return vector3<T>(v.x * s, v.y * s, v.z * s);
}
template <typename T> inline vector3<T>
operator/(vector3<T> const& v, T const& s)
{
	return vector3<T>(v.x / s, v.y / s, v.z / s);
}
template <typename T> inline vector3<T>
operator/(T const& s, vector3<T> const& v)
{
	return vector3<T>(v.x / s, v.y / s, v.z / s);
}

template <typename T> inline T
norm(vector3<T> const& v)
{
	return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
template <typename T> inline T
normSq(vector3<T> const& v)
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}
template <typename T> inline vector3<T>
unit(vector3<T> const& v)
{
	return v / norm(v);
}
template <typename T> inline T
dot(vector3<T> const& v0, vector3<T> const& v1)
{
	return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}
template <typename T> inline vector3<T>
cross(vector3<T> const& v0, vector3<T> const& v1)
{
	return vector3<T>(v0.y * v1.z - v0.z * v1.y,
	                  v0.z * v1.x - v0.x * v1.z,
	                  v0.x * v1.y - v0.y * v1.x);
}

} // namespace photino

#endif // !PHOTINO_MATH_VECTOR3_H_
