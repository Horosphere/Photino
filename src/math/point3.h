#ifndef PHOTINO_MATH_POINT3_H_
#define PHOTINO_MATH_POINT3_H_


#include "vector3.h"

namespace photino
{

template <typename T>
struct point3
{
	point3(): x(0), y(0), z(0)
	{
	}

	point3(T const& x, T const& y, T const& z): x(x), y(y), z(z)
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

	point3<T>& operator+=(vector3<T> const&);
	point3<T>& operator-=(vector3<T> const&);

	T x, y, z;
};

template <typename T> point3<T>
operator+(point3<T> const&, vector3<T> const&);
template <typename T> vector3<T>
operator-(point3<T> const&, point3<T> const&);
template <typename T> point3<T>
operator-(point3<T> const&, vector3<T> const&);


// Implementations

template <typename T> inline point3<T>&
point3<T>::operator+=(vector3<T> const& v)
{
	x += v.x; y += v.y; z += v.z;
	return *this;
}
template <typename T> inline point3<T>&
point3<T>::operator-=(vector3<T> const& v)
{
	x -= v.x; y -= v.y; z -= v.z;
	return *this;
}
template <typename T> inline point3<T>
operator+(point3<T> const& p, vector3<T> const& v)
{
	return point3<T>(p.x + v.x, p.y + v.y, p.z + v.z);
}
template <typename T> inline vector3<T>
operator-(point3<T> const& p, point3<T> const& v)
{
	return point3<T>(p.x - v.x, p.y - v.y, p.z - v.z);
}
template <typename T> inline point3<T>
operator-(point3<T> const& p0, vector3<T> const& p1)
{
	return vector3<T>(p0.x - p1.x, p0.y - p1.y, p0.z - p1.z);
}


} // namespace photino

#endif // !PHOTINO_MATH_POINT3_H_
