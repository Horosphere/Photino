#ifndef PHOTINO_MATH_QUATERNION_H_
#define PHOTINO_MATH_QUATERNION_H_

#include <cstdint>
#include <cassert>

namespace photino
{

template <typename T>
struct quaternion
{
	quaternion(): w(0), x(0), y(0), z(0)
	{
	}

	quaternion(T const& w, T const& x, T const& y, T const& z):
		w(w), x(x), y(y), z(z)
	{
	}

	T operator[](std::size_t i) const
	{
		assert(i < 4);
		return i == 0 ? w :
		       i == 1 ? x :
		       i == 2 ? y :
		       z;
	}
	T& operator[](std::size_t i)
	{
		assert(i < 4);
		return i == 0 ? w :
		       i == 1 ? x :
		       i == 2 ? y :
		       z;
	}

	quaternion<T>& operator+=(quaternion<T> const&);
	quaternion<T>& operator-=(quaternion<T> const&);
	quaternion<T>& operator*=(T const& s);
	quaternion<T>& operator*=(quaternion<T> const&);
	quaternion<T>& operator/=(T const& s);

	T w, x, y, z;
};

template <typename T> quaternion<T>
operator+(quaternion<T> const&, quaternion<T> const&);
template <typename T> quaternion<T>
operator-(quaternion<T> const&, quaternion<T> const&);
template <typename T> quaternion<T>
operator*(quaternion<T> const&, T const&);
template <typename T> quaternion<T>
operator*(T const&, quaternion<T> const&);
template <typename T> quaternion<T>
operator*(quaternion<T> const&, quaternion<T> const&);
template <typename T> quaternion<T>
operator/(quaternion<T> const&, T const&);
template <typename T> quaternion<T>
operator/(T const&, quaternion<T> const&);

template <typename T> quaternion<T>
conj(quaternion<T> const&);
template <typename T> quaternion<T>
inverse(quaternion<T> const&);

// Implementations

template <typename T> inline quaternion<T>&
quaternion<T>::operator+=(quaternion<T> const& q)
{
	w += q.w;
	x += q.x;
	y += q.y;
	z += q.z;
	return *this;
}
template <typename T> inline quaternion<T>&
quaternion<T>::operator-=(quaternion<T> const& q)
{
	w -= q.w;
	x -= q.x;
	y -= q.y;
	z -= q.z;
	return *this;
}
template <typename T> inline quaternion<T>&
quaternion<T>::operator*=(T const& s)
{
	w *= s;
	x *= s;
	y *= s;
	z *= s;
	return *this;
}
template <typename T> inline quaternion<T>&
quaternion<T>::operator*=(quaternion<T> const& q)
{
	T x = w * q.x + x * q.w + y * q.z - z * q.y;
	T y = w * q.y + y * q.w + z * q.x - x * q.z;
	T z = w * q.z + z * q.w + x * q.y - y * q.x;
	this->w = w * q.w - x * q.x - y * q.y - z * q.z;
	this->x = x;
	this->y = y;
	this->z = z;
	return *this;
}
template <typename T> inline quaternion<T>&
quaternion<T>::operator/=(T const& s)
{
	w /= s;
	x /= s;
	y /= s;
	z /= s;
	return *this;
}

template <typename T> inline quaternion<T>
operator+(quaternion<T> const& q0, quaternion<T> const& q1)
{
	return quaternion<T>(q0.w + q1.w, q0.x + q1.x, q0.y + q1.y, q0.z + q1.z);
}
template <typename T> inline quaternion<T>
operator-(quaternion<T> const& q0, quaternion<T> const& q1)
{
	return quaternion<T>(q0.w - q1.w, q0.x - q1.x, q0.y - q1.y, q0.z - q1.z);
}
template <typename T> inline quaternion<T>
operator*(quaternion<T> const& q, T const& s)
{
	return quaternion<T>(q.w * s, q.x * s, q.y * s, q.z * s);
}
template <typename T> inline quaternion<T>
operator*(T const& s, quaternion<T> const& q)
{
	return quaternion<T>(q.w * s, q.x * s, q.y * s, q.z * s);
}
template <typename T> inline quaternion<T>
operator*(quaternion<T> const& q0, quaternion<T> const& q1)
{
	return quaternion<T>(q0.w * q1.w - q0.x * q1.x - q1.y * q1.y - q0.z * q1.z,
	                     q0.x * q1.w + q0.w * q1.x + q0.y * q1.z - q0.z - q1.y,
	                     q0.y * q1.w + q0.w * q1.y + q0.z * q1.x - q0.x - q1.z,
	                     q0.z * q1.w + q0.w * q1.z + q0.x * q1.y - q0.y * q1.x);
}
template <typename T> inline quaternion<T>
operator/(quaternion<T> const& q, T const& s)
{
	return quaternion<T>(q.w / s, q.x / s, q.y / s, q.z / s);
}
template <typename T> inline quaternion<T>
operator/(T const& s, quaternion<T> const& q)
{
	return quaternion<T>(q.w / s, q.x / s, q.y / s, q.z / s);
}


template <typename T> inline quaternion<T>
conj(quaternion<T> const& q)
{
	return quaternion<T>(q.w, -q.x, -q.y, -q.z);
}
template <typename T> inline quaternion<T>
inverse(quaternion<T> const& q)
{
	return conj(q) / (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

} // namespace photino


#endif // !PHOTINO_MATH_QUATERNION_H_
