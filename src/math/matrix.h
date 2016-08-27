#ifndef PHOTINO_MATH_MATRIX_H_
#define PHOTINO_MATH_MATRIX_H_

#include <cassert>
#include <cstdint>

/*
 * Benchmarking:
 * Direct access vs operator(): No performance difference in Release.
 *	operator() is slower in Debug.
 */
namespace photino
{

template <typename T, std::size_t m, std::size_t n>
struct matrix
{

	T operator()(std::size_t j, std::size_t k) const
	{
		return elements[j][k];
	}
	T& operator()(std::size_t j, std::size_t k)
	{
		return elements[j][k];
	}

	matrix<T, m, n>& operator+=(matrix<T, m, n> const&);
	matrix<T, m, n>& operator-=(matrix<T, m, n> const&);
	matrix<T, m, n>& operator*=(T const&);
	matrix<T, m, n>& operator/=(T const&);
	matrix<T, m, n>& operator*=(matrix<T, n, n> const&);
	
	T elements[m][n];
};



template <typename T, std::size_t m, std::size_t n> matrix<T, m, n>
operator+(matrix<T, m, n> const&, matrix<T, m, n> const&);
template <typename T, std::size_t m, std::size_t n> matrix<T, m, n>
operator-(matrix<T, m, n> const&, matrix<T, m, n> const&);
template <typename T, std::size_t m, std::size_t n> matrix<T, m, n>
operator*(matrix<T, m, n> const&, T const&);
template <typename T, std::size_t m, std::size_t n> matrix<T, m, n>
operator*(T const&, matrix<T, m, n> const&);
template <typename T, std::size_t m, std::size_t n> matrix<T, m, n>
operator/(matrix<T, m, n> const&, T const&);
template <typename T, std::size_t m, std::size_t n> matrix<T, m, n>
operator/(T const&, matrix<T, m, n> const&);

template <typename T, std::size_t m, std::size_t n, std::size_t p> matrix<T, m, p>
operator*(matrix<T, m, n> const&, matrix<T, n, p> const&);

template <typename T, std::size_t m, std::size_t n> bool
operator==(matrix<T, m, n> const&, matrix<T, m, n> const&);
template <typename T, std::size_t m, std::size_t n> bool
operator!=(matrix<T, m, n> const&, matrix<T, m, n> const&);


// Implementations

template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>&
matrix<T, m, n>::operator+=(matrix<T, m, n> const& mat)
{
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			elements[j][k] += mat(j, k);
	return *this;
}
template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>&
matrix<T, m, n>::operator-=(matrix<T, m, n> const& mat)
{
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			elements[j][k] -= mat(j, k);
	return *this;
}
template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>&
matrix<T, m, n>::operator*=(T const& s)
{
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			elements[j][k] *= s;
	return *this;
}
template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>&
matrix<T, m, n>::operator/=(T const& s)
{
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			elements[j][k] /= s;
	return *this;
}
template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>&
matrix<T, m, n>::operator*=(matrix<T, n, n> const& mat)
{
	for (std::size_t j = 0; j < m; ++j)
	{
		T row[m];
		for (std::size_t r = 0; r < n; ++r)
		{
			row[r] = 0;
			for (std::size_t k = 0; k < n; ++k)
				row[r] += elements[j][k] * mat(k, r);
		}
		memcpy(elements[j], row, sizeof(row));
	}
		
	return *this;
}

template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>
operator+(matrix<T, m, n> const& mat0, matrix<T, m, n> const& mat1)
{
	matrix<T, m, n> result;
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			result(j, k) = mat0(j, k) + mat1(j, k);
}
template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>
operator-(matrix<T, m, n> const& mat0, matrix<T, m, n> const& mat1)
{
	matrix<T, m, n> result;
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			result(j, k) = mat0(j, k) - mat1(j, k);
}
template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>
operator*(matrix<T, m, n> const& mat, T const& s)
{
	matrix<T, m, n> result;
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			result(j, k) = mat(j, k) * s;
}
template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>
operator*(T const& s, matrix<T, m, n> const& mat)
{
	matrix<T, m, n> result;
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			result(j, k) = mat(j, k) * s;
}
template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>
operator/(matrix<T, m, n> const& mat, T const& s)
{
	matrix<T, m, n> result;
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			result(j, k) = mat(j, k) / s;
}
template <typename T, std::size_t m, std::size_t n> inline matrix<T, m, n>
operator/(T const& s, matrix<T, m, n> const& mat)
{
	matrix<T, m, n> result;
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			result(j, k) = mat(j, k) / s;
}

template <typename T, std::size_t m, std::size_t n, std::size_t p>
inline matrix<T, m, p>
operator*(matrix<T, m, n> const& mat0, matrix<T, n, p> const& mat1)
{
	matrix<T, m, p> result;
	for (std::size_t j = 0; j < m; ++j)
			for (std::size_t r = 0; r < p; ++r)
			{
				result(j, r) = 0;
				for (std::size_t k = 0; k < n; ++k)
					result(j, r) += mat0(j, k) * mat1(k, r);
			}
}

template <typename T, std::size_t m, std::size_t n> inline bool
operator==(matrix<T, m, n> const& mat0, matrix<T, m, n> const& mat1)
{
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			if (mat0(j, k) != mat1(j, k)) return false;
	return true;
}
template <typename T, std::size_t m, std::size_t n> inline bool
operator!=(matrix<T, m, n> const& mat0, matrix<T, m, n> const& mat1)
{
	for (std::size_t j = 0; j < m; ++j)
		for (std::size_t k = 0; k < n; ++k)
			if (mat0(j, k) != mat1(j, k)) return true;
	return false;
}


} // namespace photino


#endif // !PHOTINO_MATH_MATRIX_H_
