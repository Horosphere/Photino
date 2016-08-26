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

	T elements[m][n];
};

} // namespace photino


#endif // !PHOTINO_MATH_MATRIX_H_
