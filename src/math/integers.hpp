#ifndef PHOTINO_MATH_INTEGERS_HPP_
#define PHOTINO_MATH_INTEGERS_HPP_

#include <cstdint>

namespace photino
{

/**
 * @brief Rounds a integer up to a multiple of mod
 */
template <typename T> T roundUpModulo(T x, T mod);

uint32_t roundUpPow2(uint32_t);
uint64_t roundUpPow2(uint64_t);


// Implementations


template <typename T> inline T
roundUpModulo(T x, T mod)
{
	return (x + mod - 1) & ~(mod - 1);
}
uint32_t roundUpPow2(uint32_t i)
{
	--i;
	i |= i >> 1;
	i |= i >> 2;
	i |= i >> 4;
	i |= i >> 8;
	i |= i >> 16;
	++i;
	return i;
}
uint64_t roundUpPow2(uint64_t i)
{
	--i;
	i |= i >> 1;
	i |= i >> 2;
	i |= i >> 4;
	i |= i >> 8;
	i |= i >> 16;
	i |= i >> 32;
	++i;
	return i;

}
} // namespace photino


#endif // !PHOTINO_MATH_INTEGERS_HPP_
