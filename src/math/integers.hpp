#ifndef PHOTINO_MATH_INTEGERS_HPP_
#define PHOTINO_MATH_INTEGERS_HPP_

namespace photino
{

/**
 * @brief Rounds a integer up to a multiple of mod
 */
template <typename T> T roundUpModulo(T x, T mod);


// Implementations


template <typename T> inline T
roundUpModulo(T x, T mod)
{
	return (x + mod - 1) & ~(mod - 1);
}

} // namespace photino


#endif // !PHOTINO_MATH_INTEGERS_HPP_
