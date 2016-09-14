#ifndef PHOTINO_MATH_NUMBERS_HPP_
#define PHOTINO_MATH_NUMBERS_HPP_

#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974
#endif

namespace photino
{

template <typename S, typename T> T
lerp(S const& time, T const& t0, T const& t1);


// Implementations

template <typename S, typename T> inline T
lerp(S const& time, T const& t0, T const& t1)
{
	return (1 - time) * t0 + time * t1;
}

} // namespace photino

#endif // !PHOTINO_MATH_NUMBERS_HPP_
