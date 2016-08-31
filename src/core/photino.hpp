#ifndef PHOTINO_CORE_PHOTINO_HPP_
#define PHOTINO_CORE_PHOTINO_HPP_

#include <random>
#include <limits>

namespace photino
{

#define PHOTINO_MEMALIGN 64

typedef double real;
#undef INFINITY
constexpr real const INFINITY = std::numeric_limits<real>::max();

typedef std::default_random_engine Random;

} // namespace photino


#endif // !PHOTINO_CORE_PHOTINO_HPP_
