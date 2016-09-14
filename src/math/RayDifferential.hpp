#ifndef PHOTINO_MATH_RAYDIFFERENTIAL_HPP_
#define PHOTINO_MATH_RAYDIFFERENTIAL_HPP_

#include "geometry.hpp"

namespace photino
{

template <std::size_t m>
struct RayDifferential
{
	Ray<m> rx, ry;

	RayDifferential(Ray<m> const&, Ray<m> const&);

	RayDifferential<m>& scale(Ray<m> const& r, real s);
};

template <std::size_t m> inline
RayDifferential<m>::RayDifferential(Ray<m> const& rx, Ray<m> const& ry):
	rx(rx), ry(ry)
{
}

template <std::size_t m> inline RayDifferential<m>&
RayDifferential<m>::scale(Ray<m> const& r, real s)
{
	rx.origin() = r.origin() + (rx.origin() - r.origin()) * s;
	ry.origin() = r.origin() + (ry.origin() - r.origin()) * s;
	rx.direction() = r.direction() + (rx.direction() - r.direction()) * s;
	ry.direction() = r.direction() + (ry.direction() - r.direction()) * s;
	return *this;
}

} // namespace photino


#endif // !PHOTINO_MATH_RAYDIFFERENTIAL_HPP_
