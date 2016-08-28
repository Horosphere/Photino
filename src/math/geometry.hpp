#ifndef PHOTINO_MATH_GEOMETRY_HPP_
#define PHOTINO_MATH_GEOMETRY_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "../core/photino.hpp"

/*
 * Wrapper for Eigen objects
 */

namespace photino
{

template <std::size_t m, std::size_t n>
using Matrix = Eigen::Matrix<real, m, n>;
template <std::size_t m>
using Vector = Eigen::Matrix<real, m, 1>;
template <std::size_t m>
using Normal = Eigen::Matrix<real, m, 1>;
template <std::size_t m>
using Point = Eigen::Matrix<real, m, 1>;
template <std::size_t m>
using Ray = Eigen::ParametrizedLine<real, m>;

typedef Eigen::Quaternion<real> Quaternion;

template <std::size_t m>
using BoxAxisAligned = Eigen::AlignedBox<real, m>;

// Wrapper functions with no impact on performance

auto cross(Matrix<3, 1> const& v0, Matrix<3, 1> const& v1)
	-> decltype(v0.cross(v1));
template <std::size_t m, std::size_t n> auto
norm(Matrix<m, n> const& mat) -> decltype(mat.norm());
template <std::size_t m, std::size_t n> auto
unit(Matrix<m, n> const& mat) -> decltype(mat.normalized());


// Implementations
inline auto
cross(Matrix<3, 1> const& v0, Matrix<3, 1> const& v1) -> decltype(v0.cross(v1))
{
	return v0.cross(v1);
}
template <std::size_t m, std::size_t n> inline auto
norm(Matrix<m, n> const& mat) -> decltype(mat.norm())
{
	return mat.norm();
}
template <std::size_t m, std::size_t n> inline auto
unit(Matrix<m, n> const& mat) -> decltype(mat.normalized())
{
	return mat.normalized();
}

} // namespace photino

#endif // !PHOTINO_MATH_GEOMETRY_HPP_
