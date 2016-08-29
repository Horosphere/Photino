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

template <int m> BoxAxisAligned<m>&
operator|=(BoxAxisAligned<m>&, BoxAxisAligned<m> const&);
template <int m> BoxAxisAligned<m>&
operator&=(BoxAxisAligned<m>&, BoxAxisAligned<m> const&);
template <int m> BoxAxisAligned<m>
operator|(BoxAxisAligned<m> const&, BoxAxisAligned<m> const&);
template <int m> BoxAxisAligned<m>
operator&(BoxAxisAligned<m> const&, BoxAxisAligned<m> const&);


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

template <int m> inline BoxAxisAligned<m>&
operator|=(BoxAxisAligned<m>& b0, BoxAxisAligned<m> const& b1)
{
	return b0.extend(b1);
}
template <int m> inline BoxAxisAligned<m>&
operator&=(BoxAxisAligned<m>& b0, BoxAxisAligned<m> const& b1)
{
	return b0.clamp(b1);
}
template <int m> inline BoxAxisAligned<m>
operator|(BoxAxisAligned<m> const& b0, BoxAxisAligned<m> const& b1)
{
	return b0.merged(b1);
}
template <int m> inline BoxAxisAligned<m>
operator&(BoxAxisAligned<m> const& b0, BoxAxisAligned<m> const& b1)
{
	return b0.intersection(b1);
}

} // namespace photino

#endif // !PHOTINO_MATH_GEOMETRY_HPP_
