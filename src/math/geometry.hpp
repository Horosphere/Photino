#ifndef PHOTINO_MATH_GEOMETRY_HPP_
#define PHOTINO_MATH_GEOMETRY_HPP_

#include <Eigen/Dense>

#include "../core/photino.hpp"

/*
 * Wrapper for Eigen objects
 */

namespace photino
{

template <std::size_t m, std::size_t n = m>
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
template <int m, int n> auto
dot(Matrix<m, n> const& mat0, Matrix<m, n> const& mat1) -> decltype(mat0.dot(mat1));
template <int m, int n> auto
transpose(Matrix<m, n> const& mat) -> decltype(mat.transpose());
template <int m, int n> auto
conj(Matrix<m, n> const& mat) -> decltype(mat.conjugate());
template <int m, int n> auto
inverse(Matrix<m, n> const& mat) -> decltype(mat.inverse());
template <int m, int n> auto
norm2(Matrix<m, n> const& mat) -> decltype(mat.norm());
template <int m, int n> auto
norm2Sq(Matrix<m, n> const& mat) -> decltype(mat.squaredNorm());
template <int m, int n> auto
unit(Matrix<m, n> const& mat) -> decltype(mat.normalized());

Quaternion slerp(Quaternion const&, Quaternion const&, real);

template <int m> BoxAxisAligned<m>&
operator|=(BoxAxisAligned<m>&, BoxAxisAligned<m> const&);
template <int m> BoxAxisAligned<m>&
operator|=(BoxAxisAligned<m>&, Point<m> const&);
template <int m> BoxAxisAligned<m>&
operator&=(BoxAxisAligned<m>&, BoxAxisAligned<m> const&);
template <int m> BoxAxisAligned<m>
operator|(BoxAxisAligned<m>, Point<m> const&);
template <int m> BoxAxisAligned<m>
operator&(BoxAxisAligned<m> const&, BoxAxisAligned<m> const&);

template <int m> real
maxExtent(BoxAxisAligned<m> const&);
/**
 * @tparam m Dimension
 * @param[dim] The dimensional index of the longest axis. Cannot be nullptr.
 */
template <int m> real
maxExtent(BoxAxisAligned<m> const&, int* dim);


// Implementations
inline auto
cross(Matrix<3, 1> const& v0, Matrix<3, 1> const& v1) -> decltype(v0.cross(v1))
{
	return v0.cross(v1);
}
template <int m, int n> inline auto
dot(Matrix<m, n> const& mat0, Matrix<m, n> const& mat1) -> decltype(mat0.dot(mat1))
{
	return mat0.dot(mat1);
}
template <int m, int n> inline auto
transpose(Matrix<m, n> const& mat) -> decltype(mat.transpose())
{
	return mat.transpose();
}
template <int m, int n> inline auto
conj(Matrix<m, n> const& mat) -> decltype(mat.conjugate())
{
	return mat.conjugate();
}
template <int m, int n> inline auto
inverse(Matrix<m, n> const& mat) -> decltype(mat.inverse())
{
	return mat.inverse();
}
template <int m, int n> inline auto
norm2(Matrix<m, n> const& mat) -> decltype(mat.norm())
{
	return mat.norm();
}
template <int m, int n> inline auto
norm2Sq(Matrix<m, n> const& mat) -> decltype(mat.squaredNorm())
{
	return mat.squaredNorm();
}
template <int m, int n> inline auto
unit(Matrix<m, n> const& mat) -> decltype(mat.normalized())
{
	return mat.normalized();
}
inline Quaternion slerp(Quaternion const& q0, Quaternion const& q1, real t)
{
	return q0.slerp(t, q1);
}

template <int m> inline BoxAxisAligned<m>&
operator|=(BoxAxisAligned<m>& b0, BoxAxisAligned<m> const& b1)
{
	return b0.extend(b1);
}
template <int m> inline BoxAxisAligned<m>&
operator|=(BoxAxisAligned<m>& b, Point<m> const& p)
{
	return b.extend(p);
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
operator|(BoxAxisAligned<m> b, Point<m> const& p)
{
	return b.extend(p);
}
template <int m> inline BoxAxisAligned<m>
operator&(BoxAxisAligned<m> const& b0, BoxAxisAligned<m> const& b1)
{
	return b0.intersection(b1);
}

template <int m> inline real
maxExtent(BoxAxisAligned<m> const& b)
{
	return b.sizes().maxCoeff();
}
template <int m> inline real
maxExtent(BoxAxisAligned<m> const& b, int* index)
{
	return b.sizes().maxCoeff(index);
}

} // namespace photino

#endif // !PHOTINO_MATH_GEOMETRY_HPP_
