#ifndef PHOTINO_MATH_TRANSFORM_HPP_
#define PHOTINO_MATH_TRANSFORM_HPP_

#include "geometry.hpp"

namespace photino
{

template <int m, int type>
class Transform final
{
public:
	static Transform<m, type> identity();

	Transform() noexcept {}
	/**
	 * @warning Result undefined if the given matrix is singular.
	 */
	Transform(Matrix<m> const&);
	/**
	 * @warning Result undefined if the given matrix is singular.
	 */
	Transform(Matrix<m + 1> const&);
	/**
	 * @warning The two matrices must be inverse of each other.
	 */
	Transform(Matrix<m> const&, Matrix<m> const&) noexcept;
	/**
	 * @warning The two matrices must be inverse of each other.
	 */
	Transform(Matrix<m + 1> const&, Matrix<m + 1> const&) noexcept;

	Matrix<m + 1> const& matrix() const;
	Matrix<m + 1> const& inverseMatrix() const;
	bool swapsChirality() const;

	Transform<m, type>& translate(Vector<m> const&);
	Transform<m, type>& scale(Vector<m> const&);
	template <typename Rotation> Transform<m, type>&
	rotate(Rotation const&);
	Transform<m, type>& operator*=(Transform<m, type> const&);

	Point<m> trPoint(Point<m> const&) const;
	Vector<m> trVector(Vector<m> const&) const;
	Normal<m> trNormal(Normal<m> const&) const;
	Ray<m> trRay(Ray<m> const&) const;
	BoxAxisAligned<m> trBoxAA(BoxAxisAligned<m> const&) const;

private:
	Eigen::Transform<real, m, type> mat;
	Eigen::Transform<real, m, type> invMat;

	template <int n, int ty>
	friend Transform<n, ty> inverse(Transform<n, ty> const&) noexcept;
	template <int n, int ty> friend bool
	operator==(Transform<n, ty> const&, Transform<n, ty> const&);
	template <int n, int ty> friend bool
	operator!=(Transform<n, ty> const&, Transform<n, ty> const&);
	template <int n, int ty0, int ty1> friend Transform<m, ty0>
	operator*(Transform<m, ty0> const&, Transform<m, ty1> const&);
};


template <int m, int type> bool
operator==(Transform<m, type> const&, Transform<m, type> const&);
template <int m, int type> bool
operator!=(Transform<m, type> const&, Transform<m, type> const&);

template <int m, int type0, int type1> Transform<m, type0>
operator*(Transform<m, type0> const&, Transform<m, type1> const&);

template <int m, int type> Transform<m, type>
inverse(Transform<m, type> const&) noexcept;


template <int m>
using TransformAffine = Transform<m, Eigen::AffineCompact>;
template <int m>
using TransformProjective = Transform<m, Eigen::Projective>;

// Implementations

template <int m, int type> inline Transform<m, type>
Transform<m, type>::identity()
{
	Transform<m, type> result;
	result.invMat = result.mat = Eigen::Transform<real, m, type>::Identity();
	return result;
}

template <int m, int type> inline
Transform<m, type>::Transform(Matrix<m> const& mat):
	mat(mat), invMat(inverse(mat))
{
}
template <int m, int type> inline
Transform<m, type>::Transform(Matrix<m + 1> const& mat):
	mat(mat), invMat(inverse(mat))
{
}
template <int m, int type> inline
Transform<m, type>::Transform(Matrix<m> const& mat,
                              Matrix<m> const& invMat) noexcept:
	mat(mat), invMat(invMat)
{
}
template <int m, int type> inline
Transform<m, type>::Transform(Matrix<m + 1> const& mat,
                              Matrix<m + 1> const& invMat) noexcept:
	mat(mat), invMat(invMat)
{
}
template <int m, int type> inline Matrix<m + 1> const&
Transform<m, type>::matrix() const
{
	return mat.matrix();
}
template <int m, int type> inline Matrix<m + 1> const&
Transform<m, type>::inverseMatrix() const
{
	return invMat.matrix();
}
template <int m, int type> inline bool
Transform<m, type>::swapsChirality() const
{
	return mat.linear().determinant() < 0;
}

template <int m, int type> inline Transform<m, type>&
Transform<m, type>::translate(Vector<m> const& v)
{
	mat.translate(v);
	invMat.translate(-v);
	return *this;
}
template <int m, int type> inline Transform<m, type>&
Transform<m, type>::scale(Vector<m> const& v)
{
	mat.scale(v);
	invMat.scale(v.cwiseInverse());
	return *this;
}
template <int m, int type>
template <typename Rotation> inline Transform<m, type>&
Transform<m, type>::rotate(Rotation const& rotation)
{
	Matrix<m> rotationMatrix = rotation.toRotationMatrix();
	mat = mat * Eigen::Transform<real, m, Eigen::Affine>(rotationMatrix);
	invMat = invMat * Eigen::Transform<real, m, Eigen::Affine>(rotationMatrix.transpose());
	return *this;
}
template <int m, int type> inline Transform<m, type>&
Transform<m, type>::operator*=(Transform<m, type> const& t)
{
	mat = mat * t.mat;
	invMat = t.invMat * invMat;
	return *this;
}

template <int m, int type> Point<m>
Transform<m, type>::trPoint(Point<m> const& p) const
{
	return mat * p;
}
template <int m, int type> inline Vector<m>
Transform<m, type>::trVector(Vector<m> const& v) const
{
	return mat.linear() * v;
}
template <int m, int type> inline Normal<m>
Transform<m, type>::trNormal(Normal<m> const& n) const
{
	return invMat.linear().transpose() * n;
}
template <int m, int type> inline Ray<m>
Transform<m, type>::trRay(Ray<m> const& r) const
{
	return Ray<m>(trPoint(r.origin()), trVector(r.direction()));
}
template <int m, int type> inline BoxAxisAligned<m>
Transform<m, type>::trBoxAA(BoxAxisAligned<m> const& b) const
{
	assert(m == 3 && "Not implemented for dimensions other than 3");
	BoxAxisAligned<m> result(trPoint(b.corner(BoxAxisAligned<m>::BottomLeftFloor)));
	result |= trPoint(b.corner(BoxAxisAligned<m>::BottomLeftCeil));
	result |= trPoint(b.corner(BoxAxisAligned<m>::BottomRightFloor));
	result |= trPoint(b.corner(BoxAxisAligned<m>::BottomRightCeil));
	result |= trPoint(b.corner(BoxAxisAligned<m>::TopLeftFloor));
	result |= trPoint(b.corner(BoxAxisAligned<m>::TopLeftCeil));
	result |= trPoint(b.corner(BoxAxisAligned<m>::TopRightFloor));
	result |= trPoint(b.corner(BoxAxisAligned<m>::TopRightCeil));
	return result;
}

template <int m, int type> inline bool
operator==(Transform<m, type> const& t0, Transform<m, type> const& t1)
{
	return t0.mat == t1.mat;
}
template <int m, int type> bool
operator!=(Transform<m, type> const& t0, Transform<m, type> const& t1)
{
	return t0.mat != t1.mat;
}
template <int m, int type0, int type1> Transform<m, type0>
operator*(Transform<m, type0> const& t0, Transform<m, type1> const& t1)
{
	return Transform<m, type0>(t0.mat * t1.mat,
	                           t1.invMat * t0.invMat);
}

template <int m, int type> inline Transform<m, type>
inverse(Transform<m, type> const& t) noexcept
{
	return Transform<m, type>(t.invMat, t.mat);
}

} // namespace photino

#endif // !PHOTINO_MATH_TRANSFORM_HPP_
