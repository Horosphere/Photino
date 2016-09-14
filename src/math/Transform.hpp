#ifndef PHOTINO_MATH_TRANSFORM_HPP_
#define PHOTINO_MATH_TRANSFORM_HPP_

#include "geometry.hpp"
#include "RayDifferential.hpp"

namespace photino
{

template <int dim, int type>
class Transform final
{
public:
	static Transform<dim, type> identity();

	Transform() noexcept {}
	/**
	 * @warning Result undefined if the given matrix is singular.
	 */
	Transform(Matrix<dim> const&);
	/**
	 * @warning Result undefined if the given matrix is singular.
	 */
	Transform(Matrix<dim + 1> const&);
	/**
	 * @warning The two matrices must be inverse of each other.
	 */
	Transform(Matrix<dim> const&, Matrix<dim> const&) noexcept;
	/**
	 * @warning The two matrices must be inverse of each other.
	 */
	Transform(Matrix<dim + 1> const&, Matrix<dim + 1> const&) noexcept;

	Matrix<dim> linear() const;
	Matrix<dim> inverseLinear() const;
	Vector<dim> translation() const;

	bool swapsChirality() const;

	Transform<dim, type>& translate(Vector<dim> const&);
	Transform<dim, type>& scale(Vector<dim> const&);
	template <typename Rotation> Transform<dim, type>&
	rotate(Rotation const&);
	Transform<dim, type>& operator*=(Transform<dim, type> const&);

	Point<dim> trPoint(Point<dim> const&) const;
	Vector<dim> trVector(Vector<dim> const&) const;
	Normal<dim> trNormal(Normal<dim> const&) const;
	Ray<dim> trRay(Ray<dim> const&) const;
	RayDifferential<dim> trRayD(RayDifferential<dim> const&) const;
	BoxAxisAligned<dim> trBoxAA(BoxAxisAligned<dim> const&) const;

private:
	/**
	 * The transforms must be inverses of each other.
	 */
	Transform(Eigen::Transform<real, dim, type> const&,
	          Eigen::Transform<real, dim, type> const&);

	Eigen::Transform<real, dim, type> mat;
	Eigen::Transform<real, dim, type> invMat;

	template <int n, int ty>
	friend Transform<n, ty> inverse(Transform<n, ty> const&) noexcept;
	template <int n, int ty> friend bool
	operator==(Transform<n, ty> const&, Transform<n, ty> const&);
	template <int n, int ty> friend bool
	operator!=(Transform<n, ty> const&, Transform<n, ty> const&);
	template <int n, int ty0, int ty1> friend Transform<dim, ty0>
	operator*(Transform<dim, ty0> const&, Transform<dim, ty1> const&);
};


template <int dim, int type> bool
operator==(Transform<dim, type> const&, Transform<dim, type> const&);
template <int dim, int type> bool
operator!=(Transform<dim, type> const&, Transform<dim, type> const&);

template <int dim> Transform<dim, Eigen::AffineCompact>
operator*(Transform<dim, Eigen::AffineCompact> const&,
          Transform<dim, Eigen::AffineCompact> const&);
template <int dim> Transform<dim, Eigen::Projective>
operator*(Transform<dim, Eigen::Projective> const&,
          Transform<dim, Eigen::AffineCompact> const&);
template <int dim> Transform<dim, Eigen::Projective>
operator*(Transform<dim, Eigen::AffineCompact> const&,
          Transform<dim, Eigen::Projective> const&);
template <int dim> Transform<dim, Eigen::Projective>
operator*(Transform<dim, Eigen::Projective> const&,
          Transform<dim, Eigen::Projective> const&);

template <int dim, int type> Transform<dim, type>
inverse(Transform<dim, type> const&) noexcept;


template <int dim>
using TransformAffine = Transform<dim, Eigen::AffineCompact>;
template <int dim>
using TransformProjective = Transform<dim, Eigen::Projective>;

// Implementations

template <int dim, int type> inline Transform<dim, type>
Transform<dim, type>::identity()
{
	Transform<dim, type> result;
	result.invMat = result.mat = Eigen::Transform<real, dim, type>::Identity();
	return result;
}

template <int dim, int type> inline
Transform<dim, type>::Transform(Matrix<dim> const& m):
	mat(m), invMat(m.inverse().eval())
{
}
template <int dim, int type> inline
Transform<dim, type>::Transform(Matrix<dim + 1> const& m):
	mat(m), invMat(m.inverse().eval())
{
}
template <int dim, int type> inline
Transform<dim, type>::Transform(Matrix<dim> const& m,
                              Matrix<dim> const& invM) noexcept:
	mat(m), invMat(invM)
{
}
template <int dim, int type> inline
Transform<dim, type>::Transform(Matrix<dim + 1> const& m,
                              Matrix<dim + 1> const& invM) noexcept:
	mat(m), invMat(invM)
{
}
template <int dim, int type> inline Matrix<dim>
Transform<dim, type>::linear() const
{
	return mat.linear();
}
template <int dim, int type> inline Matrix<dim>
Transform<dim, type>::inverseLinear() const
{
	return invMat.linear();
}
template <int dim, int type> inline Vector<dim>
Transform<dim, type>::translation() const
{
	return mat.translation();
}
template <int dim, int type> inline bool
Transform<dim, type>::swapsChirality() const
{
	return mat.linear().determinant() < 0;
}

template <int dim, int type> inline Transform<dim, type>&
Transform<dim, type>::translate(Vector<dim> const& v)
{
	mat.translate(v);
	invMat.translate(-v);
	return *this;
}
template <int dim, int type> inline Transform<dim, type>&
Transform<dim, type>::scale(Vector<dim> const& v)
{
	mat.scale(v);
	invMat.scale(v.cwiseInverse());
	return *this;
}
template <int dim, int type>
template <typename Rotation> inline Transform<dim, type>&
Transform<dim, type>::rotate(Rotation const& rotation)
{
	Matrix<dim> rotationMatrix = rotation.toRotationMatrix();
	mat = mat * Eigen::Transform<real, dim, Eigen::Affine>(rotationMatrix);
	invMat = Eigen::Transform<real, dim, Eigen::Affine>(rotationMatrix.transpose()) * invMat;
	return *this;
}
template <int dim, int type> inline Transform<dim, type>&
Transform<dim, type>::operator*=(Transform<dim, type> const& t)
{
	mat = mat * t.mat;
	invMat = t.invMat * invMat;
	return *this;
}

template <int dim, int type> Point<dim>
Transform<dim, type>::trPoint(Point<dim> const& p) const
{
	return mat * p;
}
template <int dim, int type> inline Vector<dim>
Transform<dim, type>::trVector(Vector<dim> const& v) const
{
	return mat.linear() * v;
}
template <int dim, int type> inline Normal<dim>
Transform<dim, type>::trNormal(Normal<dim> const& n) const
{
	return invMat.linear().transpose() * n;
}
template <int dim, int type> inline Ray<dim>
Transform<dim, type>::trRay(Ray<dim> const& r) const
{
	return Ray<dim>(trPoint(r.origin()), trVector(r.direction()));
}
template <int dim, int type> inline RayDifferential<dim>
Transform<dim, type>::trRayD(RayDifferential<dim> const& rd) const
{
	return RayDifferential<dim>(trRay(rd.rx), trRay(rd.ry));
}
template <int dim, int type> inline BoxAxisAligned<dim>
Transform<dim, type>::trBoxAA(BoxAxisAligned<dim> const& b) const
{
	assert(dim == 3 && "Not implemented for dimensions other than 3");
	BoxAxisAligned<dim> result(trPoint(b.corner(BoxAxisAligned<dim>::BottomLeftFloor)));
	result |= trPoint(b.corner(BoxAxisAligned<dim>::BottomLeftCeil));
	result |= trPoint(b.corner(BoxAxisAligned<dim>::BottomRightFloor));
	result |= trPoint(b.corner(BoxAxisAligned<dim>::BottomRightCeil));
	result |= trPoint(b.corner(BoxAxisAligned<dim>::TopLeftFloor));
	result |= trPoint(b.corner(BoxAxisAligned<dim>::TopLeftCeil));
	result |= trPoint(b.corner(BoxAxisAligned<dim>::TopRightFloor));
	result |= trPoint(b.corner(BoxAxisAligned<dim>::TopRightCeil));
	return result;
}

template <int dim, int type> inline
Transform<dim, type>::Transform(Eigen::Transform<real, dim, type> const& mat,
                              Eigen::Transform<real, dim, type> const& invMat):
	mat(mat), invMat(invMat)
{
}

template <int dim, int type> inline bool
operator==(Transform<dim, type> const& t0, Transform<dim, type> const& t1)
{
	return t0.mat == t1.mat;
}
template <int dim, int type> bool
operator!=(Transform<dim, type> const& t0, Transform<dim, type> const& t1)
{
	return t0.mat != t1.mat;
}
template <int dim> Transform<dim, Eigen::AffineCompact>
operator*(Transform<dim, Eigen::AffineCompact> const& t0,
          Transform<dim, Eigen::AffineCompact> const& t1)
{
	return Transform<dim, Eigen::AffineCompact>(t0.mat * t1.mat,
	       t1.invMat * t0.invMat);
}
template <int dim> Transform<dim, Eigen::Projective>
operator*(Transform<dim, Eigen::Projective> const& t0,
          Transform<dim, Eigen::AffineCompact> const& t1)
{
	return Transform<dim, Eigen::Projective>(t0.mat * t1.mat,
	                                       t1.invMat * t0.invMat);
}
template <int dim> Transform<dim, Eigen::Projective>
operator*(Transform<dim, Eigen::AffineCompact> const& t0,
          Transform<dim, Eigen::Projective> const& t1)
{
	return Transform<dim, Eigen::Projective>(t0.mat * t1.mat,
	                                       t1.invMat * t0.invMat);
}
template <int dim> Transform<dim, Eigen::Projective>
operator*(Transform<dim, Eigen::Projective> const& t0,
          Transform<dim, Eigen::Projective> const& t1)
{
	return Transform<dim, Eigen::Projective>(t0.mat * t1.mat,
	                                       t1.invMat * t0.invMat);
}

template <int dim, int type> inline Transform<dim, type>
inverse(Transform<dim, type> const& t) noexcept
{
	return Transform<dim, type>(t.invMat, t.mat);
}

} // namespace photino

#endif // !PHOTINO_MATH_TRANSFORM_HPP_
