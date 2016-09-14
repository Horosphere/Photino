#ifndef PHOTINO_MATH_INTERPTRANSFORM3_HPP_
#define PHOTINO_MATH_INTERPTRANSFORM3_HPP_

#include "Transform.hpp"
#include "numbers.hpp"

namespace photino
{

/**
 * This function uses polar decomposition based on Jacobi SVD algorithm. Let
 * linear = M. Then
 *
 * M = UZV
 *
 * The rotation matrix is R = UV*, and the scale matrix is S = VZV*. Hence
 * M = RS.
 *
 * @brief Decomposes a linear operation into a quaternion rotation and a matrix
 *	scale/shear component
 * @param[in] linear A matrix
 * @param[out] rotation Rotation component of the matrix
 * @param[out] scale Scale/Shear component of the matrix
 */
void decomposeLinear(Matrix<3> const& linear,
                     Quaternion* const rotation,
                     Matrix<3>* const scale);
/**
 * @brief Used to interpolate two 3D affine transformations
 */
class InterpTransform3 final
{
public:
	InterpTransform3(TransformAffine<3> const* tr0, real time0,
	                 TransformAffine<3> const* tr1, real time1);

	TransformAffine<3> interpolate01(real t) const;
	TransformAffine<3> interpolate(real time) const;

	Point<3> trPoint(real ti, Point<3> const&) const;
	Vector<3> trVector(real ti, Vector<3> const&) const;
	Normal<3> trNormal(real ti, Normal<3> const&) const;
	Ray<3> trRay(real ti, Ray<3> const&) const;
	RayDifferential<3> trRayD(real ti, RayDifferential<3> const&) const;
	BoxAxisAligned<3> motionBounds(Point<3> const&) const;
	BoxAxisAligned<3> motionBounds(BoxAxisAligned<3> const&) const;

private:
	real const time[2];
	TransformAffine<3> const* const transform[2];
	bool const still;
	Vector<3> translation[2];
	Quaternion rotation[2];
	Matrix<3> scale[2];

};


// Implementations

inline void decomposeLinear(Matrix<3> const& linear,
                            Quaternion* const rotation,
                            Matrix<3>* const scale)
{
	// Use Jacobi SVD (Efficient for small matrices)
	auto svd = linear.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	Matrix<3> u = svd.matrixU();
	Matrix<3> v = svd.matrixV();

	// Convert SVD to polar decomposition
	*scale = v * svd.singularValues().asDiagonal() * v.adjoint();
	*rotation = Quaternion(u * v.adjoint());
}

inline InterpTransform3::InterpTransform3(
  TransformAffine<3> const* tr0, real time0,
  TransformAffine<3> const* tr1, real time1):
	time{time0, time1}, transform{tr0, tr1},
	still(tr0 == tr1)
{
	translation[0] = transform[0]->translation();
	translation[1] = transform[1]->translation();
	decomposeLinear(transform[0]->linear(), &rotation[0], &scale[0]);
	decomposeLinear(transform[1]->linear(), &rotation[1], &scale[1]);
}

inline TransformAffine<3> InterpTransform3::interpolate01(real t) const
{
	if (still || t <= 0)
		return *transform[0];
	if (t >= 1)
		return *transform[1];

	Vector<3> translate = (1 - t) * translation[0] + t * translation[1];
	Quaternion rotate = slerp(t, rotation[0], rotation[1]);
	Matrix<3> scaling = lerp<real, Matrix<3>>(t, scale[0], scale[1]);

	return TransformAffine<3>(scaling).rotate(rotate).translate(translate);
}
inline TransformAffine<3> InterpTransform3::interpolate(real ti) const
{
	if (still || ti <= time[0])
		return *transform[0];
	if (ti >= time[1])
		return *transform[1];

	real t = (ti - time[0]) / (time[1] - time[0]);
	Vector<3> translate = (1 - t) * translation[0] + t * translation[1];
	Quaternion rotate = slerp(t, rotation[0], rotation[1]);
	Matrix<3> scaling = lerp<real, Matrix<3>>(t, scale[0], scale[1]);

	return TransformAffine<3>(scaling).rotate(rotate).translate(translate);
}

inline Point<3>
InterpTransform3::trPoint(real ti, Point<3> const& p) const
{
	if (still || ti < time[0])
		return transform[0]->trPoint(p);
	else if (ti > time[1])
		return transform[1]->trPoint(p);

	return interpolate(ti).trPoint(p);
}
inline Vector<3>
InterpTransform3::trVector(real ti, Vector<3> const& v) const
{
	if (still || ti < time[0])
		return transform[0]->trVector(v);
	else if (ti > time[1])
		return transform[1]->trVector(v);

	return interpolate(ti).trVector(v);
}
inline Normal<3>
InterpTransform3::trNormal(real ti, Normal<3> const& n) const
{
	if (still || ti < time[0])
		return transform[0]->trNormal(n);
	else if (ti > time[1])
		return transform[1]->trNormal(n);

	return interpolate(ti).trNormal(n);
}
inline Ray<3>
InterpTransform3::trRay(real ti, Ray<3> const& r) const
{
	if (still || ti < time[0])
		return transform[0]->trRay(r);
	else if (ti > time[1])
		return transform[1]->trRay(r);

	return interpolate(ti).trRay(r);
}
inline RayDifferential<3>
InterpTransform3::trRayD(real ti, RayDifferential<3> const& rd) const
{
	if (still || ti < time[0])
		return transform[0]->trRayD(rd);
	else if (ti > time[1])
		return transform[1]->trRayD(rd);

	return interpolate(ti).trRayD(rd);
}
inline BoxAxisAligned<3>
InterpTransform3::motionBounds(BoxAxisAligned<3> const& b) const
{
	if (still) return transform[0]->trBoxAA(b);
	BoxAxisAligned<3> result;
	for (unsigned int i = 0; i < 8; ++i)
		result |= motionBounds(cornerOf(b, i));
	return result;
}

} // namespace photino

#endif // !PHOTINO_MATH_INTERPTRANSFORM3_HPP_
