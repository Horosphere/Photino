#ifndef PHOTINO_MATH_COORDINATES_HPP_
#define PHOTINO_MATH_COORDINATES_HPP_

#include "geometry.hpp"

namespace photino
{

/**
 * @brief Finds two orthogonal vectors w.r.t. the given vector. It is
 *  guarenteed that v0 x v1 = vIn
 * @param[in] vIn A unit vector
 * @param[out] v0 The first vector that is orthogonal to vIn
 * @param[out] v1 The second vector that is orthogonal to vIn
 */
void getPerpendicular(Vector<3> const& vIn,
                      Vector<3>* const v0, Vector<3>* const v1);
/**
 * @brief Right-handed lookAt matrix
 */
Matrix<4> lookAt(Point<3> const& camera, Point<3> const& focus,
                 Vector<3> const& up);


// Implementations

inline void getPerpendicular(Vector<3> const& vIn,
                             Vector<3>* const v0, Vector<3>* const v1)
{
	if (abs(vIn[0]) > abs(vIn[1]))
	{
		real fac = 1.0 / sqrt(vIn[0] * vIn[0] + vIn[2] * vIn[2]);
		(*v0)[0] = -vIn[2] * fac;
		(*v0)[1] = 0;
		(*v0)[2] = vIn[0] * fac;
	}
	else
	{
		real fac = 1.0 / sqrt(vIn[1] * vIn[1] + vIn[2] * vIn[2]);
		(*v0)[0] = 0;
		(*v0)[1] = vIn[2] * fac;
		(*v0)[2] = -vIn[1] * fac;
	}
	(*v1) = cross(vIn, *v0);
}
inline Matrix<4> lookAt(Point<3> const& camera, Point<3> const& focus,
                        Vector<3> const& up)
{
	Matrix<4> result = Matrix<4>::Identity();
	result.col(3).head(3) = camera;

	Vector<3> z = (camera - focus).normalized();
	Vector<3> x = unit(cross(up, z));
	Vector<3> y = cross(z, x);
	result.col(0).head(3) = x;
	result.col(1).head(3) = y;
	result.col(2).head(3) = z;
	return result;
}

} // namespace photino

#endif // !PHOTINO_MATH_COORDINATES_HPP_
