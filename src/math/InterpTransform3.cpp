#include "InterpTransform3.hpp"

namespace photino
{


BoxAxisAligned<3>
InterpTransform3::motionBounds(Point<3> const& p) const
{
	if (still) return BoxAxisAligned<3>(transform[0]->trPoint(p));

	// TODO: Use another algorithm other than this brute forcing approach
	BoxAxisAligned<3> result;
	constexpr int const nSteps = 128;
	for (int i = 0; i < nSteps; ++i)
	{
		real t = i / (real)(nSteps - 1);
		result |= interpolate01(t).trPoint(p);
	}
	return result;
}

} // namespace photino
