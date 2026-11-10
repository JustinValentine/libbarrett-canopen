#ifndef BARRETT_MATH_WAM8G_H_
#define BARRETT_MATH_WAM8G_H_

#include <barrett/units.h>

namespace barrett {
namespace math {

/** Coordinate selection only: J5 is measured, never actuated. No linkage model. */
struct Wam8gJointMap {
	enum { MEASURED_DOF = 8, ACTIVE_DOF = 7, PASSIVE_INDEX = 4 };

	static size_t measuredIndex(size_t activeIndex) {
		return activeIndex < PASSIVE_INDEX ? activeIndex : activeIndex + 1;
	}

	template<class Active, class Measured>
	static Active select(const Measured& measured) {
		Active active;
		for (size_t i = 0; i < ACTIVE_DOF; ++i) active[i] = measured[measuredIndex(i)];
		return active;
	}

	template<class Measured, class Active>
	static Measured expand(const Active& active, double passive = 0.0) {
		Measured measured;
		measured[PASSIVE_INDEX] = passive;
		for (size_t i = 0; i < ACTIVE_DOF; ++i) measured[measuredIndex(i)] = active[i];
		return measured;
	}

	// Partial derivative with J5 held at its latest measured angle.
	static Matrix<6, ACTIVE_DOF> activeJacobian(const Matrix<6, MEASURED_DOF>& full) {
		Matrix<6, ACTIVE_DOF> active;
		for (size_t i = 0; i < ACTIVE_DOF; ++i) active.col(i) = full.col(measuredIndex(i));
		return active;
	}
};

}
}
#endif
