#ifndef BARRETT_SYSTEMS_WAM8G_H_
#define BARRETT_SYSTEMS_WAM8G_H_

#include <barrett/math/wam8g.h>
#include <barrett/systems/wam.h>
#include <barrett/systems/callback.h>

namespace barrett {
namespace systems {

/** The 4-DOF WAM plus a passive encoder joint (J5) and the 3-motor Alberta
 * wrist, as ONE eight-joint WAM.
 *
 * Wam8g IS a Wam<8>: every generic application, tool and example that is
 * written against Wam<DOF> (standard_main_function.h dispatches it as
 * DOF = 8) sees eight measured joints J1..J8 and commands eight torques.
 * J5 has no motor: the low-level driver never sends it torque, and its
 * controller gains are zero, so an eight-vector command simply has an
 * inert slot 4.  Nothing else about the product is special.
 *
 * On top of that view, this class adds a seven-joint "active" view for
 * applications that do not want to carry the passive slot: J1,J2,J3,J4,
 * J6,J7,J8.  Those members are all named active*, or take the active
 * (seven-wide) types, so the two views cannot be confused.  J5's measured
 * angle still enters forward kinematics, the Jacobian and gravity.
 */
class Wam8g : public Wam<8> {
public:
	enum { MEASURED_DOF = 8, ACTIVE_DOF = 7, PASSIVE_INDEX = 4 };
	typedef units::JointPositions<7>::type active_jp_type;
	typedef units::JointVelocities<7>::type active_jv_type;
	typedef units::JointTorques<7>::type active_jt_type;

private:
	typedef math::Wam8gJointMap Map;
	typedef boost::tuple<active_jp_type, jp_type> reference_type;
	Callback<jp_type, active_jp_type> selectPosition;
	Callback<jv_type, active_jv_type> selectVelocity;
	Callback<active_jt_type, jt_type> expandTorque;
	Callback<active_jt_type, jt_type> expandTorqueReference;
	Callback<active_jv_type, jv_type> expandVelocityReference;
	TupleGrouper<active_jp_type, jp_type> positionReference;
	Callback<reference_type, jp_type> expandPositionReference;
	ToolForceToJointTorques<8> forceToTorque;
	ToolTorqueToJointTorques<8> momentToTorque;
	Summer<jt_type, 3> externalTorques;
	const bool kinematicsCalibrated;
	const bool gravityCalibrated;

public:
	/** Seven active torques (J1..J4, J6..J8); summed into Wam<8>::input. */
	System::Input<active_jt_type>& activeInput;
	System::Output<active_jp_type>& activeJpOutput;
	System::Output<active_jv_type>& activeJvOutput;

	Wam8g(ExecutionManager* em, const std::vector<Puck*>& pucks,
			SafetyModule* safety, const libconfig::Setting& setting);

	// Eight-wide getters, moveTo(jp_type), trackReferenceSignal(...),
	// moveHome, idle, gravity etc. come from Wam<8>.
	using Wam<8>::moveTo;
	using Wam<8>::trackReferenceSignal;

	active_jp_type getActiveJointPositions() const;
	active_jv_type getActiveJointVelocities() const;
	active_jp_type getActiveHomePosition() const;
	/** Commanded active joint torques, not measured interaction torques. */
	active_jt_type getActiveJointTorques() const;

	void trackReferenceSignal(System::Output<active_jp_type>& reference);
	void trackReferenceSignal(System::Output<active_jv_type>& reference);
	void trackReferenceSignal(System::Output<active_jt_type>& reference);
	/** Moves the seven active joints; J5 keeps its current measured angle
	 *  as its (inert) target.  Continuous with a previous moveTo(). */
	void moveTo(const active_jp_type& destination, bool blocking = true,
			double velocity = 0.5, double acceleration = 0.5);
	// Cartesian entry points, guarded by requireKinematics() (see below).
	void trackReferenceSignal(System::Output<cp_type>& reference);
	void trackReferenceSignal(System::Output<Eigen::Quaterniond>& reference);
	void trackReferenceSignal(System::Output<pose_type>& reference);
	void moveTo(const cp_type& destination, bool blocking = true,
			double velocity = 0.1, double acceleration = 0.2);
	void moveTo(const Eigen::Quaterniond& destination, bool blocking = true,
			double velocity = 0.5, double acceleration = 0.5);
	void moveTo(const pose_type& destination, bool blocking = true,
			double velocity = 0.1, double acceleration = 0.2);

	/** Partial Jacobian: the seven active columns at the measured J5. */
	math::Matrix<6, 7> getActiveToolJacobian() const;
	// Tool-frame getters, guarded like the Cartesian entry points.
	cp_type getToolPosition() const;
	cv_type getToolVelocity() const;
	Eigen::Quaterniond getToolOrientation() const;
	pose_type getToolPose() const;
	math::Matrix<6, 8> getToolJacobian() const;
	/** Wrench at the configured tool origin, world frame, mapped through
	 *  the full eight-joint Jacobian (J5's row is dropped downstream). */
	System::Input<cf_type>& getToolForceInput();
	System::Input<ct_type>& getToolTorqueInput();

	/** Cartesian and gravity use are refused until the assembled robot's
	 *  geometry / gravity model are supplied and the config flags are set
	 *  (through this class; the plain Wam<8> view is not guarded). */
	void gravityCompensate(bool compensate = true);
	bool hasCalibratedKinematics() const { return kinematicsCalibrated; }
	bool hasCalibratedGravity() const { return gravityCalibrated; }
	void requireKinematics() const;

private:
	static const libconfig::Setting& validate(const std::vector<Puck*>& pucks,
			const libconfig::Setting& setting);
	static jp_type expandPosition(const reference_type& reference);
	DISALLOW_COPY_AND_ASSIGN(Wam8g);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
#endif
