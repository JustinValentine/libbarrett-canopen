#ifndef BARRETT_SYSTEMS_WAM8G_H_
#define BARRETT_SYSTEMS_WAM8G_H_

#include <barrett/math/wam8g.h>
#include <barrett/systems/wam.h>
#include <barrett/systems/callback.h>

namespace barrett {
namespace systems {

/** A 4-DOF WAM with a passive encoder joint (J5) and a three-motor wrist,
 * as one eight-joint WAM.
 *
 * Wam8g is a Wam<8>: code written against Wam<DOF> sees eight measured
 * joints and commands eight torques.  J5 has no motor: its torque is never
 * sent and its controller gains are zero.
 *
 * This class adds a seven-joint "active" view (J1-J4, J6-J8) for
 * applications that do not want to carry the passive slot.  Its members
 * are named active* or take the seven-wide active types.  J5's measured
 * angle still enters the kinematics, the Jacobian and gravity.
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
	/** Moves the seven active joints. */
	void moveTo(const active_jp_type& destination, bool blocking = true,
			double velocity = 0.5, double acceleration = 0.5);
	// The Cartesian interface throws unless kinematics_calibrated is set.
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
	cp_type getToolPosition() const;
	cv_type getToolVelocity() const;
	Eigen::Quaterniond getToolOrientation() const;
	pose_type getToolPose() const;
	math::Matrix<6, 8> getToolJacobian() const;
	/** Wrench at the tool origin, in the world frame. */
	System::Input<cf_type>& getToolForceInput();
	System::Input<ct_type>& getToolTorqueInput();

	/** Throws unless the configuration sets kinematics_calibrated and
	 *  gravity_calibrated. */
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
