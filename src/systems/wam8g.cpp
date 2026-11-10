#include <cmath>
#include <stdexcept>
#include <barrett/bus/canopen.h>
#include <barrett/systems/wam8g.h>

namespace barrett {
namespace systems {

const libconfig::Setting& Wam8g::validate(const std::vector<Puck*>& pucks,
		const libconfig::Setting& setting)
{
	if (pucks.size() != MEASURED_DOF || pucks[0] == NULL ||
			bus::CANopen::getSensorOnlyNodes(pucks[0]->getBus()) != (1 << 5)) {
		throw std::invalid_argument("wam8g requires nodes 1-8 and sensor-only node 5; call ProductManager::wakeAllPucks() first.");
	}
	for (size_t i = 0; i < MEASURED_DOF; ++i) {
		if (pucks[i] == NULL || pucks[i]->getId() != int(i + 1) ||
				&pucks[i]->getBus() != &pucks[0]->getBus()) {
			throw std::invalid_argument("wam8g requires ordered puck nodes 1-8.");
		}
	}
	jp_type home(setting["low_level"]["home"]);
	const libconfig::Setting& moments=setting["gravity_compensation"]["mus"];
	if (moments.getLength() != MEASURED_DOF) {
		throw std::invalid_argument("wam8g gravity calibration requires eight link first moments.");
	}
	for (size_t i = 0; i < MEASURED_DOF; ++i) {
		math::Vector<3>::type mu(moments[int(i)]);
		if (!std::isfinite(home[i]) || !mu.allFinite()) {
			throw std::invalid_argument("wam8g home and gravity parameters must be finite.");
		}
	}
	// A motorless sensor must have its own coordinate. Reject a transmission
	// that would couple J5 into an active motor (or vice versa).
	math::Matrix<8, 8> transmission(setting["low_level"]["j2mp"]);
	for (size_t i = 0; i < MEASURED_DOF; ++i) {
		for (size_t j = 0; j < MEASURED_DOF; ++j) {
			if (!std::isfinite(transmission(i,j)) ||
					((i == PASSIVE_INDEX) != (j == PASSIVE_INDEX) && transmission(i,j) != 0.0)) {
				throw std::invalid_argument("wam8g j2mp must be finite and must not couple passive J5 to active motors.");
			}
		}
	}
	Eigen::FullPivLU<math::Matrix<8,8>::Base> lu(transmission);
	if (!lu.isInvertible()) throw std::invalid_argument("wam8g j2mp must be invertible.");
	// Enforce this even in custom profiles, rather than relying on zero gains
	// in the shipped template. The final low-level command also masks J5.
	const char* gains[] = {"kp", "ki", "kd"};
	for (size_t g = 0; g < 3; ++g) {
		math::Vector<8>::type positionGains(setting["joint_position_control"][gains[g]]);
		math::Vector<8>::type velocityGains(setting["joint_velocity_control"][0][gains[g]]);
		if (positionGains[PASSIVE_INDEX] != 0.0 || velocityGains[PASSIVE_INDEX] != 0.0) {
			throw std::invalid_argument("wam8g J5 position and velocity controller gains must be zero.");
		}
	}
	return setting;
}

Wam8g::jp_type Wam8g::expandPosition(const reference_type& reference)
{
	return Map::expand<jp_type>(boost::get<0>(reference), boost::get<1>(reference)[PASSIVE_INDEX]);
}

Wam8g::Wam8g(ExecutionManager* em, const std::vector<Puck*>& pucks,
		SafetyModule* safety, const libconfig::Setting& setting) :
	Wam<8>(em, pucks, safety, validate(pucks, setting), std::vector<int>(), "Wam8g"),
	selectPosition(&Map::select<active_jp_type, jp_type>),
	selectVelocity(&Map::select<active_jv_type, jv_type>),
	expandTorque([](const active_jt_type& t) { return Map::expand<jt_type>(t); }),
	expandTorqueReference([](const active_jt_type& t) { return Map::expand<jt_type>(t); }),
	expandVelocityReference([](const active_jv_type& v) { return Map::expand<jv_type>(v); }),
	positionReference(), expandPositionReference(&Wam8g::expandPosition),
	forceToTorque(), momentToTorque(), externalTorques(true),
	kinematicsCalibrated(setting.exists("kinematics_calibrated") && bool(setting["kinematics_calibrated"])),
	gravityCalibrated(setting.exists("gravity_calibrated") && bool(setting["gravity_calibrated"])),
	activeInput(expandTorque.input),
	activeJpOutput(selectPosition.output), activeJvOutput(selectVelocity.output)
{
	connect(jpOutput, selectPosition.input);
	connect(jvOutput, selectVelocity.input);
	connect(jpOutput, positionReference.getInput<1>());
	connect(positionReference.output, expandPositionReference.input);
	connect(kinematicsBase.kinOutput, forceToTorque.kinInput);
	connect(kinematicsBase.kinOutput, momentToTorque.kinInput);
	connect(expandTorque.output, externalTorques.getInput(0));
	connect(forceToTorque.output, externalTorques.getInput(1));
	connect(momentToTorque.output, externalTorques.getInput(2));
	// Into the reserved summer slot, NOT into Wam<8>::input: 'input' stays
	// free for applications written against Wam<DOF> (opus 2026-09-22:
	// lbc_torque_sign asserted on an already-connected input).
	connect(externalTorques.output, jtSum.getInput(EXT_INPUT));
}

Wam8g::active_jp_type Wam8g::getActiveJointPositions() const { return Map::select<active_jp_type>(getJointPositions()); }
Wam8g::active_jv_type Wam8g::getActiveJointVelocities() const { return Map::select<active_jv_type>(getJointVelocities()); }
Wam8g::active_jp_type Wam8g::getActiveHomePosition() const { return Map::select<active_jp_type>(getHomePosition()); }
Wam8g::active_jt_type Wam8g::getActiveJointTorques() const { return Map::select<active_jt_type>(getJointTorques()); }

void Wam8g::trackReferenceSignal(System::Output<active_jp_type>& reference) {
	// Route through the eight-wide reference every time, as the velocity
	// and torque overloads do: any eight-wide move (moveTo, moveHome) takes
	// over jpController's reference input, so a one-time wire would leave a
	// later seven-wide reference silently ignored.
	BARRETT_SCOPED_LOCK(getEmMutex());
	forceConnect(reference, positionReference.getInput<0>());
	Wam<8>::trackReferenceSignal(expandPositionReference.output);
}
void Wam8g::trackReferenceSignal(System::Output<active_jv_type>& reference) {
	BARRETT_SCOPED_LOCK(getEmMutex());
	forceConnect(reference, expandVelocityReference.input);
	Wam<8>::trackReferenceSignal(expandVelocityReference.output);
}
void Wam8g::trackReferenceSignal(System::Output<active_jt_type>& reference) {
	BARRETT_SCOPED_LOCK(getEmMutex());
	forceConnect(reference, expandTorqueReference.input);
	Wam<8>::trackReferenceSignal(expandTorqueReference.output);
}

void Wam8g::moveTo(const active_jp_type& destination, bool blocking, double velocity, double acceleration) {
	// Expand to the eight-joint move: J5's target is its current measured
	// angle, which its zero gains ignore anyway.  Wam<8>::moveTo() starts
	// from the current set point when a move is already being tracked, so
	// chained moves are continuous.
	Wam<8>::moveTo(Map::expand<jp_type>(destination, getJointPositions()[PASSIVE_INDEX]),
			blocking, velocity, acceleration);
}

void Wam8g::trackReferenceSignal(System::Output<cp_type>& reference) { requireKinematics(); Wam<8>::trackReferenceSignal(reference); }
void Wam8g::trackReferenceSignal(System::Output<Eigen::Quaterniond>& reference) { requireKinematics(); Wam<8>::trackReferenceSignal(reference); }
void Wam8g::trackReferenceSignal(System::Output<pose_type>& reference) { requireKinematics(); Wam<8>::trackReferenceSignal(reference); }
void Wam8g::moveTo(const cp_type& destination, bool blocking, double velocity, double acceleration) {
	requireKinematics(); Wam<8>::moveTo(destination, blocking, velocity, acceleration);
}
void Wam8g::moveTo(const Eigen::Quaterniond& destination, bool blocking, double velocity, double acceleration) {
	requireKinematics(); Wam<8>::moveTo(destination, blocking, velocity, acceleration);
}
void Wam8g::moveTo(const pose_type& destination, bool blocking, double velocity, double acceleration) {
	requireKinematics(); Wam<8>::moveTo(destination, blocking, velocity, acceleration);
}

void Wam8g::requireKinematics() const {
	if (!kinematicsCalibrated) throw std::logic_error("wam8g: supply measured J5-J8/tool geometry and set kinematics_calibrated before using Cartesian control.");
}
Wam8g::cp_type Wam8g::getToolPosition() const { requireKinematics(); return Wam<8>::getToolPosition(); }
Wam8g::cv_type Wam8g::getToolVelocity() const { requireKinematics(); return Wam<8>::getToolVelocity(); }
Eigen::Quaterniond Wam8g::getToolOrientation() const { requireKinematics(); return Wam<8>::getToolOrientation(); }
Wam8g::pose_type Wam8g::getToolPose() const { requireKinematics(); return Wam<8>::getToolPose(); }
math::Matrix<6, 8> Wam8g::getToolJacobian() const { requireKinematics(); return Wam<8>::getToolJacobian(); }
math::Matrix<6, 7> Wam8g::getActiveToolJacobian() const { requireKinematics(); BARRETT_SCOPED_LOCK(getEmMutex()); return Map::activeJacobian(Wam<8>::getToolJacobian()); }
System::Input<Wam8g::cf_type>& Wam8g::getToolForceInput() { requireKinematics(); return forceToTorque.input; }
System::Input<Wam8g::ct_type>& Wam8g::getToolTorqueInput() { requireKinematics(); return momentToTorque.input; }
void Wam8g::gravityCompensate(bool compensate) {
	if (compensate) {
		requireKinematics();
		if (!gravityCalibrated) throw std::logic_error("wam8g: supply gravity calibration for the assembled arm/wrist and set gravity_calibrated.");
	}
	Wam<8>::gravityCompensate(compensate);
}

}
}
