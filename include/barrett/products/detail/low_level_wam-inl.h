/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 */

/*
 * @file low_level_wam-inl.h
 * @date 11/02/2010
 * @author Chris Dellin
 * @author Dan Cody
 *
 */


#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <cstdio>
#include <cassert>

#include <boost/static_assert.hpp>
#include <Eigen/LU>
#include <libconfig.h++>

#include <barrett/os.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/bus/canopen.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/safety_module.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/abstract/multi_puck_product.h>

#include <barrett/products/low_level_wam.h>


namespace barrett {


template<size_t DOF>
const enum Puck::Property LowLevelWam<DOF>::props[] = { Puck::P, Puck::T };


template<size_t DOF>
LowLevelWam<DOF>::LowLevelWam(const std::vector<Puck*>& _pucks, SafetyModule* _safetyModule, const libconfig::Setting& setting, std::vector<int> torqueGroupIds) :
	MultiPuckProduct(DOF, _pucks, PuckGroup::BGRP_WAM, props, sizeof(props)/sizeof(props[0]), "LowLevelWam::LowLevelWam()"),
	safetyModule(_safetyModule), torqueGroups(),
	home(setting["home"]), j2mp(setting["j2mp"]),
	noJointEncoders(true), positionSensor(PS_MOTOR_ENCODER),
	lastUpdate(0.0), torquePropId(group.getPropertyIdNoThrow(Puck::T)),
	hbDivider(0)
{
	logMessage("  Config setting: %s => \"%s\"") % setting.getSourceFile() % setting.getPath();


	// CANopen: no MODE_IDLE broadcast is needed here.  The pucks power
	// up (and are configured by Puck::wake()) in the CiA-402
	// Switch-on-disabled state, and only the safety board's controlword
	// broadcast can enable torque (contract section 7.1).


	// Zero-compensation?
	bool zeroCompensation = setting.exists("zeroangle");
	if (!zeroCompensation) {
		logMessage("  Missing \"zeroangle\" vector: no zero-compensation");
	}


	// Compute motor/joint transforms
	Eigen::FullPivLU<typename sqm_type::Base> lu(j2mp);
	if (!lu.isInvertible()) {
		(logMessage("LowLevelWam::%s(): j2mp matrix is not invertible.")
				% __func__).template raise<std::runtime_error>();
	}
	m2jp = lu.inverse();
	j2mt = m2jp.transpose();


	// Setup torque groups
	// If no IDs are provided, use the defaults: the CANopen torque-quad
	// COB-IDs (contract section 3)
	if (torqueGroupIds.size() == 0) {
		torqueGroupIds.push_back(bus::CANopen::COB_QUAD_A);
		torqueGroupIds.push_back(bus::CANopen::COB_QUAD_B);
	}
	size_t numTorqueGroups = ceil(static_cast<double>(DOF)/MotorPuck::PUCKS_PER_TORQUE_GROUP);
	if (numTorqueGroups > torqueGroupIds.size()) {
		(logMessage("LowLevelWam::%s(): Too few torque group IDs. "
				"Need %d torque groups, only %d IDs provided.")
				% __func__ % numTorqueGroups % torqueGroupIds.size())
				.template raise<std::logic_error>();
	}

	size_t i = 0;
	for (size_t g = 0; g < numTorqueGroups; ++g) {
		std::vector<Puck*> tgPucks;
		while (tgPucks.size() < 4  &&  i < DOF) {
			tgPucks.push_back(pucks[i]);
			++i;
		}
		torqueGroups.push_back(new PuckGroup(torqueGroupIds[g], tgPucks));
	}
	// A constructor that throws from here on (a refused zero, a missing
	// reply) never runs the destructor, so free the groups on the way out.
	struct TorqueGroupGuard {
		std::vector<PuckGroup*>* groups;
		~TorqueGroupGuard() { if (groups != NULL) detail::purge(*groups); }
	} torqueGroupGuard = { &torqueGroups };


	// Compute puck/joint transforms
	// (See DC notebook 1p25)
	v_type cpr;
	for (size_t i = 0; i < DOF; ++i) {
		cpr[i] = motorPucks[i].getCountsPerRad();
	}
	j2pp = cpr.asDiagonal() * j2mp;

	v_type rpc;
	for (size_t i = 0; i < DOF; ++i) {
		rpc[i] = motorPucks[i].getRadsPerCount();
	}
	p2jp = m2jp * rpc.asDiagonal();

	// CANopen/CST torque scale: per-mille of rated torque per Nm
	// replaces IPNM (contract section 5.2)
	v_type ppnm;
	for (size_t i = 0; i < DOF; ++i) {
		ppnm[i] = motorPucks[i].getPermillePerNm();
	}
	j2pt = ppnm.asDiagonal() * j2mt;
	ppOffset.setConstant(0.0);

	// Per-motor signs relative to the BarrettCAN conventions that j2mp
	// encodes.  Measure them only AFTER the pucks' commutation zero is
	// calibrated: bench 2026-09-16, odin 4-DOF, P4 fw 4.3.2, both counts
	// and torque came out mirrored (-1) on all four pucks; the earlier
	// (-1,1,1,1) torque reading was an artifact of a 115-150 degree
	// commutation error on pucks 2-4.  Optional; +1 = as j2mp expects.
	// posSign[i] != trqSign[i] would make every joint loop positive
	// feedback, so that combination is refused below.
	posSign.setConstant(1.0);
	trqSign.setConstant(1.0);
	if (setting.exists("canopen_position_sign")) {
		posSign = v_type(setting["canopen_position_sign"]);
	}
	if (setting.exists("canopen_torque_sign")) {
		trqSign = v_type(setting["canopen_torque_sign"]);
	}
	for (size_t i = 0; i < DOF; ++i) {
		if (fabs(fabs(posSign[i]) - 1.0) > 1e-9 || fabs(fabs(trqSign[i]) - 1.0) > 1e-9) {
			(logMessage("LowLevelWam::%s(): canopen_position_sign / canopen_torque_sign "
					"entries must be +1 or -1.") % __func__).template raise<std::runtime_error>();
		}
		if (posSign[i] != trqSign[i]) {
			(logMessage("LowLevelWam::%s(): motor %d has canopen_position_sign %+.0f but "
					"canopen_torque_sign %+.0f. Opposite signs make its joint loop positive "
					"feedback; that reading means the puck's commutation zero is wrong "
					"(calibrate it, then re-measure), not that the signs differ.")
					% __func__ % (int)(i + 1) % posSign[i] % trqSign[i]).template raise<std::runtime_error>();
		}
	}


	// Zero the WAM?
	if (safetyModule == NULL) {
		logMessage("  No safetyModule: WAM might not be zeroed");
	} else if (safetyModule->wamIsZeroed()) {
		if (safetyModule->hasZeroRecord()) {
			restoreZero();
		} else {
			logMessage("  WAM was already zeroed");
		}
	} else if (safetyModule->zeroWasKept()) {
		// The operator was told the zero would be kept, so the WAM is not
		// necessarily at home: zeroing it here would be silently wrong.
		(logMessage("LowLevelWam::%s(): the WAM zero kept from the previous "
				"run was lost during startup (a puck was reset?). Restart "
				"the program to zero the WAM at its home position.")
				% __func__).template raise<std::runtime_error>();
	} else if (zeroCompensation) {
		// Barrett's zero compensation ("rough placement"): each P4's
		// single-turn encoder (0x3012:01) is absolute within one motor turn,
		// so the WAM only has to be within half a motor turn of home.
		// zeroangle: each motor's encoder angle with its joints at zero, as
		// the puck reports it.  zeroangle_dir: how that encoder counts
		// relative to the puck's position counter; bt-wam-zerocal measures
		// it, and a motor without it (0) is zeroed at home as before.  CANopen
		// pucks report no BarrettCAN ROLE, so the P3 test for an absolute
		// encoder (ROLE option RO_MagEncOnSerial) does not apply.
		v_type zeroAngle(setting["zeroangle"]);
		v_type zeroAngleDir(0.0);
		if (setting.exists("zeroangle_dir")) {
			zeroAngleDir = v_type(setting["zeroangle_dir"]);
		}
		const v_type homeMotor = j2mp * home;
		v_type errorAngle(0.0);  // per motor: reading expected at home minus actual
		std::vector<bool> snapped(DOF, false);
		std::vector<int> marginal;
		for (size_t i = 0; i < DOF; ++i) {
			const int id = pucks[i]->getId();
			if (zeroAngleDir[i] != 1.0  &&  zeroAngleDir[i] != -1.0) {
				logMessage("  No zero-compensation for Puck %d: encoder direction not measured") % id;
				continue;
			}
			if (zeroAngle[i] >= 2*M_PI  ||  zeroAngle[i] < 0) {
				logMessage("  No zero-compensation for Puck %d: bad calibration data") % id;
				continue;
			}
			int mech = 0;
			if (bus::CANopen::sdoUpload(bus, id, bus::CANopen::OD_RAW_POSITION, 1, &mech) != 0) {
				(logMessage("LowLevelWam::%s(): no reply reading the single-turn "
						"encoder of Puck %d; cannot zero.") % __func__ % id)
						.template raise<std::runtime_error>();
			}
			// dir: that encoder's direction relative to this library's motor angles
			const double dir = zeroAngleDir[i] * posSign[i];
			double error = zeroAngle[i] + dir*homeMotor[i] - motorPucks[i].counts2rad(mech);
			error -= 2*M_PI * std::floor((error + M_PI) / (2*M_PI));  // [-pi, pi)
			errorAngle[i] = dir * error;
			snapped[i] = true;
			if (std::fabs(error) > 0.6*M_PI) {
				marginal.push_back(id);
			}
		}

		const jp_type zeroed(home - m2jp*errorAngle);
		definePosition(zeroed);
		if (std::find(snapped.begin(), snapped.end(), true) == snapped.end()) {
			logMessage("  WAM zeroed without zero-compensation");
		} else {
			// Per joint: how far from home the WAM was ("--": none of the
			// joint's motors has encoder data, so it was taken to be at home).
			const double tolerance = m2jp.cwiseAbs().maxCoeff() * 1e-5;
			std::string offsets;
			char text[32];
			for (size_t j = 0; j < DOF; ++j) {
				bool known = false;
				for (size_t i = 0; i < DOF; ++i) {
					known = known || (snapped[i]  &&  std::fabs(m2jp(j,i)) > tolerance);
				}
				if (known) {
					snprintf(text, sizeof(text), "%sJ%d %+.2f", j ? ", " : "", (int)j + 1,
							(zeroed[j] - home[j]) * 180.0/M_PI);
				} else {
					snprintf(text, sizeof(text), "%sJ%d --", j ? ", " : "", (int)j + 1);
				}
				offsets += text;
			}
			logMessage("  WAM zeroed from the single-turn encoders; it sat this far "
					"from home (deg): %s", true) % offsets;
		}
		for (size_t k = 0; k < marginal.size(); ++k) {
			logMessage("  WARNING: Puck %d read more than 60%% of half a motor turn "
					"from home. If the WAM was that far off, its zero may be a whole "
					"motor turn wrong: check the joint angles, or E-Stop, Shift-Idle "
					"and restart with the WAM closer to home.", true) % marginal[k];
		}
	} else {
		definePosition(home);
		logMessage("  WAM zeroed without zero-compensation");
	}


	// Joint encoders
	setPositionSensor(PS_MOTOR_ENCODER);  // Use motor encoders by default
	int numJe = 0;
	int numInitializedJe = 0;

	for (size_t i = 0; i < DOF; ++i) {
		if (pucks[i]->hasOption(Puck::RO_OpticalEncOnEnc)) {
			++numJe;
			if (motorPucks[i].foundIndexPulse()) {
				++numInitializedJe;
			}
		}
	}

	// If there are joint encoders, look up counts per revolution from the config file
	if (numJe != 0) {
		noJointEncoders = false;
		logMessage("  Found %d joint encoders (%d are initialized)") % numJe % numInitializedJe;

		v_type jointEncoderCpr(setting["joint_encoder_counts"]);
		for (size_t i = 0; i < DOF; ++i) {
			if (jointEncoderCpr[i] == 0.0) {
				jointEncoder2jp[i] = 0.0;
			} else {
				jointEncoder2jp[i] = 2*M_PI / jointEncoderCpr[i];
			}
		}
	}


	// Prevent velocity faults on startup due to WAM motion while no program was running.
	if (safetyModule != NULL) {
		safetyModule->ignoreNextVelocityFault();
	}


	// Get good initial values for jp_1 and lastUpdate
	update();
	jv_best.setZero();
	torqueGroupGuard.groups = NULL;  // constructed: the destructor owns them now
}

template<size_t DOF>
LowLevelWam<DOF>::~LowLevelWam()
{
	detail::purge(torqueGroups);
}


template<size_t DOF>
inline const typename LowLevelWam<DOF>::jp_type& LowLevelWam<DOF>::getJointPositions(enum PositionSensor sensor) const {
	switch (sensor) {
	case PS_MOTOR_ENCODER:
		return jp_motorEncoder;
		break;
	case PS_JOINT_ENCODER:
		return jp_jointEncoder;
		break;
	default:
		return jp_best;
	}
}

template<size_t DOF>
void LowLevelWam<DOF>::setPositionSensor(enum PositionSensor sensor)
{
	switch (sensor) {
	case PS_MOTOR_ENCODER:
		useJointEncoder.assign(false);
		break;
	case PS_JOINT_ENCODER:
		if ( !hasJointEncoders() ) {
			(logMessage("LowLevelWam::%s: This WAM is not equipped with joint encoders.")
					% __func__).template raise<std::logic_error>();
		}
		for (size_t i = 0; i < DOF; ++i) {
			if (pucks[i]->hasOption(Puck::RO_OpticalEncOnEnc)  &&  motorPucks[i].foundIndexPulse()) {
				useJointEncoder[i] = true;
			} else {
				useJointEncoder[i] = false;
			}
		}
		break;
	default:
		(logMessage("LowLevelWam::%s: Bad sensor value: %d")
				% __func__ % sensor).template raise<std::logic_error>();
		break;
	}

	positionSensor = sensor;
}


template<size_t DOF>
void LowLevelWam<DOF>::update()
{
	double now = highResolutionSystemTime();

	// CANopen execution cycle (contract sections 7.2/10): emit one SYNC
	// at the top of the cycle - the pucks apply the previously received
	// torque quads and transmit their position TPDO1s - then collect
	// the DOF positions.  The host heartbeat (0x77F, the SB's
	// IDLE-state host-liveness signal) rides the same cycle at 250 ms.
	// Joint encoders are out of the CANopen port's scope (contract
	// section 1), so the old secondary-encoder path is gone.
	{
		BARRETT_SCOPED_LOCK(bus.getMutex());

		// A position TPDO still buffered now is left over from an earlier
		// SYNC (a lost cycle, a duplicate frame).  Consumed one per cycle,
		// it would keep that puck a cycle behind the others for good.
		unsigned char data[bus::CommunicationsBus::MAX_MESSAGE_LEN];
		size_t len;
		for (size_t i = 0; i < DOF; ++i) {
			const int busId = bus::CANopen::COB_TPDO1_BASE + pucks[i]->getId();
			for (int n = 0; n < 16  &&  bus.receive(busId, data, len, false) == 0; ++n) {
			}
		}

		if (bus::CANopen::sync(bus) != 0) {
			(logMessage("LowLevelWam::%s(): could not send SYNC.") % __func__)
					.template raise<std::runtime_error>();
		}
		if (++hbDivider >= 125) {  // 250 ms at the 500 Hz cycle rate
			hbDivider = 0;
			bus::CANopen::hostHeartbeat(bus);
		}

		for (size_t i = 0; i < DOF; ++i) {
			int busId = bus::CANopen::COB_TPDO1_BASE + pucks[i]->getId();
			int ret = bus.receive(busId, data, len, true, true);
			if (ret != 0) {
				(logMessage("LowLevelWam::%s(): Failed to receive position "
						"TPDO for Puck %d: receive() returned %d.")
						% __func__ % pucks[i]->getId() % ret)
						.template raise<std::runtime_error>();
			}
			double counts;
			if (MotorPuck::MotorPositionParser<double>::parse(
					pucks[i]->getId(), 0, &counts, data, len) != 0) {
				(logMessage("LowLevelWam::%s(): Malformed position TPDO "
						"from Puck %d.") % __func__ % pucks[i]->getId())
						.template raise<std::runtime_error>();
			}
			pp[i] = posSign[i] * counts + ppOffset[i];
		}
		// Under the lock: definePosition() rewrites pp from another thread.
		jp_motorEncoder = p2jp * pp;  // Convert from Puck positions to joint positions
	}

	jp_best = jp_motorEncoder;

	jv_best = (jp_best - jp_best_1) / (now - lastUpdate);
	// TODO(dc): Detect unreasonably large velocities

	jp_best_1 = jp_best;
	lastUpdate = now;
}

template<size_t DOF>
void LowLevelWam<DOF>::setTorques(const jt_type& jt)
{
	// Get around C++ address-of-static-member weirdness...
	static const size_t PUCKS_PER_TORQUE_GROUP = MotorPuck::PUCKS_PER_TORQUE_GROUP;

	// Remove sensor-only generalized torques BEFORE the transmission. Merely
	// zeroing a motor row can leak a passive-axis torque into coupled motors.
	jt_type activeTorques(jt);
	for (size_t j = 0; j < DOF; ++j) {
		if (bus::CANopen::isSensorOnly(bus, pucks[j]->getId())) activeTorques[j] = 0.0;
	}
	pt = j2pt * activeTorques;
	pt = trqSign.asDiagonal() * pt;  // CANopen per-motor torque direction (config)
	for (size_t j = 0; j < DOF; ++j) {
		if (bus::CANopen::isSensorOnly(bus, pucks[j]->getId())) pt[j] = 0.0;
	}

	size_t i = 0;
	BARRETT_SCOPED_LOCK(bus.getMutex());
	for (size_t g = 0; g < torqueGroups.size(); ++g) {
		MotorPuck::sendPackedTorques(bus, torqueGroups[g]->getId(), torquePropId, pt.data()+i, std::min(PUCKS_PER_TORQUE_GROUP, DOF-i));
		i += PUCKS_PER_TORQUE_GROUP;
	}
}

template<size_t DOF>
void LowLevelWam<DOF>::definePosition(const jp_type& jp)
{
	// Tell the safety logic to ignore the next velocity fault.  (A P3's
	// position register jumped here; the CANopen board never sees the
	// software zero, so this only skips one of its checks.)
	if (safetyModule != NULL) {
		safetyModule->ignoreNextVelocityFault();
	}

	// Read SDOs without excluding cyclic CAN traffic; applyZero() then
	// commits all offsets together. A failed read leaves the existing
	// zero intact.
	v_type rawPosition;
	for (size_t i = 0; i < DOF; ++i) {
		int raw = 0;
		if (bus::CANopen::sdoUpload(bus, pucks[i]->getId(),
				bus::CANopen::OD_POSITION, 0, &raw) != 0) {
			(logMessage("LowLevelWam::%s(): no position reply from "
					"Puck %d; cannot zero.") % __func__ % pucks[i]->getId())
					.template raise<std::runtime_error>();
		}
		rawPosition[i] = raw;
	}
	applyZero(jp, rawPosition);

	// The safety board is deliberately NOT told the WAM is zeroed: it
	// sees the pucks' raw (reset-relative) counts, so its Cartesian
	// velocity model would be wrong.  With ZERO left at 0 it applies its
	// per-joint velocity limits, which depend only on differences.  The
	// zero is parked on the board instead, for the next run.
	if (safetyModule != NULL  &&  safetyModule->hasZeroRecord()) {
		recordZero(jp, rawPosition);
	}
}

template<size_t DOF>
double LowLevelWam<DOF>::singleTurnAtZero(size_t motor, double reading, const jp_type& jp, int direction) const
{
	const v_type motorAngle = j2mp * jp;
	double angle = reading - direction * posSign[motor] * motorAngle[motor];
	angle -= 2*M_PI * std::floor(angle / (2*M_PI));
	return (angle < 2*M_PI) ? angle : 0.0;  // rounding can land on 2pi itself
}

template<size_t DOF>
int LowLevelWam<DOF>::nodeMask() const
{
	int mask = 0;
	for (size_t i = 0; i < DOF; ++i) {
		mask |= 1 << pucks[i]->getId();
	}
	return mask;
}

// P4 0x6064 restarts at 0 on NMT reset and 0x3013:02 does not move it, so
// the zero is a software offset per puck: raw counts rawPosition mean jp.
template<size_t DOF>
void LowLevelWam<DOF>::applyZero(const jp_type& jp, const v_type& rawPosition)
{
	v_type target = j2pp * jp;  // Convert from joint positions to Puck positions

	// Commit all offsets together under the execution-cycle's I/O lock.
	BARRETT_SCOPED_LOCK(bus.getMutex());
	for (size_t i = 0; i < DOF; ++i) {
		ppOffset[i] = floor(target[i]) - posSign[i] * rawPosition[i];
		pp[i] = posSign[i] * rawPosition[i] + ppOffset[i];
	}
}

// Park the zero on the safety board so that the next program run can keep
// it (bus::CANopen::ZeroRecord).  Best effort: without it the next run just
// asks for the home position again.
template<size_t DOF>
void LowLevelWam<DOF>::recordZero(const jp_type& jp, const v_type& rawPosition)
{
	try {
		bus::CANopen::ZeroRecord record = {};
		record.nodeMask = nodeMask();
		for (size_t i = 0; i < DOF; ++i) {
			const int node = pucks[i]->getId();
			int mech = 0;
			if (bus::CANopen::sdoUpload(bus, node, bus::CANopen::OD_RAW_POSITION, 1, &mech) != 0) {
				throw std::runtime_error("no reply reading the single-turn encoder");
			}
			record.jp[node - 1] = jp[i];
			record.raw[node - 1] = (int)rawPosition[i];
			record.mech[node - 1] = mech;
		}
		bus::CANopen::writeZeroRecord(bus, record);
	} catch (const std::exception& e) {
		logMessage("LowLevelWam::%s(): the WAM zero cannot be kept for the "
				"next run: %s", true) % __func__ % e.what();
	}
}

template<size_t DOF>
void LowLevelWam<DOF>::restoreZero()
{
	bus::CANopen::ZeroRecord record;
	std::string why;
	if ( !bus::CANopen::readZeroRecord(bus, nodeMask(), &record, &why) ) {
		(logMessage("LowLevelWam::%s(): cannot use the WAM zero kept from the "
				"previous run (%s). Restart the program to zero the WAM at its "
				"home position.") % __func__ % why).template raise<std::runtime_error>();
	}
	jp_type jp;
	v_type rawPosition;
	for (size_t i = 0; i < DOF; ++i) {
		jp[i] = record.jp[pucks[i]->getId() - 1];
		rawPosition[i] = record.raw[pucks[i]->getId() - 1];
	}
	applyZero(jp, rawPosition);
	logMessage("  WAM zero kept from the previous run");
}


}
