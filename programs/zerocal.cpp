/*
	Copyright 2009, 2010, 2011, 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * zerocal.cpp
 *
 *  Created on: Apr 8, 2010
 *      Author: cd
 *      Author: dc
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cassert>
#include <cmath>
#include <cstring>

#include <syslog.h>

#include <curses.h>
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/os.h>
#include <barrett/exception.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#include <barrett/config.h>

#include "utils.h"


using namespace barrett;
using detail::waitForEnter;

const std::string CAL_CONFIG_FILE = barrett::EtcPathRelative("calibration.conf");
const std::string DATA_CONFIG_FILE = barrett::EtcPathRelative("calibration_data/%s/zerocal.conf");


// Convenience class that wraps ncurses text attributes.
class ScopedAttr {
public:
	ScopedAttr() : isOn(false), a(-1) {}
	explicit ScopedAttr(unsigned long attr, bool startOn = false) : isOn(false), a(-1) {
		set(attr, startOn);
	}
	~ScopedAttr() { off(); }

	void set(unsigned long attr, bool startOn = true) {
		if (isOn) {
			off();
		}
		a = attr;
		if (startOn) {
			on();
		}
	}
	void on() {
		if (!isOn) {
			attron(a);
			isOn = true;
		}
	}
	void off() {
		if (isOn) {
			attroff(a);
			isOn = false;
		}
	}

protected:
	bool isOn;
	unsigned long a;
};


// Represents a generic step in the calibration process.
// Handles displaying a menu item and provides hooks for keyboard interactions
// and state transitions. Can be "selected" and/or "active".
class CalibrationStep {
public:
	explicit CalibrationStep(const std::string& title_ = "") :
		selected(false), active(false), title(title_) {}
	virtual ~CalibrationStep() {}

	void setSelected(bool on) { selected = on; }
	void setActive(bool on) {
		if (active != on) {
			active = on;
			onChangeActive();
		}
	}

	virtual void onChangeActive() {}
	virtual bool onKeyPress(enum Key k) {
		return true;
	}

	static const int L_OFFSET = 10;

	// Returns the number of lines that were displayed
	virtual int display(int top, int left) {
		ScopedAttr sa(A_BOLD);
		if (selected) {
			sa.on();
			mvprintw(top,left, "--> ] ");
		} else {
			mvprintw(top,left, "    ] ");
		}
		printw("%s", title.c_str());

		return height(top);
	}

	// Helper for calculating display heights
	static int height(int origTop) {
		return getcury(stdscr) - origTop + 1;
	}

protected:
	bool selected, active;
	std::string title;
};

// Rough placement at power-up needs to know which way each P4's single-turn
// encoder (0x3012:01) counts relative to its position counter (0x6064).
// Rather than trust a puck setting, watch both: one reading at rest before
// the calibration moves the arm, compared with later readings at rest (each
// joint's [Enter], and the end).
template<size_t DOF>
class SingleTurnSurvey {
public:
	explicit SingleTurnSurvey(const LowLevelWam<DOF>& llw_) : llw(llw_) {
		for (size_t i = 0; i < DOF; ++i) {
			dir[i] = 0;
			conflict[i] = false;
			haveStart[i] = read(i, &startPosition[i], &startSingleTurn[i]);
		}
	}

	// Read motor i at rest and let its motion since the start vote on the
	// direction.  Returns the single-turn reading, or -1 if it was not at rest.
	int sample(size_t i) {
		int position = 0, singleTurn = 0;
		if ( !read(i, &position, &singleTurn) ) {
			return -1;
		}
		if (haveStart[i]) {
			const int d = bus::CANopen::singleTurnDirection(
					position - startPosition[i], singleTurn - startSingleTurn[i]);
			if (d != 0  &&  dir[i] == 0) {
				dir[i] = d;
			} else if (d != 0  &&  d != dir[i]) {
				conflict[i] = true;
			}
		}
		return singleTurn;
	}

	// +1 or -1; 0 if the motor never turned enough to tell, or if the
	// readings disagreed (its counter does not track the encoder).
	int direction(size_t i) const { return conflict[i] ? 0 : dir[i]; }
	bool disagreed(size_t i) const { return conflict[i]; }
	bool started(size_t i) const { return haveStart[i]; }

protected:
	bool read(size_t i, int* position, int* singleTurn) const {
		const Puck& puck = *llw.getPucks()[i];
		for (int attempt = 0; attempt < 10; ++attempt) {
			if (bus::CANopen::readCounterAndSingleTurn(puck.getBus(), puck.getId(), position, singleTurn)) {
				return true;
			}
			btsleep(0.05);
		}
		return false;
	}

	const LowLevelWam<DOF>& llw;
	int dir[DOF], startPosition[DOF], startSingleTurn[DOF];
	bool conflict[DOF], haveStart[DOF];
};

// Handles actions an displays associated with adjusting the angle of a
// particular joint.
template<size_t DOF>
class AdjustJointStep : public CalibrationStep {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	static const int DEFAULT_DIGIT = -2;  // Start by adjusting the hundredth's place.
	static const int MAX_DIGIT = 0;  // Display 1's...
	static const int MIN_DIGIT = -3;  // ... through thousandth's.

public:
	explicit AdjustJointStep(const libconfig::Setting& setting, systems::Wam<DOF>* wamPtr, jp_type* calOffsetPtr, jp_type* zeroPosPtr, v_type* zeroAnglePtr, SingleTurnSurvey<DOF>* surveyPtr) :
		CalibrationStep("Joint " + boost::lexical_cast<std::string>((int)setting[0])),
		j((int)setting[0] - 1), calPos(setting[1]), endCondition(setting[2].c_str()),
		wam(*wamPtr), calOffset(*calOffsetPtr), zeroPos(*zeroPosPtr), zeroAngle(*zeroAnglePtr),
		survey(*surveyPtr), state(0), digit(DEFAULT_DIGIT)
	{
		assert(j >= 0);
		assert(j < DOF);
		assert(wamPtr != NULL);
		assert(calOffsetPtr != NULL);
		assert(zeroPosPtr != NULL);
		assert(zeroAnglePtr != NULL);
		assert(surveyPtr != NULL);
	}
	virtual ~AdjustJointStep() {}


	virtual void onChangeActive() {
		if (active) {
			// Reset
			state = 0;
			digit = DEFAULT_DIGIT;
		}
	}

	virtual bool onKeyPress(enum Key k) {
		switch (state) {
		// Prompt to move the WAM
		case 0:
			if (k == K_ENTER) {
				state++;
				move();
			}
			break;

		// Wait until the move is done
		case 1:
			// This transition is handled in display().
			break;

		// Adjust the offset
		default:
			switch (k) {
			case K_LEFT:
				digit = std::min(MAX_DIGIT, digit+1);
				break;
			case K_RIGHT:
				digit = std::max(MIN_DIGIT, digit-1);
				break;

			case K_UP:
				calOffset[j] += std::pow(10.0, digit);
				move();
				break;
			case K_DOWN:
				calOffset[j] -= std::pow(10.0, digit);
				move();
				break;

			case K_ENTER:
				if (wam.moveIsDone()) {
					LowLevelWam<DOF>& llw = wam.getLowLevelWam();

					// Record actual joint position, not commanded joint position
					zeroPos[j] = wam.getJointPositions()[j];

					// Record the motor angles that affect this joint
					const sqm_type& m2jp = llw.getMotorToJointPositionTransform();
					double tolerance = m2jp.cwiseAbs().maxCoeff() * 1e-5;
					for (size_t i = 0; i < DOF; ++i) {
						// If the j,i entry is non-zero, then Motor i is in some way connected to Joint j
						if (math::abs(m2jp(j,i)) > tolerance) {
							int mech = llw.getPucks()[i]->getProperty(Puck::MECH);
							zeroAngle[i] = llw.getMotorPucks()[i].counts2rad(mech);
						}
					}
					// Every motor, not just this joint's: a row may swing
					// other joints out so that their motors turn too.
					for (size_t i = 0; i < DOF; ++i) {
						survey.sample(i);
					}

					// Log joint encoder data for factory calibration
					if (llw.hasJointEncoders()) {
						logMessage("Joint encoder angles: %s") % llw.getJointPositions(LowLevelWam<DOF>::PS_JOINT_ENCODER);
					  std::cout << "JP (Joint Encoders): " << llw.getJointPositions(LowLevelWam<DOF>::PS_JOINT_ENCODER);
                    }

					return false;  // Move on to the next joint!
				}
				break;

			default:
				break;
			}
		}

		return true;
	}

	virtual int display(int top, int left) {
		int line = top + CalibrationStep::display(top, left);  // Call super
		left += L_OFFSET;

		if (active) {
			ScopedAttr sa(A_STANDOUT, true);

			switch (state) {
			// Prompt to move the WAM
			case 0:
				mvprintw(line++,left, "Press [Enter] to move to: [%0.2f", calPos[0]);
				for (size_t i = 1; i < DOF; ++i) {
					printw(", %04.2f", calPos[i]);
				}
				printw("]");
				break;

			// Wait until the move is done
			case 1:
				mvprintw(line++,left, "Wait for the WAM to finish moving.");

				if (wam.moveIsDone()) {  // Have we arrived yet?
					state++;
				}
				break;

			// Adjust the offset
			default:
				mvprintw(line++,left, "Use arrow keys to adjust the Joint %d offset.", (int)j+1);
				sa.off();
				line++;
				mvprintw(line++,left, "                 ");
				attron(A_UNDERLINE);
				printw("Offset (rad)");
				attroff(A_UNDERLINE);
				mvprintw(  line,left, "                    %+06.3f", calOffset[j]);

				int x = getcurx(stdscr);
				mvchgat(line, x-5-digit + ((digit<0) ? 1:0), 1, A_STANDOUT, 0, NULL);

				line += 2;
				mvprintw(line++,left, "Press [Enter] once %s.", endCondition.c_str());
				break;
			}
		}

		return height(top);
	}

protected:
	void move() {
		while ( !wam.moveIsDone() ) {
			btsleep(0.02);
		}
		wam.moveTo(jp_type(calPos + calOffset), false);
	}

	size_t j;
	const jp_type calPos;
	std::string endCondition;

	systems::Wam<DOF>& wam;
	jp_type& calOffset;
	jp_type& zeroPos;
	v_type& zeroAngle;
	SingleTurnSurvey<DOF>& survey;

	int state, digit;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// std::min/max take these by reference: an unoptimized build needs their
// definitions to link.
template<size_t DOF> const int AdjustJointStep<DOF>::DEFAULT_DIGIT;
template<size_t DOF> const int AdjustJointStep<DOF>::MAX_DIGIT;
template<size_t DOF> const int AdjustJointStep<DOF>::MIN_DIGIT;

// The Exit menu item.
class ExitStep : public CalibrationStep {
public:
	explicit ExitStep(const std::string& title_ = "Exit") : CalibrationStep(title_) {}
	virtual ~ExitStep() {}

	virtual bool onKeyPress(enum Key k) {
		return k != K_ENTER;
	}

	virtual int display(int top, int left) {
		int line = top + CalibrationStep::display(top, left);  // Call super
		left += L_OFFSET;

		if (active) {
			ScopedAttr sa(A_STANDOUT, true);
			mvprintw(line++,left, "Press [Enter] to move the home position and record calibration data.");
		}

		return height(top);
	}
};


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	jp_type calOffset(0.0);  // The offsets as entered by the user
	jp_type zeroPos(0.0);  // Zero position angles in joint space
	v_type zeroAngle(0.0);  // Motor positions that correspond to zeroPos


	// Disable torque saturation because gravity compensation is off
	wam.jpController.setControlSignalLimit(jp_type());
	wam.moveTo(wam.getJointPositions());  // Hold position

	// Start watching the single-turn encoders before anything moves.
	SingleTurnSurvey<DOF> survey(wam.getLowLevelWam());


	// Read from config
	std::vector<CalibrationStep*> steps;
	try {
		libconfig::Config config;
		config.readFile(CAL_CONFIG_FILE.c_str());
		const libconfig::Setting& setting = config.lookup("zerocal")[pm.getWamDefaultConfigPath()];
		assert(setting.isList());
		assert(setting.getLength() == (int)DOF);

		for (size_t i = 0; i < DOF; ++i) {
			// Populate the menu
			steps.push_back(new AdjustJointStep<DOF>(setting[i], &wam, &calOffset, &zeroPos, &zeroAngle, &survey));
		}
	} catch (libconfig::ParseException e) {
		printf(">>> CONFIG FILE ERROR on line %d of %s: \"%s\"\n", e.getLine(), CAL_CONFIG_FILE.c_str(), e.getError());
		return 1;
	} catch (libconfig::SettingNotFoundException e) {
		printf(">>> CONFIG FILE ERROR in %s: could not find \"%s\"\n", CAL_CONFIG_FILE.c_str(), e.getPath());
		return 1;
	} catch (libconfig::SettingTypeException e) {
		printf(">>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n", CAL_CONFIG_FILE.c_str(), e.getPath());
		return 1;
	}
	steps.push_back(new ExitStep);


	// Set up ncurses
	initscr();
	curs_set(0);
	noecho();
	timeout(0);


	// Display loop
	assert(steps.size() >= 1);
	int selected = 0;
	steps[selected]->setSelected(true);
	int active = 0;
	steps[active]->setActive(true);
	int maxStep = 0;

	enum Key k;
	bool done = false;
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE  &&  !done) {
		k = getKey();
		if (active == -1) {  // Menu mode
			switch (k) {
			case K_UP:
				steps[selected]->setSelected(false);
				selected = std::max(0, selected-1);
				steps[selected]->setSelected(true);
				break;
			case K_DOWN:
				steps[selected]->setSelected(false);
				selected = std::min(maxStep, selected+1);
				steps[selected]->setSelected(true);
				break;
			case K_ENTER:
				active = selected;
				steps[active]->setActive(true);
				break;
			default:
				break;
			}
		} else {  // Adjust mode
			switch (k) {
			case K_BACKSPACE:
			case K_ESCAPE:
			case K_TAB:
				steps[active]->setActive(false);
				active = -1;
				break;
			default:
				// Forward to the active item by calling onKeyPress()
				if ( !steps[active]->onKeyPress(k) ) {
					// This step is done. Move to the next one.
					steps[selected]->setSelected(false);
					steps[active]->setActive(false);
					++selected;
					if (selected > maxStep)
						maxStep = selected;
					if (selected < (int)steps.size()) {
						active = selected;
						steps[selected]->setSelected(true);
						steps[active]->setActive(true);
					} else {  // Finished the last step!
						done = true;
					}
				}
				break;
			}
		}

		// Display
		clear();
		mvprintw(0,18, "*** Barrett WAM Zero Calibration Utility ***");
		int top = 3;
		for (size_t i = 0; i < steps.size(); ++i) {
			top += steps[i]->display(top,0);
		}
		refresh();

		btsleep(0.1);  // Slow loop down to ~10Hz
	}
	endwin();  // Turn off ncurses
	detail::purge(steps);


	// Restore stdout's line buffering
	if (freopen(NULL, "w", stdout) == NULL) {
		fprintf(stderr, ">>> ERROR: freopen(stdout) failed.\n");
		return 1;
	}


	if (done) {
		printf(">>> Calibration completed!\n");

		// Single-turn encoders, for rough placement at power-up.  The arm
		// still holds the last calibration pose: read every motor at rest once
		// more.  Where the direction is known, record the encoder angle at the
		// joint-zero pose computed from the actual joint angles.  Unlike the
		// reading taken at a joint's [Enter], that does not depend on how well
		// a differential's other joint was held on its mark.
		LowLevelWam<DOF>& llw = wam.getLowLevelWam();
		v_type zeroAngleDir(0.0);
		for (size_t i = 0; i < DOF; ++i) {
			const int singleTurn = survey.sample(i);
			const int d = survey.direction(i);
			if (singleTurn >= 0  &&  d != 0) {
				const jp_type fromZero(wam.getJointPositions() - zeroPos);
				zeroAngle[i] = llw.singleTurnAtZero(i, llw.getMotorPucks()[i].counts2rad(singleTurn), fromZero, d);
				zeroAngleDir[i] = d;
			}
		}

		char* dataConfigFile = new char[strlen(DATA_CONFIG_FILE.c_str()) + strlen(pm.getWamDefaultConfigPath()) - 2 + 1];
		sprintf(dataConfigFile, DATA_CONFIG_FILE.c_str(), pm.getWamDefaultConfigPath());
		manageBackups(dataConfigFile);  // Backup old calibration data

		// Save to the data config file
		jp_type newHome(wam.getHomePosition() - zeroPos);

		libconfig::Config dataConfig;
		libconfig::Setting& homeSetting = dataConfig.getRoot().add("home", libconfig::Setting::TypeList);
		libconfig::Setting& zaSetting = dataConfig.getRoot().add("zeroangle", libconfig::Setting::TypeList);
		libconfig::Setting& zdSetting = dataConfig.getRoot().add("zeroangle_dir", libconfig::Setting::TypeList);
		for (size_t i = 0; i < DOF; ++i) {
			homeSetting.add(libconfig::Setting::TypeFloat) = newHome[i];
			zaSetting.add(libconfig::Setting::TypeFloat) = zeroAngle[i];
			zdSetting.add(libconfig::Setting::TypeFloat) = zeroAngleDir[i];
		}

		dataConfig.writeFile(dataConfigFile);
		printf(">>> Data written to: %s\n", dataConfigFile);

		delete[] dataConfigFile;

		// Which joints may now be placed roughly at power-up?
		const sqm_type& m2jp = llw.getMotorToJointPositionTransform();
		const double tolerance = m2jp.cwiseAbs().maxCoeff() * 1e-5;
		std::string rough, exact;
		for (size_t j = 0; j < DOF; ++j) {
			bool measured = true;
			for (size_t i = 0; i < DOF; ++i) {
				if (math::abs(m2jp(j,i)) > tolerance  &&  zeroAngleDir[i] == 0.0) {
					measured = false;
				}
			}
			std::string& list = measured ? rough : exact;
			list += (list.empty() ? "J" : ", J") + boost::lexical_cast<std::string>(j + 1);
		}
		printf(">>> Rough placement at power-up (within half a motor turn of home): %s.\n",
				rough.empty() ? "no joints" : rough.c_str());
		if ( !exact.empty()) {
			printf("    Place exactly at home, e.g. on a stop or pin: %s. Their motors did not\n"
					"    turn enough during this calibration to measure their encoders.\n", exact.c_str());
		}
		for (size_t i = 0; i < DOF; ++i) {
			const int id = llw.getPucks()[i]->getId();
			if ( !survey.started(i)) {
				printf("    Puck %d could not be read at rest when the calibration started.\n", id);
			} else if (survey.disagreed(i)) {
				printf("    Puck %d: its position counter did not track its single-turn encoder,\n"
						"    so it is not used for rough placement.\n", id);
			}
		}


		// Move home!
		printf(">>> Moving back to home position.\n");
		wam.moveHome();
	} else {
		printf(">>> ERROR: WAM was Idled before the calibration was completed.\n");
		return 1;
	}


	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	// Make sure the user applies their new calibration
	pm.getSafetyModule()->setWamZeroed(false);

	return 0;
}


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();


	printf(
"\n"
"                  *** Barrett WAM Zero Calibration Utility ***\n"
"\n"
"This utility will help you set the joint angle offsets for your WAM Arm. Joint\n"
"angle offsets affect operations that rely on the robot's kinematics, such as\n"
"gravity compensation, haptics, and Cartesian position and orientation control.\n"
"You should perform this calibration any time you replace a broken cable, add\n"
"tension to a cable, or re-mount a Puck.\n"
"\n"
"The WAM's standard home position is described in the User Manual. However, you\n"
"may choose any other pose to be your home position if it is more convenient for\n"
"your application. You will need to return the WAM to the home position on power-\n"
"up and after certain types of safety faults.\n"
"\n"
"Modern WAMs have absolute encoders on their motors, so the WAM only needs to be\n"
"returned to within one motor revolution of its home position. This translates to\n"
"roughly +/- 5 degrees at the joint. (The tolerance is different for each joint.)\n"
"\n"
"This program assumes the WAM is mounted such that the base is horizontal.\n"
"\n"
"\n"
	);


	ProductManager pm;
	pm.waitForWam(false);  // Don't prompt on zeroing
	pm.wakeAllPucks();

	SafetyModule* sm = pm.getSafetyModule();
	if (sm == NULL) {
		printf("ERROR: No SafetyModule found.\n");
		return 1;
	} else {
		sm->setWamZeroed(false);
	}

	// Remove existing zerocal information, if present. Must be done before
	// calling ProductManager::getWam*().
	libconfig::Setting& llSetting = pm.getConfig().lookup(pm.getWamDefaultConfigPath())["low_level"];
	if (llSetting.exists("zeroangle")) {
		llSetting.remove(llSetting["zeroangle"].getIndex());
		syslog(LOG_ERR, "** Ignoring previous \"zeroangle\" vector **");
	}
	if (llSetting.exists("zeroangle_dir")) {
		llSetting.remove(llSetting["zeroangle_dir"].getIndex());
	}

	if (sm->getPuck()->getVers() >= bus::CANopen::SB_CANOPEN_VERS) {
		// P4 pucks: rough placement needs each motor's single-turn encoder
		// direction, which this program measures while the arm moves.
		printf(">>> CANopen WAM: this calibration also measures each P4's single-turn encoder.\n"
				"    Afterwards, power-up needs the WAM only roughly at home (within half a\n"
				"    motor turn) at every joint whose motors turn during the calibration.\n");
	}
	printf(">>> Please *carefully* place the WAM in its home position, then press [Enter].");
	waitForEnter();

	if (pm.foundWam8g()) {
		return wam_main(argc, argv, pm, *pm.getWam8g());
	} else if (pm.foundWam3()){
    return wam_main(argc, argv, pm, *pm.getWam3());
  } else if (pm.foundWam4()) {
		return wam_main(argc, argv, pm, *pm.getWam4());
	} else if (pm.foundWam7()) {
		return wam_main(argc, argv, pm, *pm.getWam7());
	} else {
		printf(">>> ERROR: No WAM was found. Perhaps you have found a bug in ProductManager::waitForWam().\n");
		return 1;
	}
}
