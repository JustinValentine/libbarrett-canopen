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
 *
 */

/*
 * @file product_manager.cpp
 *
 * @date Jan 3, 2011
 * @author D.Cody
 * @author J.Hagstrand
 */

#include <stdexcept>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#include <string.h>
#include <libgen.h>

#include <libconfig.h++>

#include <barrett/os.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/bus/canopen.h>
#include <barrett/products/puck.h>
#include <barrett/products/hand.h>
#include <barrett/products/gimbals_hand_controller.h>
#include <barrett/products/safety_module.h>
#include <barrett/products/force_torque_sensor.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/real_time_execution_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/systems/wam8g.h>
#include <barrett/products/product_manager.h>

#include <barrett/config.h>

namespace barrett {


const std::string ProductManager::DEFAULT_CONFIG_FILE = barrett::EtcPathRelative("default.conf");

ProductManager::ProductManager(const char* configFile, bus::CommunicationsBus* _bus) :
	config(), bus(_bus), deleteBus(false),
	pucks(), wamPucks(MAX_WAM_DOF), handPucks(Hand::DOF),
	sm(NULL), rtem(NULL), wam3(NULL), wam4(NULL), wam7(NULL), wam8g(NULL), fts(NULL), hand(NULL), ghc(NULL)
{
	int ret;


	logMessage("ProductManager::%s()") % __func__;

	char cfSource[8] = "param";
	if (configFile == NULL  ||  configFile[0] == '\0') {
		configFile = std::getenv("BARRETT_CONFIG_FILE");
		if (configFile == NULL  ||  configFile[0] == '\0') {
			configFile = DEFAULT_CONFIG_FILE.c_str();
			strcpy(cfSource, "default");
		} else {
			strcpy(cfSource, "env");
		}
	}
	logMessage("  Config file (from %s): %s") % cfSource % configFile;


	char* cf1 = strdup(configFile);
	if (cf1 == NULL) {
		throw std::runtime_error("Out of memory.");
	}
	char* cf2 = strdup(configFile);
	if (cf2 == NULL) {
		free(cf1);
		throw std::runtime_error("Out of memory.");
	}
	char* origWd = getcwd(NULL, 0);
	if (origWd == NULL) {
		free(cf1);
		free(cf2);
		throw std::runtime_error("Out of memory.");
	}

	// These functions require copies of the string because they sometimes modify their argument.
	// Also, dirname() and basename() may EITHER perform a malloc() OR just return a pointer to a substring.
	// Since we free() cf1 and cf2 later, we force a malloc() with strdup().
	configDir = strdup(dirname(cf1));
	configBase = strdup(basename(cf2));

	// make @include paths in configFile relative to the containing directory
	ret = chdir(configDir);
	if (ret != 0) {
		free(cf1);
		free(cf2);
		free(origWd);
		throw std::runtime_error("Couldn't change to the config directory.");
	}

	try {
		config.readFile(configBase);

		if (bus == NULL) {
			bus = new bus::BusManager;
			deleteBus = true;
		}
		if ( !bus->isOpen() ) {
			bus->open(config.lookup("bus.port"));
		}
	} catch (libconfig::ParseException pe) {
		printf("\n>>> CONFIG FILE ERROR on line %d of %s: \"%s\"\n\n", pe.getLine(), configFile, pe.getError());
		printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
		printf("This error usually means that a configuration file is corrupted or missing.\n");
		printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");

		ret = chdir(origWd);  // Ignore ret value
		free(cf1);
		free(cf2);
		free(origWd);
		throw;
	} catch (...) {
		ret = chdir(origWd);  // Ignore ret value
		free(cf1);
		free(cf2);
		free(origWd);
		throw;
	}

	ret = chdir(origWd);
	free(cf1);
	free(cf2);
	free(origWd);
	if (ret != 0) {
		throw std::runtime_error("Couldn't change back to the original working directory.");
	}

	enumerate();
}

ProductManager::~ProductManager()
{
	destroyEstopProducts();
	delete sm;
	sm = NULL;
	detail::purge(pucks);
	if (deleteBus) {
		delete bus;
		bus = NULL;
	}
}

void ProductManager::enumerate()
{
	int ret, result;
	Puck* p = NULL;
	int lastId = -1;
	int id;
	unsigned char data[8];
	size_t len;

	logMessage("ProductManager::%s()") % __func__;

	// Clear out the socket (stop on an error too: it may never clear)
	do {
		ret  = bus->receiveRaw(id, data, len, false);
	} while (ret == 0);

	logMessage("  Pucks:");
	for (int id = Puck::MIN_ID; id <= Puck::MAX_ID; ++id) {
		// CANopen node probe (contract section 10): every CANopen
		// device answers an SDO read of 0x1000 (device type); the old
		// broadcast STAT poll does not exist on this bus.
		ret = bus::CANopen::sdoUpload(*bus, id, bus::CANopen::OD_DEVICE_TYPE, 0,
				&result, 0.02);
		p = getPuck(id);

		if (ret == 0) {
			// if the Puck doesn't exist, make it
			if (p == NULL) {
				p = new Puck(*bus, id);
				pucks.push_back(p);
			} else {
				// if the Puck already exists (from a previous enumeration), update it
				p->updateRole();
				p->updateStatus();
				p->setAwake(false);  // an e-stop may have restarted it
			}

			if (lastId != id - 1  &&  lastId != -1) {
				logMessage("    --");  // marker to indicate that the listed IDs are not contiguous
			}
			logMessage("    ID=%2d VERS=%3d (identity rev %d) ROLE=0x%04x TYPE=%s%s")
					% p->getId() % p->getVers() % p->getIdentityRevision() % p->getRole()
					% Puck::getPuckTypeStr(p->getType())
					% ((p->getEffectiveType() == Puck::PT_Monitor) ? " (Monitor)" : "");
			lastId = id;
		} else if (p != NULL) {
			// if the Puck has disappeared since the last enumeration, remove it
			deletePuck(p);
		}
	}


	// update WAM/Hand Pucks
	for (size_t i = 0; i < MAX_WAM_DOF; ++i) {
		wamPucks[i] = getPuck(i + FIRST_WAM_ID);
	}
	for (size_t i = 0; i < Hand::DOF; ++i) {
		handPucks[i] = getPuck(i + FIRST_HAND_ID);
	}


	logMessage("  Products:");
	bool noProductsFound = true;
	bool wamFound = false;
	// TODO(JH): Rehab Update implement and test
	if (foundWam3()) {
		noProductsFound = false;
		wamFound = true;
		logMessage("    3-DOF WAM");
	}
	if (foundWam4()) {
		noProductsFound = false;
		wamFound = true;
		logMessage("    4-DOF WAM");
	}
	if (foundWam7()) {
		noProductsFound = false;
		wamFound = true;
		logMessage("    7-DOF WAM%s%s") % (foundWam7Wrist() ? " (Wrist)" : "") % (foundWam7Gimbals() ? " (Gimbals)" : "");
	}
	if (foundWam8g()) {
		noProductsFound = false;
		wamFound = true;
		logMessage("    WAM8g (8 measured, 7 active joints; passive J5)");
	}
	if (wamFound) {
		if (foundSafetyModule()) {
			logMessage("    Safety Module");
		} else {
			logMessage("    *** NO SAFETY MODULE ***");
		}
	}
	if (foundForceTorqueSensor()) {
		noProductsFound = false;
		logMessage("    Force-Torque Sensor");
	}
	if (foundHand()) {
		noProductsFound = false;
		logMessage("    BarrettHand");
	}
	// TODO(dc): Don't report GHC because we don't have a very selective test.

	if (noProductsFound) {
		logMessage("    (none)");
	}
}

void ProductManager::cleanUpAfterEstop()
{
	destroyEstopProducts();
	enumerate();
}
void ProductManager::destroyEstopProducts()
{
	delete rtem;
	rtem = NULL;
	// TODO(JH): Rehab Update implement and test
	delete wam3;
	wam3 = NULL;
	delete wam4;
	wam4 = NULL;
	delete wam7;
	wam7 = NULL;
	delete wam8g;
	wam8g = NULL;
	delete fts;
	fts = NULL;
	delete hand;
	hand = NULL;
	delete ghc;
	ghc = NULL;
}

int ProductManager::expectedPuckMask() const
{
	// Optional.  Without it the product is identified from the pucks that
	// answered, exactly as libbarrett always has.  A host that must never
	// be mistaken for a smaller WAM when a puck is missing (an eight-puck
	// wam8g losing node 8 would otherwise look like a 7-DOF WAM) declares
	// its physical puck set here and startup stops on any mismatch.
	if (!config.exists("bus.expected_puck_mask")) {
		return 0;
	}
	const int mask = config.lookup("bus.expected_puck_mask");
	if (mask <= 0 || (mask & ~0x01FE) != 0) {
		throw std::invalid_argument("bus.expected_puck_mask must contain only puck-node bits 1-8 and must not be empty.");
	}
	if (mask == 0x01FE && !config.exists("wam8g")) {
		throw std::invalid_argument("The eight-puck IROS system requires the wam8g profile; refusing to identify it as a seven-DOF WAM.");
	}
	return mask;
}

int ProductManager::presentPuckMask() const
{
	int mask = 0;
	for (int node = 1; node <= 8; ++node) {
		Puck* puck = getPuck(node);
		if (puck != NULL && puck->getType() == Puck::PT_Motor) {
			mask |= 1 << node;
		}
	}
	return mask;
}

bool ProductManager::expectedPucksPresent() const
{
	const int expected = expectedPuckMask();
	return expected == 0 || expected == presentPuckMask();
}

void ProductManager::wakeAllPucks() const
{
	const int present = presentPuckMask();
	const int expected = expectedPuckMask();
	if (present == 0) {
		(logMessage("ProductManager::%s(): no pucks found. Startup stopped.")
				% __func__).raise<std::runtime_error>();
	}
	if (!expectedPucksPresent()) {
		(logMessage("ProductManager::%s(): puck set mismatch: expected 0x%03x, found 0x%03x. Startup stopped.")
				% __func__ % expected % present).raise<std::runtime_error>();
	}
	// The safety board watches the declared set, or the detected one.
	const int required = (expected != 0) ? expected : present;
	if (!foundSafetyModule()) {
		throw std::runtime_error("ProductManager::wakeAllPucks(): no safety board; startup stopped.");
	}
	// Sensor-only nodes (the wam8g's motorless J5 puck) must be known to
	// both the host wake path and the safety board (0x2100:04) before any
	// bring-up. Keyed on the DETECTED product: every host config includes
	// wam8g.conf, and a plain 7-DOF WAM's J5 drive must not be exempted.
	const int mask = foundWam8g() ? (1 << 5) : 0;
	// Both masks are acknowledged and read back before resetting or arming
	// any puck. The required set also repairs the SB's watch list when
	// warm drives emitted no heartbeat during Shift-Idle enumeration.
	bus::CANopen::configureSafetyNodes(*bus, required, mask);
	bus::CANopen::setSensorOnlyNodes(*bus, mask);
	bus::CANopen::validateWamEncoders(*bus);

	// Keep the zero of the previous run?  Only while the safety board
	// still holds it (SB r209+; it forgets it on ESTOP->IDLE and whenever
	// a puck reboots or is reset) and every puck provably kept counting.
	// Then skip the NMT resets: they would restart the position counters.
	bool keepZero = false;
	if (getPuck(SAFETY_MODULE_ID)->getVers() >= bus::CANopen::SB_ZERO_RECORD_VERS
			&& bus::CANopen::zeroRecordNodes(*bus) != 0) {
		bus::CANopen::ZeroRecord record;
		std::string why;
		keepZero = bus::CANopen::readZeroRecord(*bus, required, &record, &why);
		if (!keepZero) {
			printf(">>> Not keeping the previous WAM zero: %s.\n", why.c_str());
			bus::CANopen::clearZeroRecord(*bus);
		}
	}

	try {
		Puck::wake(getPucks(), !keepZero);
	} catch (...) {
		// armCstMode temporarily enables a drive to select its mode.
		// Any incomplete bring-up must also request a hardware cutoff.
		try {
			getPuck(SAFETY_MODULE_ID)->setProperty(Puck::MODE, SafetyModule::ESTOP, true);
		} catch (...) {
			logMessage("ProductManager::wakeAllPucks(): safety-board ESTOP request failed during startup cleanup.", true);
		}
		throw;
	}

	// The host keeps the WAM zero (a P4 restarts its position counter at 0
	// on every reset), so the board's own ZERO flag must stay 0: it keeps
	// the board on per-joint velocity checks rather than its stock-WAM
	// Cartesian model.  Before r209 it is also what tells LowLevelWam to
	// zero the WAM again (SafetyModule::wamIsZeroed()).
	getPuck(SAFETY_MODULE_ID)->setProperty(Puck::ZERO, 0, true);
}


bool ProductManager::foundSafetyModule() const
{
	return getPuck(SAFETY_MODULE_ID) != NULL;
}
SafetyModule* ProductManager::getSafetyModule()
{
	if (sm == NULL  &&  foundSafetyModule()) {
		sm = new SafetyModule(getPuck(SAFETY_MODULE_ID));
	}
	return sm;
}


const std::vector<Puck*>& ProductManager::getWamPucks() const
{
	return wamPucks;
}
// TODO(JH): Rehab Update implement and test
bool ProductManager::foundWam3() const
{
	return verifyWamPucks(3);
}		
bool ProductManager::foundWam4() const
{
	return verifyWamPucks(4);
}
bool ProductManager::foundWam7() const
{
	return verifyWamPucks(7);
}
bool ProductManager::foundWam8g() const
{
	return config.exists("wam8g") && verifyWamPucks(8);
}
bool ProductManager::foundWam7Wrist() const
{
	return wam7FoundHelper(6);
}
bool ProductManager::foundWam7Gimbals() const
{
	return wam7FoundHelper(8);
}
bool ProductManager::wam7FoundHelper(int poles) const
{
	if (foundWam7()) {
		Puck* p7 = getPuck(7);
		p7->wake();
		return p7->getProperty(Puck::POLES) == poles;
	} else {
		return false;
	}
}

void ProductManager::waitForWam(bool promptOnZeroing)
{
	if ( !foundSafetyModule() ) {
		printf(">>> ERROR: No SafetyModule was found.\n");
		exit(1);
	}
	SafetyModule* sm = getSafetyModule();

	sm->waitForMode(SafetyModule::IDLE);
	if ( !foundWam() ) {
		enumerate();
		if ( !foundWam() ) {
			printf(">>> ERROR: No complete WAM was found (expected puck mask 0x%03x, found 0x%03x).\n", expectedPuckMask(), presentPuckMask());
			exit(1);
		}
	}

	if (promptOnZeroing) {
		if ( !sm->wamIsZeroed() ) {
			printf(">>> The WAM needs to be zeroed. Please move it to its home position, then press [Enter].");
			detail::waitForEnter();
		} else if (sm->hasZeroRecord()) {
			printf(">>> Keeping the WAM zero from the previous run (no puck has been reset since).\n"
					"    To zero it again at the home position: E-Stop, Shift-Idle, restart.\n");
		}
	}

}
const char* ProductManager::getWamDefaultConfigPath()
{
	// TODO(JH): Rehab Update implement and test
	if (foundWam8g()) {
		return "wam8g";
	} else if (foundWam3()){
		return "wam3";
	} else if (foundWam4()) {
		return "wam4";
	} else if (foundWam7Wrist()) {
		return "wam7w";
	} else if (foundWam7Gimbals()) {
		return "wam7g";
	} else {
		throw std::logic_error("ProductManager::getWamDefaultConfigPath(): No WAM found.");
	}
}
// TODO(JH): Rehab Update implement and test
systems::Wam<3>* ProductManager::getWam3(bool waitForShiftActivate, const char* configPath)
{
	if ( !foundWam3() ) {
		throw std::logic_error("ProductManager::getWam3(): No WAM3 was found on the bus.");
	}

	if (wam3 == NULL) {
		std::vector<Puck*> wam3Pucks = wamPucks;
		wam3Pucks.resize(3);  // Discard all but the first 3 elements

		if (configPath == NULL) {
			configPath = getWamDefaultConfigPath();
		}
		try {
			wam3 = new systems::Wam<3>(getExecutionManager(), wam3Pucks, getSafetyModule(), getConfig().lookup(configPath));
		} catch (libconfig::FileIOException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: I/O while reading file\n\n", configPath);
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingNotFoundException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: could not find \"%s\"\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a specific setting is missing.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingTypeException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a setting is improperly formatted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		}
		startExecutionManager();
	}

	if (waitForShiftActivate) {
		if ( !foundSafetyModule() ) {
			throw std::logic_error("ProductManager::getWam3(): No SafetyModule was found on the bus.");
		}

		// Check rapidly in case the user wants to perform some action (like
		// enabling gravity compensation) immediately after Shift-activate.
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 0.05);
	}

	return wam3;
}
systems::Wam<4>* ProductManager::getWam4(bool waitForShiftActivate, const char* configPath)
{
	if ( !foundWam4() ) {
		throw std::logic_error("ProductManager::getWam4(): No WAM4 was found on the bus.");
	}

	if (wam4 == NULL) {
		std::vector<Puck*> wam4Pucks = wamPucks;
		wam4Pucks.resize(4);  // Discard all but the first 4 elements

		if (configPath == NULL) {
			configPath = getWamDefaultConfigPath();
		}
		try {
			wam4 = new systems::Wam<4>(getExecutionManager(), wam4Pucks, getSafetyModule(), getConfig().lookup(configPath));
		} catch (libconfig::FileIOException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: I/O while reading file\n\n", configPath);
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingNotFoundException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: could not find \"%s\"\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a specific setting is missing.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingTypeException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a setting is improperly formatted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		}
		startExecutionManager();
	}

	if (waitForShiftActivate) {
		if ( !foundSafetyModule() ) {
			throw std::logic_error("ProductManager::getWam4(): No SafetyModule was found on the bus.");
		}

		// Check rapidly in case the user wants to perform some action (like
		// enabling gravity compensation) immediately after Shift-activate.
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 0.05);
	}

	return wam4;
}

systems::Wam<7>* ProductManager::getWam7(bool waitForShiftActivate, const char* configPath)
{
	if ( !foundWam7() ) {
		throw std::logic_error("ProductManager::getWam7(): No WAM7 was found on the bus.");
	}

	if (wam7 == NULL) {
		std::vector<Puck*> wam7Pucks = wamPucks;
		wam7Pucks.resize(7);  // Discard all but the first 7 elements

		if (configPath == NULL) {
			configPath = getWamDefaultConfigPath();
		}
		try {
			wam7 = new systems::Wam<7>(getExecutionManager(), wam7Pucks, getSafetyModule(), getConfig().lookup(configPath));
		} catch (libconfig::FileIOException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: I/O while reading file\n\n", configPath);
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingNotFoundException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: could not find \"%s\"\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a specific setting is missing.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingTypeException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a setting is improperly formatted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		}
		startExecutionManager();
	}

	if (waitForShiftActivate) {
		if ( !foundSafetyModule() ) {
			throw std::logic_error("ProductManager::getWam7(): No SafetyModule was found on the bus.");
		}

		// Check rapidly in case the user wants to perform some action (like
		// enabling gravity compensation) immediately after Shift-activate.
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 0.05);
	}

	return wam7;
}

systems::Wam8g* ProductManager::getWam8g(bool waitForShiftActivate, const char* configPath)
{
	if (!foundSafetyModule()) {
		throw std::logic_error("ProductManager::getWam8g(): no safety board was found.");
	}
	if (!foundWam8g()) {
		throw std::logic_error("ProductManager::getWam8g(): no complete WAM8g was found.");
	}
	if (wam3 != NULL || wam4 != NULL || wam7 != NULL) {
		throw std::logic_error("ProductManager::getWam8g(): nodes already owned by another product; clean up before changing profiles.");
	}
	if (wam8g == NULL) {
		if (configPath == NULL) {
			configPath = getWamDefaultConfigPath();
		}
		try {
			wam8g = new systems::Wam8g(getExecutionManager(), wamPucks, getSafetyModule(), getConfig().lookup(configPath));
		} catch (libconfig::FileIOException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: I/O while reading file\n\n", configPath);
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingNotFoundException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: could not find \"%s\"\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingTypeException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		}
		startExecutionManager();
	}
	if (waitForShiftActivate) {
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 0.05);
	}
	return wam8g;
}

systems::RealTimeExecutionManager* ProductManager::getExecutionManager(double period_s, int rt_priority)
{
	if (rtem == NULL) {
		if (!std::isfinite(period_s) || std::fabs(period_s - bus::CANopen::SYNC_PERIOD_US * 1e-6) > 1e-9) {
			throw std::invalid_argument("The CANopen puck configuration requires a 2 ms (500 Hz) execution period.");
		}
		rtem = new systems::RealTimeExecutionManager(period_s, rt_priority);
	}
	return rtem;
}
void ProductManager::startExecutionManager() {
	getExecutionManager();
	if ( !rtem->isRunning() ) {
		rtem->start();
	}
}


bool ProductManager::foundForceTorqueSensor() const
{
	Puck* p = getPuck(FORCE_TORQUE_SENSOR_ID);
	return p != NULL && p->getType() == Puck::PT_ForceTorque;
}
ForceTorqueSensor* ProductManager::getForceTorqueSensor()
{
	if (fts == NULL  &&  foundForceTorqueSensor()) {
		fts = new ForceTorqueSensor(getPuck(FORCE_TORQUE_SENSOR_ID));
	}
	return fts;
}


const std::vector<Puck*>& ProductManager::getHandPucks() const
{
	return handPucks;
}
bool ProductManager::foundHand() const
{
	return std::find(handPucks.begin(), handPucks.end(), (Puck*)NULL) == handPucks.end();
}
Hand* ProductManager::getHand()
{
	if (hand == NULL  &&  foundHand()) {
		hand = new Hand(handPucks);
	}
	return hand;
}


bool ProductManager::foundGimbalsHandController() const
{
	return !foundWam8g() && wamPucks[5] != NULL  &&  wamPucks[6] != NULL  &&  !foundWam7Wrist();
}
GimbalsHandController* ProductManager::getGimbalsHandController()
{
	if (ghc == NULL  &&  foundGimbalsHandController()) {
		ghc = new GimbalsHandController(wamPucks[5], wamPucks[6]);
	}
	return ghc;
}


Puck* ProductManager::getPuck(int id) const
{
	for (size_t i = 0; i < pucks.size(); ++i) {
		if (pucks[i]->getId() == id) {
			return pucks[i];
		}
	}
	return NULL;
}

void ProductManager::deletePuck(Puck* p)
{
	std::vector<Puck*>::iterator i;
	i = std::find(pucks.begin(), pucks.end(), p);

	if (i == pucks.end()) {
		throw std::invalid_argument("ProductManager::deletePuck(): Puck is not being managed by this ProductManager.");
	}

	*i = pucks.back();
	pucks.pop_back();

	delete p;
}

bool ProductManager::verifyWamPucks(const size_t dof) const
{
	if (!expectedPucksPresent()) return false;
	if (dof > MAX_WAM_DOF) {
		return false;
	}

	for (size_t i = 0; i < MAX_WAM_DOF; ++i) {
		if ( (i < dof)  ^  (wamPucks[i] != NULL && wamPucks[i]->getType() == Puck::PT_Motor) ) {
			return false;
		}
	}

	return true;
}


}
