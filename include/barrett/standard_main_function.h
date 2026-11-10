/*
	Copyright 2009-2014 Barrett Technology <support@barrett.com>

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

*/

/** Defines a boilerplate main() function that initializes a WAM.
 *
 * This "standard" main() function simply initializes the WAM in the normal way, by:
 *   - Waiting for the WAM to be Shift-idled, if it's not already idled
 *   - Waking all Pucks found on the bus
 *   - Zero the WAM, if it's not already zeroed
 *   - Waiting for the WAM to be Shift-activated
 *   - Calling the wam_main() function that the user is responsible for defining
 *   .
 * The behavior is appropriate for many applications, but there is no issue with writing a custom main() function.
 * The eight-joint wam8g (4-DOF WAM + passive J5 + 3-motor wrist) is a
 * barrett::systems::Wam<8> and is dispatched to the same templated wam_main()
 * with DOF = 8. Define BARRETT_SMF_WAM8G before including this header for a
 * wam8g-only application that wants the seven-active-joint view, with
 * wam_main(..., barrett::systems::Wam8g&); define BARRETT_SMF_NO_WAM8G for an
 * application that cannot be built for DOF = 8.
 *
 * The wam_main() function must be able to accept a reference to a 3-DOF, 4-DOF, 7-DOF and 8-DOF barrett::systems::Wam.
 * If your code treats these two cases similarly, then define a templated function:
 * @include zero_torque.cpp
 *
 * If your code handles these two cases very differently, then define two overloaded functions:
 * @include seven_dof_only.cpp
 *
 * @file standard_main_function.h
 * @date Nov 5, 2010
 * @author Dan Cody
 * @author CJ Valle
 * @author JP Hagstrand
 */


#ifndef BARRETT_STANDARD_MAIN_FUNCTION_H_
#define BARRETT_STANDARD_MAIN_FUNCTION_H_


#include <barrett/exception.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/systems/wam8g.h>


#ifdef BARRETT_SMF_VALIDATE_ARGS
	bool validate_args(int argc, char** argv);
#endif

#ifdef BARRETT_SMF_CONFIGURE_PM
	bool configure_pm(int argc, char** argv, ::barrett::ProductManager& pm);
#endif

#ifndef BARRETT_SMF_NO_DECLARE
	template<size_t DOF> int wam_main(int argc, char** argv, ::barrett::ProductManager& pm, ::barrett::systems::Wam<DOF>& wam);
#ifdef BARRETT_SMF_WAM8G
	// Optional: an application that wants the seven-active-joint view of
	// wam8g (Wam8g is also a Wam<8>, so the template above is used for it
	// otherwise).
	int wam_main(int argc, char** argv, ::barrett::ProductManager& pm, ::barrett::systems::Wam8g& wam);
#endif
#endif

#ifndef BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#  define BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE true
#else
#  define BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE false
#endif

#ifndef BARRETT_SMF_DONT_PROMPT_ON_ZEROING
#  define BARRETT_SMF_PROMPT_ON_ZEROING true
#else
#  define BARRETT_SMF_PROMPT_ON_ZEROING false
#endif

#ifndef BARRETT_SMF_WAM_CONFIG_PATH
#  define BARRETT_SMF_WAM_CONFIG_PATH NULL
#endif


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	::barrett::installExceptionHandler();


#ifdef BARRETT_SMF_VALIDATE_ARGS
	if ( !validate_args(argc, argv) ) {
		return 1;
	}
#endif


	::barrett::ProductManager pm;
#ifdef BARRETT_SMF_CONFIGURE_PM
	if ( !configure_pm(argc, argv, pm) ) {
		return 1;
	}
#endif

	pm.waitForWam(false);
#ifdef BARRETT_SMF_WAM8G
	if (!pm.foundWam8g()) {
		printf(">>> ERROR: This application requires a complete wam8g.\n");
		return 1;
	}
#endif
#ifdef BARRETT_SMF_NO_WAM8G
	if (pm.foundWam8g()) {
		printf(">>> ERROR: This application does not support wam8g.\n");
		return 1;
	}
#endif
	pm.wakeAllPucks();
	// CANopen wake resets encoder counters and clears the ZERO flag. Prompt
	// after wake so an old flag cannot skip the required reference pose.
	pm.waitForWam(BARRETT_SMF_PROMPT_ON_ZEROING);
	// TODO(JH): Rehab Update implement and test
#ifdef BARRETT_SMF_WAM8G
	// A wam8g-only application defines just the Wam8g overload; do not
	// instantiate the template for the other products (foundWam8g() was
	// already required above).
	return wam_main(argc, argv, pm, *pm.getWam8g(BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE, BARRETT_SMF_WAM_CONFIG_PATH));
#else
#ifndef BARRETT_SMF_NO_WAM8G
	// wam8g is dispatched like any other WAM: as a Wam<8> to the template
	// wam_main.  Define BARRETT_SMF_NO_WAM8G in an application that cannot
	// be built for DOF = 8.
	if (pm.foundWam8g()) {
		return wam_main(argc, argv, pm, *pm.getWam8g(BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE, BARRETT_SMF_WAM_CONFIG_PATH));
	} else
#endif
	if (pm.foundWam3()) {
		return wam_main(argc, argv, pm, *pm.getWam3(BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE, BARRETT_SMF_WAM_CONFIG_PATH));
	} else if (pm.foundWam4()) {
		return wam_main(argc, argv, pm, *pm.getWam4(BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE, BARRETT_SMF_WAM_CONFIG_PATH));
	} else if (pm.foundWam7()) {
		return wam_main(argc, argv, pm, *pm.getWam7(BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE, BARRETT_SMF_WAM_CONFIG_PATH));
	} else {
		printf(">>> ERROR: No WAM was found. Perhaps you have found a bug in ProductManager::waitForWam().\n");
		return 1;
	}
#endif
}


#endif /* BARRETT_STANDARD_MAIN_FUNCTION_H_ */
