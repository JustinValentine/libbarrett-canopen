/* Position-sign check, no torque involved.
 *
 * Runs in IDLE (drives disabled) after zeroing at home and prints the
 * joint positions a few times a second for a while.  Move one joint at a
 * time by hand in the direction the WAM's sticker marks as positive and
 * watch whether the reported value goes up or down.
 *
 *   BARRETT_CONFIG_FILE=... ./lbc_position_watch [seconds=90]
 *
 * Never Shift-Activate during this; it is not needed and the program
 * never commands torque.
 */
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#include <barrett/standard_main_function.h>

using namespace barrett;

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	int seconds = argc > 1 ? atoi(argv[1]) : 90;

	printf("Drives stay disabled. Move ONE joint at a time in its sticker-positive "
			"direction and watch that column.\n");
	jp_type start = wam.getJointPositions();
	printf("columns: J1..J%zu absolute rad, then change since start\n", DOF);
	for (int i = 0; i < seconds * 2; ++i) {
		jp_type jp = wam.getJointPositions();
		printf("%5.1fs ", i * 0.5);
		for (size_t j = 0; j < DOF; ++j) printf(" %+6.3f", jp[j]);
		printf("   d:");
		for (size_t j = 0; j < DOF; ++j) printf(" %+6.3f", jp[j] - start[j]);
		printf("\n");
		fflush(stdout);
		usleep(500000);
	}
	printf("Done.\n");
	return 0;
}
