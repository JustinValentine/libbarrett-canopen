// Prints the joint positions of a wam8g in both of its views.  No torque is
// commanded and gravity compensation is off: support the arm when activating.
#include <iostream>
#include <barrett/detail/stl_utils.h>
#define BARRETT_SMF_WAM8G
#include <barrett/standard_main_function.h>

int wam_main(int, char**, barrett::ProductManager& pm, barrett::systems::Wam8g& wam)
{
	std::cout << "Measured [J1,J2,J3,J4,J5,J6,J7,J8]: " << wam.getJointPositions() << '\n';
	std::cout << "Active   [J1,J2,J3,J4,J6,J7,J8]:    " << wam.getActiveJointPositions() << '\n';
	std::cout << "Seven torque inputs; J5 is sensor-only. Press Enter to finish.\n";
	barrett::detail::waitForEnter();
	pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);
	return 0;
}
