/* Minimal runtime probe for the CANopen libbarrett: construct a
 * ProductManager (which opens the CAN port and enumerates over SDO) and
 * report what it found.  Never wakes or activates anything.
 *
 *   BARRETT_CONFIG_FILE=/path/default.conf ./lbc_probe
 */
#include <cstdio>
#include <exception>
#include <barrett/products/product_manager.h>

int main(int argc, char** argv)
{
	const char* cfg = argc > 1 ? argv[1] : NULL;
	try {
		barrett::ProductManager pm(cfg);
		printf("ProductManager OK: safety module %d, wam4 %d, wam7 %d, wam8g %d, "
				"complete wam %d\n",
				(int)pm.foundSafetyModule(), (int)pm.foundWam4(), (int)pm.foundWam7(),
				(int)pm.foundWam8g(), (int)pm.foundWam());
	} catch (const std::exception& e) {
		printf("exception: %s\n", e.what());
		return 1;
	}
	return 0;
}
