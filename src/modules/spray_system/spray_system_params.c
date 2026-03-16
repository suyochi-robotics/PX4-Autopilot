#include <px4_platform_common/param.h>


/**
 * Spray pump capacity in liters per minute
 *
 * @min 3.0
 * @max 8.0
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(SPRAY_PUMP_LPM, 5.0f);

/**
 * Centrifugal spray system speed in RPM
 *
 * @min 1000
 * @max 20000
 * @unit rpm
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(SPRAY_CENT_RPM, 10000.0f);

/**
 * Spray enable mode
 *
 * 1 = Always ON
 * 2 = ON when vehicle armed
 * 3 = ON during AUTO mission
 *
 * @min 1
 * @max 3
 * @group Spray System
 */
PARAM_DEFINE_INT32(SPRAY_EN_MODE, 3);
