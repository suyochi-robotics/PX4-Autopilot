#include <px4_platform_common/param.h>


/**
 * Pump PWM at zero flow
 *
 * @unit us
 * @min 800
 * @max 2200
 * @reboot_required true
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(PUMP_MIN_PWM, 1050.f);

/**
 * Pump PWM at maximum flow
 *
 * @unit us
 * @min 800
 * @max 2200
 * @reboot_required true
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(PUMP_MAX_PWM, 1950.f);

/**
 * Pump flow at PUMP_MAX_PWM
 *
 * @min 0
 * @max 100
 * @reboot_required true
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(PUMP_MAX_FLOW, 8.f);

/**
 * Requested pump flow rate. A negative value disables the pump.
 *
 * @min -1
 * @max 100
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(PUMP_EXP_FLOW, 6.f);

/**
 * Centrifugal sprayer PWM at zero speed
 *
 * @unit us
 * @min 800
 * @max 2200
 * @reboot_required true
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(SPRAYER_MIN_PWM, 1050.f);

/**
 * Centrifugal sprayer PWM at 100 percent speed
 *
 * @unit us
 * @min 800
 * @max 2200
 * @reboot_required true
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(SPRAYER_MAX_PWM, 1950.f);

/**
 * Requested centrifugal sprayer speed. A negative value disables the sprayer.
 *
 * @unit %
 * @min -1
 * @max 100
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(SPRYAER_EXP_SPD, 100.f);

/**
 * Spray enable mode
 *
 * 0 = Disabled
 * 1 = Always ON
 * 2 = ON when vehicle armed
 * 3 = ON during AUTO mission
 * 4 = Manually enabled with SPRAY_EN_MAN
 *
 * @min 0
 * @max 4
 * @reboot_required true
 * @group Spray System
 */
PARAM_DEFINE_INT32(SPRAY_EN_MODE, 3);

/**
 * Manual spray enable, used when SPRAY_EN_MODE is 4
 *
 * @boolean
 * @min 0
 * @max 1
 * @group Spray System
 */
PARAM_DEFINE_INT32(SPRAY_EN_MAN, 0);
