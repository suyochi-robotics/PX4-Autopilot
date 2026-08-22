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
 * Requested pump speed. A negative value disables the pump.
 *
 * @unit %
 * @min -1
 * @max 100
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(PUMP_EXP_SPD, 100.f);

/**
 * Minimum acceptable liquid flow while the spray flow failsafe is active.
 *
 * @min 0
 * @max 100
 * @decimal 3
 * @reboot_required true
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(SPRY_FLOW_MIN, 0.f);

/**
 * Maximum duration of low flow before the spray flow failsafe requests Return mode.
 *
 * @unit s
 * @min 0.1
 * @max 60
 * @decimal 1
 * @reboot_required true
 * @group Spray System
 */
PARAM_DEFINE_FLOAT(SPRY_FLOW_TOUT, 5.f);

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
 * Start the spray system driver during system initialization.
 *
 * @boolean
 * @min 0
 * @max 1
 * @reboot_required true
 * @group Spray System
 */
PARAM_DEFINE_INT32(SPRAY_ENABLE, 0);

/**
 * Manually enable spray output.
 *
 * @boolean
 * @min 0
 * @max 1
 * @group Spray System
 */
PARAM_DEFINE_INT32(SPRAY_EN_MAN, 0);
