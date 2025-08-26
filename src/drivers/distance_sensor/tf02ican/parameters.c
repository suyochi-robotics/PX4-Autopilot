/****************************************************************************
 * Parameters for TF02-i CAN distance sensor
 ****************************************************************************/

/**
 * Enable TF02-i CAN driver
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_EN_TF02IC, 0);

/**
 * TF02-i CAN receiving ID (standard frame, 11-bit)
 *
 * Default: 3
 * @min 0
 * @max 2047
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TF02IC_ID, 3);

/**
 * TF02-i CAN baud rate in bit/s
 *
 * Default: 115200
 * @min 10000
 * @max 1000000
 * @unit bit/s
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TF02IC_BAUD, 115200);

/**
 * TF02-i CAN measurement interval
 *
 * Default: 100 ms
 * @min 10
 * @max 1000
 * @unit ms
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TF02IC_INT, 100);

/**
 * TF02-i CAN maximum distance
 *
 * Default: 4.0 m
 * @min 0.2
 * @max 45
 * @unit m
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SENS_TF02IC_MAX, 40.0f);

/**
 * TF02-i CAN minimum distance
 *
 * Default: 0.01 m
 * @min 0.01
 * @max 0.2
 * @unit m
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SENS_TF02IC_MIN, 0.1f);
