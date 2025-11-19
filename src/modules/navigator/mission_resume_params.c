/****************************************************************************
 *
 *   Mission Resume Parameters (<=16 char names)
 *
 *   These parameters store the information required to resume a Mission
 *   from the last known state after leaving Mission mode. They are written
 *   by Commander when exiting AUTO.MISSION and consumed by Navigator’s
 *   MissionResume module when resuming.
 *
 ****************************************************************************/

#include <px4_platform_common/param.h>

/**
 * Mission Resume: Valid Flag
 *
 * Indicates whether mission resume data is available.
 *
 * 0 - No resume data saved
 * 1 - Resume data valid and ready for use
 *
 * @boolean
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_RSM_VALID, 0);

/**
 * Mission Resume: Next Mission Item Index
 *
 * Stores the mission item index from which the mission should continue.
 *
 * @min 0
 * @max 100000
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_RSM_IDX, 0);

/**
 * Mission Resume: Latitude
 *
 * Latitude at the moment Mission mode was exited.
 * Stored as int32 (degrees * 1e7).
 *
 * NOTE: No @unit tag here because the value is an integer-encoded latitude
 *       in degrees * 1e7 (MAVLink / mission item format).
 *
 * @min -900000000
 * @max  900000000
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_RSM_LAT, 0);

/**
 * Mission Resume: Longitude
 *
 * Longitude at moment Mission mode was exited.
 * Stored as int32 (degrees * 1e7).
 *
 * NOTE: No @unit tag here because the value is an integer-encoded longitude
 *       in degrees * 1e7 (MAVLink / mission item format).
 *
 * @min -1800000000
 * @max  1800000000
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_RSM_LON, 0);

/**
 * Mission Resume: Altitude
 *
 * Altitude in meters when Mission mode was exited.
 *
 * @unit m
 * @min -2000
 * @max 10000
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_RSM_ALT, 0.0f);

/**
 * Mission Resume: Mission ID Snapshot
 *
 * Used to verify the mission has not changed before resuming.
 *
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_RSM_MID, 0);

/**
 * Mission Resume: Auto Arm Enable
 *
 * 0 - Disabled (manual arm required)
 * 1 - Navigator auto-arms before resume
 *
 * @boolean
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_RSM_ARM_EN, 0);

/**
 * Mission Resume: Maximum Resume Distance
 *
 * Maximum allowed distance between current vehicle location and
 * resume point. Resume denied if exceeded.
 *
 * Default: 1000 meters
 *
 * @unit m
 * @min 10
 * @max 5000
 * @decimal 1
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_RSM_MAX_DST, 1000.0f);

/**
 * Mission Resume: Timestamp
 *
 * Timestamp (seconds since boot) when resume parameters were saved.
 *
 * @unit s
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_RSM_TS, 0);
