/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

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
