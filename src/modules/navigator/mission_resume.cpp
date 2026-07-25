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
/**
 * mission_resume.cpp
 *
 * NavigatorMode that performs a resume mission flow:
 *  - validate resume params and mission
 *  - optionally request arm
 *  - takeoff if landed
 *  - goto resume point
 *  - commit mission by sending VEHICLE_CMD_MISSION_START with param1 = resume_index
 *
 * This implementation does NOT modify navigator.h.
 */

#include "navigator.h"
#include "mission_resume.hpp"

#include <px4_platform_common/log.h>
#include <uORB/Publication.hpp>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <math.h>

static constexpr float DEFAULT_MAX_DIST_M = 1000.0f;
static constexpr float MIN_TAKEOFF_ALT = 10.0f; // meters AGL or AMSL depending on your alt reference
static constexpr hrt_abstime MAX_POSITION_AGE_US = 2_s;
static constexpr hrt_abstime MAX_LAND_DETECTION_AGE_US = 2_s;

/* ---------------------- params helpers ---------------------- */

bool mission_resume::load_resume_data(ResumeData &r)
{
	r = {};
	int32_t valid = 0;

	if (param_get(param_find("MIS_RSM_VALID"), &valid) != PX4_OK) {
		return false;
	}

	if (valid != 1) {
		return false;
	}

	int32_t idx = 0;
	int32_t lat_i = 0;
	int32_t lon_i = 0;
	float alt_f = 0.0f;
	int32_t mid = 0;

	if (param_get(param_find("MIS_RSM_IDX"), &idx) != PX4_OK
	    || param_get(param_find("MIS_RSM_LAT"), &lat_i) != PX4_OK
	    || param_get(param_find("MIS_RSM_LON"), &lon_i) != PX4_OK
	    || param_get(param_find("MIS_RSM_ALT"), &alt_f) != PX4_OK
	    || param_get(param_find("MIS_RSM_MID"), &mid) != PX4_OK) {
		return false;
	}

	r.valid = true;
	r.index = idx;
	r.lat = static_cast<double>(lat_i) / 1e7;
	r.lon = static_cast<double>(lon_i) / 1e7;
	r.alt = alt_f;
	r.mission_id = mid;

	return resume_data_valid(r);
}

bool mission_resume::resume_data_valid(const ResumeData &r)
{
	return r.valid
	       && r.index >= 0
	       && r.mission_id != 0
	       && PX4_ISFINITE(r.lat) && r.lat >= -90.0 && r.lat <= 90.0
	       && PX4_ISFINITE(r.lon) && r.lon >= -180.0 && r.lon <= 180.0
	       && PX4_ISFINITE(r.alt);
}

void mission_resume::clear_resume_data()
{
	int32_t z = 0;
	float fz = 0.0f;

	// Invalidate first. This also asks the normal parameter autosave mechanism
	// to persist the fact that this record must not be reused after a reboot.
	param_set(param_find("MIS_RSM_VALID"), &z);
	param_set(param_find("MIS_RSM_IDX"), &z);
	param_set(param_find("MIS_RSM_LAT"), &z);
	param_set(param_find("MIS_RSM_LON"), &z);
	param_set(param_find("MIS_RSM_MID"), &z);
	param_set(param_find("MIS_RSM_TS"), &z);
	param_set(param_find("MIS_RSM_ALT"), &fz);
}

/* ---------------------- MissionResume class ---------------------- */

MissionResume::MissionResume(Navigator *navigator)
	: NavigatorMode(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
{
}

void MissionResume::initialize()
{
	// Nothing required; initialization happens in on_activation()
}

void MissionResume::on_activation()
{
	PX4_INFO("MissionResume: activated");
	_state = State::Start;
	_state_start = hrt_absolute_time();
	_last_arm_request = 0;
	_failure_reported = false;

	// Load data once during activation
	if (!mission_resume::load_resume_data(_r) || !_r.valid) {
		fail_resume("resume record is missing or invalid");
		return;
	}
}

bool MissionResume::position_valid()
{
	const vehicle_global_position_s *gpos = _navigator->get_global_position();
	return (gpos && gpos->lat_lon_valid && PX4_ISFINITE(gpos->lat) && PX4_ISFINITE(gpos->lon)
		&& hrt_elapsed_time(&gpos->timestamp) <= MAX_POSITION_AGE_US);
}

bool MissionResume::altitude_valid()
{
	const vehicle_global_position_s *gpos = _navigator->get_global_position();
	return (gpos && gpos->alt_valid && PX4_ISFINITE(gpos->alt)
		&& hrt_elapsed_time(&gpos->timestamp) <= MAX_POSITION_AGE_US);
}

bool MissionResume::is_armed()
{
	const vehicle_status_s *v = _navigator->get_vstatus();
	return (v && v->arming_state == vehicle_status_s::ARMING_STATE_ARMED);
}

bool MissionResume::is_landed()
{
	const vehicle_land_detected_s *ld = _navigator->get_land_detected();
	return (ld && ld->landed);
}

bool MissionResume::land_detection_valid()
{
	const vehicle_land_detected_s *ld = _navigator->get_land_detected();
	return ld && ld->timestamp != 0 && hrt_elapsed_time(&ld->timestamp) <= MAX_LAND_DETECTION_AGE_US;
}

bool MissionResume::reached_alt(float target_amsl)
{
	const vehicle_global_position_s *gpos = _navigator->get_global_position();

	if (!gpos) { return false; }

	return fabsf(gpos->alt - target_amsl) <= _accept_alt_m;
}

bool MissionResume::reached_position(double lat, double lon, float alt)
{
	double d_xy = 0.0;
	float  d_z  = 0.0f;

	if (!compute_distance_to_given_point(lat, lon, alt, d_xy, d_z)) {
		return false;
	}

	const double accept_r = static_cast<double>(_accept_radius_m);
	const double accept_z = static_cast<double>(_accept_alt_m);

	return (d_xy <= accept_r) && (static_cast<double>(d_z) <= accept_z);
}

/* publish an arm request (broadcast) */
void MissionResume::publish_arm_request()
{
	uORB::Publication<vehicle_command_s> pub{ORB_ID(vehicle_command)};
	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	cmd.param1 = 1.0f; // arm
	cmd.target_system = 0;    // broadcast
	cmd.target_component = 0; // broadcast
	pub.publish(cmd);

	PX4_INFO("MissionResume: requested arm");
}

/* publish a climb-only setpoint (keep lat/lon NAN to indicate vertical climb) */
void MissionResume::publish_takeoff_setpoint(float target_alt_amsl)
{
	position_setpoint_triplet_s *t = _navigator->get_position_setpoint_triplet();

	// Clear the entire triplet
	memset(t, 0, sizeof(position_setpoint_triplet_s));

	// CURRENT setpoint (vertical climb only)
	t->current.valid = true;
	t->current.type  = position_setpoint_s::SETPOINT_TYPE_POSITION;
	t->current.lat   = (double)NAN;             // no horizontal movement
	t->current.lon   = (double)NAN;
	t->current.alt   = target_alt_amsl;
	t->current.yaw   = NAN;
	t->current.acceptance_radius = _accept_radius_m;

	// PREVIOUS + NEXT must be marked invalid
	t->previous.valid = false;
	t->next.valid     = false;

	t->timestamp = hrt_absolute_time();

	// Tell Navigator to publish the triplet on next cycle
	_navigator->set_position_setpoint_triplet_updated();
}


/* publish goto setpoint */
void MissionResume::publish_goto_setpoint(double lat, double lon, float alt)
{
	position_setpoint_triplet_s *t = _navigator->get_position_setpoint_triplet();

	// Clear full triplet
	memset(t, 0, sizeof(position_setpoint_triplet_s));

	// CURRENT GOTO setpoint
	t->current.valid = true;
	t->current.type  = position_setpoint_s::SETPOINT_TYPE_POSITION;
	t->current.lat   = lat;
	t->current.lon   = lon;
	t->current.alt   = alt;
	t->current.yaw   = NAN;
	t->current.acceptance_radius = _accept_radius_m;

	// No previous/next WP for GOTO
	t->previous.valid = false;
	t->next.valid     = false;

	t->timestamp = hrt_absolute_time();

	// Notify Navigator
	_navigator->set_position_setpoint_triplet_updated();
}

bool MissionResume::compute_distance_to_given_point(
	double lat, double lon, float alt,
	double &dist_xy_m, float &dist_z_m)
{
	const vehicle_global_position_s *gpos = _navigator->get_global_position();

	if (!gpos) {
		dist_xy_m = (double)NAN;
		dist_z_m  = (double)NAN;
		return false;
	}

	float xy = 0.0f;
	float z  = 0.0f;

	// PX4 geo API: computes horizontal (xy) and vertical (z) distance
	get_distance_to_point_global_wgs84(
		gpos->lat, gpos->lon, gpos->alt,
		lat, lon, alt,
		&xy, &z
	);

	// promote to correct types explicitly (prevents -Wdouble-promotion)
	dist_xy_m = static_cast<double>(xy);
	dist_z_m  = fabsf(z);

	return PX4_ISFINITE(dist_xy_m) && PX4_ISFINITE(dist_z_m);
}

void MissionResume::fail_resume(const char *reason)
{
	if (_failure_reported) {
		return;
	}

	_failure_reported = true;
	_state = State::Fail;
	PX4_WARN("MissionResume: %s", reason);

	// Do not leave a bad record selecting this mode forever. The next explicit
	// AUTO.MISSION request can use a newly saved, valid record.
	mission_resume::clear_resume_data();
	_navigator->reset_triplets();

	// A failed airborne resume must not fall through to normal mission control.
	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM;
	_navigator->publish_vehicle_command(command);
}

/* -------------------- State machine -------------------- */

void MissionResume::on_active()
{
	// If position not valid, fail early
	if (!position_valid()) {
		fail_resume("global position is invalid or stale");
		return;
	}

	// If altitude not valid, fail early
	if (!altitude_valid()) {
		fail_resume("altitude estimate is invalid or stale");
		return;
	}

	// Access mission_result via navigator getter
	const mission_result_s *mr = _navigator->get_mission_result();

	if (!mr || !mr->valid || mr->seq_total == 0) {
		fail_resume("mission is unavailable or invalid");
		return;
	}

	if (_r.mission_id == 0) {
		fail_resume("saved mission ID is invalid");
		return;
	}

	if (static_cast<uint32_t>(_r.mission_id) != mr->mission_id) {
		fail_resume("mission changed since the resume point was saved");
		return;
	}

	if (static_cast<uint32_t>(_r.index) >= mr->seq_total) {
		fail_resume("saved mission index is outside the current mission");
		return;
	}

	// Load configurable max distance
	float max_dist = DEFAULT_MAX_DIST_M;
	float max_alt_dist = 20.0f;

	if (param_get(param_find("MIS_RSM_MAX_DST"), &max_dist) != PX4_OK
	    || param_get(param_find("MIS_RSM_MAX_ALT"), &max_alt_dist) != PX4_OK
	    || !PX4_ISFINITE(max_dist) || max_dist <= 0.0f
	    || !PX4_ISFINITE(max_alt_dist) || max_alt_dist <= 0.0f) {
		fail_resume("resume distance limits are invalid");
		return;
	}

	double dist_xy = 0.0;
	float  dist_z  = 0.0f;

	if (!compute_distance_to_given_point(_r.lat, _r.lon, _r.alt, dist_xy, dist_z)) {
		fail_resume("distance to the resume point is invalid");
		return;
	}

	double dist_to_resume = dist_xy;

	switch (_state) {

	case State::Start: {
			// Distance check
			if (dist_to_resume > static_cast<double>(max_dist)) {
				fail_resume("resume point exceeds the horizontal distance limit");
				return;
			}

			if (dist_z > max_alt_dist) {
				fail_resume("resume point exceeds the vertical distance limit");
				return;
			}

			if (!land_detection_valid()) {
				fail_resume("land detector state is invalid or stale");
				return;
			}

			const vehicle_status_s *vstatus = _navigator->get_vstatus();

			if (is_landed() && vstatus
			    && vstatus->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
			    && !vstatus->is_vtol) {
				fail_resume("landed fixed-wing mission resume is unsupported");
				return;
			}

			// Decide next state depending on armed/landed
			if (!is_armed()) {
				_state = State::WaitArm;
				_state_start = hrt_absolute_time();
				PX4_INFO("MissionResume: waiting for arm");

			} else if (is_landed()) {
				_state = State::Takeoff;
				_state_start = hrt_absolute_time();
				PX4_INFO("MissionResume: starting takeoff to %.2f m", (double)_r.alt);

			} else {
				_state = State::Goto;
				_state_start = hrt_absolute_time();
				PX4_INFO("MissionResume: airborne, going to resume point");
			}

			break;
		}

	case State::WaitArm: {
			int32_t arm_en = 0;

			if (param_get(param_find("MIS_RSM_ARM_EN"), &arm_en) != PX4_OK) {
				fail_resume("auto-arm setting is unavailable");
				return;
			}

			if (!is_armed()) {
				if (arm_en == 1) {
					if (_last_arm_request == 0 || hrt_elapsed_time(&_last_arm_request) >= 1_s) {
						publish_arm_request();
						_last_arm_request = hrt_absolute_time();
					}

				} else {
					// Manual arming is allowed when automatic arming is disabled.
					return;
				}
			}

			// wait for arm for timeout
			if (is_armed()) {
				// proceed to takeoff if landed, otherwise goto
				if (is_landed()) {
					_state = State::Takeoff;
					_state_start = hrt_absolute_time();

				} else {
					_state = State::Goto;
					_state_start = hrt_absolute_time();
				}

			} else if (hrt_elapsed_time(&_state_start) > static_cast<hrt_abstime>(_takeoff_timeout_s * 1_s)) {
				fail_resume("arming timed out");
				return;
			}

			break;
		}

	case State::Takeoff: {
			// Read MIS_TAKEOFF_ALT parameter (relative altitude above home)
			float mis_takeoff_rel = MIN_TAKEOFF_ALT;
			param_get(param_find("MIS_TAKEOFF_ALT"), &mis_takeoff_rel);

			// Get home altitude AMSL
			const home_position_s *home = _navigator->get_home_position();

			if (!home || !home->valid_alt || !PX4_ISFINITE(home->alt) || !PX4_ISFINITE(mis_takeoff_rel)) {
				fail_resume("home altitude or takeoff altitude is invalid");
				return;
			}

			float mis_takeoff_amsl = home->alt + mis_takeoff_rel;

			// Now both values are AMSL → safe to compare
			float climb_alt = math::max(_r.alt, mis_takeoff_amsl);

			publish_takeoff_setpoint(climb_alt);

			if (reached_alt(climb_alt)) {
				PX4_INFO("MissionResume: reached climb alt %.2f", (double)climb_alt);
				_state = State::Goto;
				_state_start = hrt_absolute_time();

			} else if (hrt_elapsed_time(&_state_start) >
				   static_cast<hrt_abstime>(_takeoff_timeout_s * 1_s)) {
				fail_resume("takeoff timed out");
				return;
			}

			break;
		}

	case State::Goto: {
			publish_goto_setpoint(_r.lat, _r.lon, _r.alt);

			if (reached_position(_r.lat, _r.lon, _r.alt)) {
				PX4_INFO("MissionResume: arrived at resume point");
				_state = State::Commit;
				_state_start = hrt_absolute_time();

			} else if (hrt_elapsed_time(&_state_start) > static_cast<hrt_abstime>(_goto_timeout_s * 1_s)) {
				fail_resume("goto resume point timed out");
				return;
			}

			break;
		}

	case State::Commit: {
			PX4_INFO("MissionResume: committing mission start index %d", (int)_r.index);

			if (!_navigator->set_mission_current_index(static_cast<uint16_t>(_r.index))) {
				fail_resume("mission rejected the saved resume index");
				return;
			}

			mission_resume::clear_resume_data();

			position_setpoint_triplet_s *sp = _navigator->get_position_setpoint_triplet();
			*sp = position_setpoint_triplet_s();    // reset triplet
			_navigator->set_position_setpoint_triplet_updated();

			_state = State::Done;
			break;
		}

	case State::Done:
		// Nothing to do. Let navigator continue the mission.
		break;

	case State::Fail:
	default:
		PX4_WARN("MissionResume: failed state");
		break;
	}
}
