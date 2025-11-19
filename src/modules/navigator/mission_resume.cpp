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

using namespace mission_resume;

static constexpr float DEFAULT_MAX_DIST_M = 1000.0f;
static constexpr float MIN_TAKEOFF_ALT = 10.0f; // meters AGL or AMSL depending on your alt reference

/* ---------------------- params helpers ---------------------- */

bool mission_resume::load_resume_data(ResumeData &r)
{
	int32_t valid = 0;
	param_get(param_find("MIS_RSM_VALID"), &valid);

	if (valid != 1) {
		return false;
	}

	int32_t idx = 0;
	int32_t lat_i = 0;
	int32_t lon_i = 0;
	float alt_f = 0.0f;
	int32_t mid = 0;

	param_get(param_find("MIS_RSM_IDX"), &idx);
	param_get(param_find("MIS_RSM_LAT"), &lat_i);
	param_get(param_find("MIS_RSM_LON"), &lon_i);
	param_get(param_find("MIS_RSM_ALT"), &alt_f);
	param_get(param_find("MIS_RSM_MID"), &mid);

	r.valid = true;
	r.index = idx;
	r.lat = static_cast<double>(lat_i) / 1e7;
	r.lon = static_cast<double>(lon_i) / 1e7;
	r.alt = alt_f;
	r.mission_id = mid;

	return true;
}

void mission_resume::clear_resume_data()
{
	int32_t z = 0;
	float fz = 0.0f;

	param_set_no_notification(param_find("MIS_RSM_VALID"), &z);
	param_set_no_notification(param_find("MIS_RSM_IDX"), &z);
	param_set_no_notification(param_find("MIS_RSM_LAT"), &z);
	param_set_no_notification(param_find("MIS_RSM_LON"), &z);
	param_set_no_notification(param_find("MIS_RSM_MID"), &z);
	param_set_no_notification(param_find("MIS_RSM_TS"), &z);
	param_set_no_notification(param_find("MIS_RSM_ALT"), &fz);
}

/* ---------------------- MissionResume class ---------------------- */

MissionResume::MissionResume(Navigator *navigator)
    : NavigatorMode(navigator)
{
    // keep defaults; optionally override from params in initialize() if needed
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

	// Load data once during activation
	if (!mission_resume::load_resume_data(_r) || !_r.valid) {
		PX4_WARN("MissionResume: no resume data");
		_state = State::Fail;
		return;
	}
}

bool MissionResume::position_valid()
{
	const vehicle_global_position_s *gpos = _navigator->get_global_position();
	return (gpos && PX4_ISFINITE(gpos->lat) && PX4_ISFINITE(gpos->lon));
}

bool MissionResume::altitude_valid()
{
	const vehicle_global_position_s *gpos = _navigator->get_global_position();
	return (gpos && PX4_ISFINITE(gpos->alt));
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
	t->current.lat   = NAN;                   // no horizontal movement
	t->current.lon   = NAN;
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
        dist_xy_m = NAN;
        dist_z_m  = NAN;
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

    return true;
}

/* -------------------- State machine -------------------- */

void MissionResume::on_active()
{
	// If position not valid, fail early
	if (!position_valid()) {
		PX4_WARN("MissionResume: no valid global position");
		return;
	}

	// If altitude not valid, fail early
	if (!altitude_valid()) {
		PX4_WARN("MissionResume: no valid altitude");
		return;
	}

	// Access mission_result via navigator getter
	const mission_result_s *mr = _navigator->get_mission_result();
	if (!mr || !mr->valid || mr->seq_total == 0) {
		PX4_WARN("MissionResume: no mission available");
		return;
	}

	if (_r.mission_id == 0) {
		PX4_WARN("MissionResume: no saved mission ID");
		return;
	}

	if (static_cast<uint32_t>(_r.mission_id) != mr->mission_id) {
		PX4_WARN("MissionResume: mission id changed (%u != %u)", static_cast<uint32_t>(_r.mission_id), mr->mission_id);
		return;
	}

	if (static_cast<uint32_t>(_r.index) >= mr->seq_total) {
		PX4_WARN("MissionResume: resume index %d >= seq_total %u", _r.index, mr->seq_total);
		return;
	}

	// Load configurable max distance
	float max_dist = DEFAULT_MAX_DIST_M;
	param_get(param_find("MIS_RSM_MAX_DST"), &max_dist);

	double dist_xy = 0.0;
	float  dist_z  = 0.0f;

	if (!compute_distance_to_given_point(_r.lat, _r.lon, _r.alt, dist_xy, dist_z)) {
		return;
	}

	double dist_to_resume = dist_xy;

	switch (_state) {

	case State::Start:
	{
		// Distance check
		if (dist_to_resume > static_cast<double>(max_dist)) {
			PX4_WARN("MissionResume: resume distance %.1f m > allowed %.1f m", dist_to_resume, static_cast<double>(max_dist));
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

	case State::WaitArm:
	{
		int32_t arm_en = 0;
		param_get(param_find("MIS_RSM_ARM_EN"), &arm_en);

		if (!is_armed()) {
		if (arm_en == 1) {
			publish_arm_request();
		} else {
			PX4_WARN("MissionResume: auto-arm disabled; aborting");
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
			PX4_WARN("MissionResume: arming timeout");
			return;
		}
		break;
	}

	case State::Takeoff:
	{
		// Read MIS_TAKEOFF_ALT parameter (relative altitude above home)
		float mis_takeoff_rel = MIN_TAKEOFF_ALT;
		param_get(param_find("MIS_TAKEOFF_ALT"), &mis_takeoff_rel);

		// Get home altitude AMSL
		const home_position_s *home = _navigator->get_home_position();
		float mis_takeoff_amsl = home ? (home->alt + mis_takeoff_rel)
						: (_r.alt); // fallback if no home

		// Now both values are AMSL → safe to compare
		float climb_alt = math::max(_r.alt, mis_takeoff_amsl);

		publish_takeoff_setpoint(climb_alt);

		if (reached_alt(climb_alt)) {
			PX4_INFO("MissionResume: reached climb alt %.2f", (double)climb_alt);
			_state = State::Goto;
			_state_start = hrt_absolute_time();
		} else if (hrt_elapsed_time(&_state_start) >
			static_cast<hrt_abstime>(_takeoff_timeout_s * 1_s)) {
			PX4_WARN("MissionResume: takeoff timeout");
			return;
		}
		break;
	}

	case State::Goto:
	{
		publish_goto_setpoint(_r.lat, _r.lon, _r.alt);

		if (reached_position(_r.lat, _r.lon, _r.alt)) {
			PX4_INFO("MissionResume: arrived at resume point");
			_state = State::Commit;
			_state_start = hrt_absolute_time();
		} else if (hrt_elapsed_time(&_state_start) > static_cast<hrt_abstime>(_goto_timeout_s * 1_s)) {
			PX4_WARN("MissionResume: goto timeout");
			return;
		}
		break;
	}

	case State::Commit:
	{
		PX4_INFO("MissionResume: committing mission start index %d", (int)_r.index);

		mission_result_s *mr_updatable = _navigator->get_mission_result();
		mr_updatable->seq_current = _r.index;
		mr_updatable->valid = true;
		mr_updatable->finished = false;
		// Do NOT publish here. Navigator publishes mission_result automatically.

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
