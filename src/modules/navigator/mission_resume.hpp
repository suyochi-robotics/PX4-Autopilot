/**
 * mission_resume.hpp
 *
 * Mission Resume NavigatorMode.
 * - Loads MIS_RESUME_* params
 * - Handles arm/takeoff/goto/commit flow
 *
 * This header does not change navigator.h.
 */
#pragma once
#include "navigator_mode.h"

#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>

#include <lib/geo/geo.h>
#include <px4_platform_common/time.h>
#include <mathlib/mathlib.h>

namespace mission_resume
{

struct ResumeData {
    bool valid{false};
    int32_t index{0};
    double lat{0.0};
    double lon{0.0};
    float alt{0.0f};
    int32_t mission_id{0};
};

/** Load resume data from params. Returns true if MIS_RESUME_VALID==1 and params read. */
bool load_resume_data(ResumeData &r);

/** Clear resume params (MIS_RESUME_VALID = 0 ...) */
void clear_resume_data();

} // namespace mission_resume


class MissionResume : public NavigatorMode
{
public:
    explicit MissionResume(Navigator *navigator);
    ~MissionResume() override = default;

    // NavigatorMode API
    void initialize() override;
    void on_activation() override;
    void on_active() override;

private:
    enum class State {
        Start,
        WaitArm,
        Takeoff,
        Goto,
        Commit,
        Done,
        Fail
    };

    State _state{State::Start};
    hrt_abstime _state_start{0};
    mission_resume::ResumeData _r{};

    // Helpers
    void publish_arm_request();
    void publish_takeoff_setpoint(float target_alt_amsl);
    void publish_goto_setpoint(double lat, double lon, float alt);

    bool is_armed();
    bool is_landed();
    bool position_valid();
    bool altitude_valid();
    bool reached_alt(float target_amsl);
    bool reached_position(double lat, double lon, float alt);
    bool compute_distance_to_given_point(double lat, double lon, float alt, double &dist_xy_m, float &dist_z_m);

    // configuration
    float _accept_radius_m = 5.0f;
    float _accept_alt_m = 2.5f;
    int _takeoff_timeout_s = 30;
    int _goto_timeout_s = 120;
};
