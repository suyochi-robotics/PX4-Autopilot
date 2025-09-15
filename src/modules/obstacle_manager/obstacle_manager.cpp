#include "obstacle_manager.hpp"

ObstacleManager::ObstacleManager()
{
    _distance_sub = orb_subscribe(ORB_ID(distance_sensor));
    _status_sub   = orb_subscribe(ORB_ID(vehicle_status));
}

void ObstacleManager::send_vehicle_command(uint16_t command, float param1)
{
    vehicle_command_s cmd{};
    cmd.command = command;
    cmd.param1 = param1;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.timestamp = hrt_absolute_time();

    if (_vehicle_command_pub == nullptr) {
        _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
    } else {
        orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &cmd);
    }
}

void ObstacleManager::switch_to_posctl()
{
    vehicle_command_s cmd{};
    cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1; // custom mode enabled
    cmd.param2 = PX4_CUSTOM_MAIN_MODE_POSCTL;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.timestamp = hrt_absolute_time();

    if (_vehicle_command_pub == nullptr) {
        _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
    } else {
        orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &cmd);
    }gi

    mavlink_log_info(&_mavlink_log_pub, "Switched to Position mode for manual control");
}

void ObstacleManager::run()
{
    px4_pollfd_struct_t fds[1];
    fds[0].fd = _distance_sub;
    fds[0].events = POLLIN;

    while (!px4_task_should_exit()) {
        int ret = px4_poll(fds, 1, 200); // 200 ms timeout

        if (ret > 0 && (fds[0].revents & POLLIN)) {
            distance_sensor_s dist{};
            vehicle_status_s status{};

            orb_copy(ORB_ID(distance_sensor), _distance_sub, &dist);
            orb_copy(ORB_ID(vehicle_status), _status_sub, &status);

            // Only act in AUTO.MISSION and forward-facing sensor
            if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION &&
                dist.orientation == distance_sensor_s::ROTATION_FORWARD_FACING) {

                float threshold = 3.0f; // meters

                if (PX4_ISFINITE(dist.current_distance) && dist.current_distance < threshold) {
                    if (!_mission_paused) {
                        // Pause mission
                        mavlink_log_info(&_mavlink_log_pub, "Obstacle ahead: Pausing mission");
                        send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE, 0.0f);
                        _mission_paused = true;

                        // Switch to manual Position mode
                        switch_to_posctl();
                    }
                }
            }

            // Wait until pilot switches back into AUTO.MISSION
            if (_mission_paused &&
                status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {

                mavlink_log_info(&_mavlink_log_pub, "AUTO mode re-engaged, mission will continue");
                send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE, 1.0f);
                _mission_paused = false;
            }
        }
    }
}

extern "C" __EXPORT int obstacle_manager_main(int argc, char *argv[])
{
    ObstacleManager manager;
    manager.run();
    return 0;
}
