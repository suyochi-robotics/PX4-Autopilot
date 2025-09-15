#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#include <lib/mavlink/mavlink_log.h>
#include <lib/geo/geo.h>

using namespace time_literals;

class ObstacleManager
{
public:
    ObstacleManager();
    ~ObstacleManager() = default;

    void run();

private:
    int _distance_sub{-1};
    int _status_sub{-1};
    orb_advert_t _vehicle_command_pub{nullptr};

    bool _mission_paused{false};

    void send_vehicle_command(uint16_t command, float param1);
};

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

            // Only act in AUTO.MISSION mode
            if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
                float threshold = 3.0f; // meters

                if (PX4_ISFINITE(dist.current_distance) && dist.current_distance < threshold) {
                    if (!_mission_paused) {
                        PX4_INFO("Obstacle detected: Pausing mission");
                        send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE, 0.0f);
                        _mission_paused = true;
                    }
                } else {
                    if (_mission_paused) {
                        PX4_INFO("Obstacle cleared: Resuming mission");
                        send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE, 1.0f);
                        _mission_paused = false;
                    }
                }
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
