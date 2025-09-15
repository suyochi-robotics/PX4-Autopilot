#pragma once

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

/**
 * @class ObstacleManager
 * @brief Simple obstacle avoidance helper.
 *
 * Listens to distance_sensor topic and pauses/resumes mission when
 * forward-facing obstacle is detected.
 */
class ObstacleManager
{
public:
    ObstacleManager();
    ~ObstacleManager() = default;

    /**
     * @brief Main run loop (blocking).
     * Polls distance_sensor and handles mission pause/resume.
     */
    void run();

private:
    int _distance_sub{-1};      ///< subscription handle for distance_sensor
    int _status_sub{-1};        ///< subscription handle for vehicle_status
    orb_advert_t _vehicle_command_pub{nullptr}; ///< publisher for vehicle_command
    orb_advert_t _mavlink_log_pub{nullptr};     ///< publisher for log messages

    bool _mission_paused{false}; ///< flag: mission currently paused

    /**
     * @brief Publish a VEHICLE_CMD message.
     * @param command Command ID (e.g. VEHICLE_CMD_DO_PAUSE_CONTINUE)
     * @param param1  Command parameter (0=pause, 1=continue)
     */
    void send_vehicle_command(uint16_t command, float param1);
};
