#ifndef gps_ublox_TYPES_HPP
#define gps_ublox_TYPES_HPP

#include <cstdint>
#include <gps_ublox/cfg.hpp>
#include <gps_ublox/RTKInfo.hpp>

namespace gps_ublox {
    /**
     * Component configuration namespace
     */
    namespace configuration {
        /**
         * Rate of periodic messages sent by the device (per second, per epoch)
         *
         * Set a rate to zero to disable
         */
        struct MessageRates {
            /** NAV-PVT message period in solution periods. Controls pose_samples
             * and gps_solution outputs
             */
            uint8_t nav_pvt = 1;
            /** NAV-RELPOSNED message period in solution periods. Controls
             * the rtk_relative_position_samples output
             */
            uint8_t nav_relposned = 0;
            /** NAV-SIG message period in solution periods. Controls
             * pose_samples and gps_solution outputs
             */
            uint8_t nav_sig = 10;
            /** NAV-SAT message period in solution periods. Controls
             * pose_samples and gps_solution outputs
             */
            uint8_t nav_sat = 10;
            /** MON-RF message period in solution periods. Controls
             * pose_samples and gps_solution outputs
             */
            uint8_t mon_rf = 10;
            /** Control of the rtk_info output
             *
             * This is actually an aggregate of messages. Part of the structure
             * is filled with NAV-SAT (controlled with nav_sat)
             */
            uint8_t rtk_info = 0;
        };

        /**
         * Device's internal odometer configuration
         */
        struct Odometer {
            bool enabled = true;
            bool low_speed_course_over_ground_filter = true;
            bool output_low_pass_filtered_velocity = true;
            bool output_low_pass_filtered_heading = true;
            OdometerProfile odometer_profile = ODOM_SWIMMING;
            uint8_t upper_speed_limit_for_heading_filter = 5;
            uint8_t max_position_accuracy_for_low_speed_heading_filter = 5;
            uint8_t velocity_low_pass_filter_level = 1;
            uint8_t heading_low_pass_filter_level = 1;
        };

        /**
         * Navigtation solutions configuration
         */
        struct Navigation {

            base::Time position_measurement_period = base::Time::fromMilliseconds(1000);
            uint16_t measurements_per_solution_ratio = 1;
            MeasurementRefTime measurement_ref_time = MEASUREMENT_REF_TIME_GPS;
            DynamicModel dynamic_model = DYNAMIC_MODEL_PORTABLE;

            /** Speed below which the receiver is considered static (in m/s)
             *
             * Resolution is 1cm
             */
            float speed_threshold = 0;

            /** Distance above which GNSSbased stationary motion is exit (m) */
            int static_hold_distance_threshold = 0;
        };
    }
}

#endif

