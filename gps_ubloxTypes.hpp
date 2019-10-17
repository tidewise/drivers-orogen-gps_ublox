#ifndef gps_ublox_TYPES_HPP
#define gps_ublox_TYPES_HPP

#include <cstdint>
#include <gps_ublox/Driver.hpp>
#include <gps_ublox/UBX.hpp>

namespace gps_ublox {
    /**
     * Component configuration namespace
     */
    namespace configuration {
        /**
         * Rate of periodic messages sent by the device (per second, per epoch)
         */
        struct MessageRates {
            uint8_t nav_pvt = 1;
            uint8_t nav_sig = 10;
            uint8_t nav_sat = 10;
            uint8_t mon_rf = 10;
        };

        /**
         * Device's internal odometer configuration
         */
        struct Odometer {
            bool enabled = true;
            bool low_speed_course_over_ground_filter = true;
            bool output_low_pass_filtered_velocity = true;
            bool output_low_pass_filtered_heading = true;
            Driver::OdometerProfile odometer_profile = Driver::ODOM_SWIMMING;
            uint8_t upper_speed_limit_for_heading_filter = 5;
            uint8_t max_position_accuracy_for_low_speed_heading_filter = 5;
            uint8_t velocity_low_pass_filter_level = 1;
            uint8_t heading_low_pass_filter_level = 1;
        };

        /**
         * Navigtation solutions configuration
         */
        struct Navigation {

            base::Time position_measurement_period =
                base::Time::fromMilliseconds(1000);
            uint16_t measurements_per_solution_ratio = 1;
            UBX::TimeSystem time_system = UBX::GPS;
            UBX::DynamicModel dynamic_model = UBX::SEA;

            /** Speed below which the receiver is considered static (in m/s)
             *
             * Resolution is 1cm
             */
            float speed_threshold = 0;

            /** Distance above which GNSSbased stationary motion is exit (m) */
            int static_hold_distance_threshold;
        };
    }
}

#endif

