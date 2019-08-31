/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <iodrivers_base/ConfigureGuard.hpp>
#include <gps_ublox/Driver.hpp>

using namespace gps_ublox;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

void Task::loadConfiguration() {
    Driver::DevicePort port = _device_port.get();

    // Message rates
    configuration::MessageRates rates = _msg_rates.get();
    mDriver->setOutputRate(port, Driver::MSGOUT_MON_RF, rates.mon_rf, false);
    mDriver->setOutputRate(port, Driver::MSGOUT_NAV_PVT, rates.nav_pvt, false);
    mDriver->setOutputRate(port, Driver::MSGOUT_NAV_SIG, rates.nav_sig, false);

    // Odometer configuration
    configuration::Odometer odom_cfg = _odometer_configuration.get();
    mDriver->setOdometer(odom_cfg.enabled, false);
    mDriver->setLowSpeedCourseOverGroundFilter(odom_cfg.low_speed_course_over_ground_filter, false);
    mDriver->setOutputLowPassFilteredVelocity(odom_cfg.output_low_pass_filtered_velocity, false);
    mDriver->setOutputLowPassFilteredHeading(odom_cfg.output_low_pass_filtered_heading, false);
    mDriver->setOdometerProfile(odom_cfg.odometer_profile, false);
    mDriver->setUpperSpeedLimitForHeadingFilter(odom_cfg.upper_speed_limit_for_heading_filter, false);
    mDriver->setMaxPositionAccuracyForLowSpeedHeadingFilter(
        odom_cfg.max_position_accuracy_for_low_speed_heading_filter, false);
    mDriver->setVelocityLowPassFilterLevel(odom_cfg.velocity_low_pass_filter_level, false);
    mDriver->setHeadingLowPassFilterLevel(odom_cfg.heading_low_pass_filter_level, false);

    // Navigation configuration
    configuration::Navigation nav_cfg = _navigation_configuration.get();
    mDriver->setPositionMeasurementPeriod(nav_cfg.position_measurement_period, false);
    mDriver->setMeasurementsPerSolutionRatio(nav_cfg.measurements_per_solution_ratio, false);
    mDriver->setTimeSystem(nav_cfg.time_system, false);
    mDriver->setDynamicModel(nav_cfg.dynamic_model, false);
    mDriver->setSpeedThreshold(nav_cfg.speed_threshold, false);
    mDriver->setStaticHoldDistanceThreshold(nav_cfg.static_hold_distance_threshold, false);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    mUTMConverter.setParameters(_utm_parameters.get());

    iodrivers_base::ConfigureGuard guard(this);
    mDriver = std::unique_ptr<gps_ublox::Driver>(new Driver());
    if (!_io_port.get().empty())
        mDriver->openURI(_io_port.get());
    setDriver(mDriver.get());

    if (! TaskBase::configureHook())
        return false;

    loadConfiguration();

    guard.commit();
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}

template<typename T>
T may_invalidate(T const& value)
{
    if (value == T::Zero())
        return T::Ones() * base::unknown<double>();
    else return value;
}
base::samples::RigidBodyState Task::convertToRBS(const gps_ublox::GPSData &data) const {
    base::samples::RigidBodyState rbs;
    gps_base::Solution geodeticPosition;

    Eigen::Vector3d body2ned_velocity = Eigen::Vector3d(
        data.vel_ned.x(), data.vel_ned.y(), data.vel_ned.z());

    rbs.time = data.time;
    rbs.velocity = Eigen::AngleAxisd(
        M_PI, Eigen::Vector3d::UnitX()) * may_invalidate(body2ned_velocity);

    geodeticPosition.latitude = data.latitude.getDeg();
    geodeticPosition.longitude = data.longitude.getDeg();
    geodeticPosition.altitude = data.height;

    rbs.position = mUTMConverter.convertToNWU(geodeticPosition).position;
    return rbs;
}
void Task::processIO()
{
    gps_ublox::UBX::Frame frame = mDriver->readFrame();
    if (frame.msg_class == UBX::MSG_CLASS_NAV && frame.msg_id == UBX::MSG_ID_PVT) {
        gps_ublox::GPSData data = UBX::parsePVT(frame.payload);

        gps_ublox::SatelliteInfo dev_sat_info = mDriver->readSatelliteInfo();
        gps_base::SatelliteInfo rock_sat_info;
        for (const SatelliteInfo::Data &data : dev_sat_info.signals) {
            gps_base::Satellite sat;
            sat.azimuth = (int)data.azimuth.getDeg();
            sat.elevation = (int)data.elevation.getDeg();
            sat.PRN = data.satellite_id;
            sat.SNR = (double)data.signal_strength;

            rock_sat_info.knownSatellites.push_back(sat);
        }

        _pose_samples.write(convertToRBS(data));
        _satellite_info.write(rock_sat_info);
        _signal_info.write(mDriver->readSignalInfo());
        _rf_info.write(mDriver->readRFInfo());
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
