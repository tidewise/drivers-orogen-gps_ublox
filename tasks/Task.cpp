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
    DevicePort port = _device_port.get();
    mDriver->setPortProtocol(port, DIRECTION_OUTPUT, PROTOCOL_UBX, true, false);

    // Message rates
    configuration::MessageRates rates = _msg_rates.get();
    mDriver->setOutputRate(port, MSGOUT_MON_RF, rates.mon_rf, false);
    mDriver->setOutputRate(port, MSGOUT_NAV_PVT, rates.nav_pvt, false);
    mDriver->setOutputRate(port, MSGOUT_NAV_SIG, rates.nav_sig, false);
    mDriver->setOutputRate(port, MSGOUT_NAV_SAT, rates.nav_sat, false);

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
    mDriver->setPositionMeasurementPeriod(
        nav_cfg.position_measurement_period.toMilliseconds(), false);
    mDriver->setMeasurementsPerSolutionRatio(
        nav_cfg.measurements_per_solution_ratio, false);
    mDriver->setMeasurementRefTime(nav_cfg.measurement_ref_time, false);
    mDriver->setDynamicModel(nav_cfg.dynamic_model, false);
    mDriver->setSpeedThreshold(
        std::round(nav_cfg.speed_threshold * 100), false);
    mDriver->setStaticHoldDistanceThreshold(
        nav_cfg.static_hold_distance_threshold, false);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    mUTMConverter.setParameters(_utm_parameters.get());

    iodrivers_base::ConfigureGuard guard(this);
    mDriver = std::unique_ptr<gps_ublox::Driver>(new Driver());
    if (!_io_port.get().empty()) {
        mDriver->openURI(_io_port.get());
    }
    setDriver(mDriver.get());

    if (! TaskBase::configureHook()) {
        return false;
    }

    loadConfiguration();

    guard.commit();
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook()) {
        return false;
    }
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}

template<typename T>
T may_invalidate(T const& value)
{
    if (value == T::Zero()) {
        return T::Ones() * base::unknown<double>();
    }
    else {
        return value;
    }
}

static gps_base::Solution convertToBaseSolution(const PVT &data);
static gps_base::SatelliteInfo convertToBaseSatelliteInfo(const SatelliteInfo &sat_info);

static base::samples::RigidBodyState convertToRBS(const gps_ublox::PVT &data, gps_base::UTMConverter& utmConverter) {
    base::samples::RigidBodyState rbs;
    gps_base::Solution geodeticPosition;

    Eigen::Vector3d body2ned_velocity = Eigen::Vector3d(
        data.vel_ned.x(), data.vel_ned.y(), data.vel_ned.z());

    rbs.time = data.time;
    rbs.velocity = Eigen::AngleAxisd(
        M_PI, Eigen::Vector3d::UnitX()) * may_invalidate(body2ned_velocity);

    auto geodetic = convertToBaseSolution(data);
    base::samples::RigidBodyState nwu = utmConverter.convertToNWU(geodetic);
    rbs.position = nwu.position;
    rbs.cov_position = nwu.cov_position;
    return rbs;
}
static gps_base::SatelliteInfo convertToBaseSatelliteInfo(const SatelliteInfo &sat_info)
{
    gps_base::SatelliteInfo rock_sat_info;

    rock_sat_info.time = base::Time::now();
    for (const SatelliteInfo::Data &data : sat_info.signals) {
        gps_base::Satellite sat;
        sat.azimuth = (int)data.azimuth.getDeg();
        sat.elevation = (int)data.elevation.getDeg();
        sat.PRN = data.satellite_id;
        sat.SNR = (double)data.signal_strength;

        rock_sat_info.knownSatellites.push_back(sat);
    }
    return rock_sat_info;
}
static gps_base::Solution convertToBaseSolution(const PVT &data)
{
    gps_base::Solution solution;

    solution.ageOfDifferentialCorrections = base::unknown<double>();
    solution.geoidalSeparation = base::unknown<double>();
    solution.latitude = data.latitude.getDeg();
    solution.longitude = data.longitude.getDeg();
    solution.altitude = data.height_above_mean_sea_level;
    solution.deviationAltitude = data.vertical_accuracy;
    solution.deviationLatitude = data.horizontal_accuracy;
    solution.deviationLongitude = data.horizontal_accuracy;
    solution.noOfSatellites = data.num_sats;
    switch (data.fix_type) {
        case PVT::NO_FIX:
        case PVT::TIME_ONLY:
        case PVT::DEAD_RECKONING:
            solution.positionType = gps_base::NO_SOLUTION;
            break;
        case PVT::FIX_2D:
            solution.positionType = gps_base::AUTONOMOUS_2D;
            break;
        case PVT::FIX_3D:
        case PVT::GNSS_PLUS_DEAD_RECKONING:
            solution.positionType = gps_base::AUTONOMOUS;
            break;
        default:
            solution.positionType = gps_base::INVALID;
            break;
    }
    solution.time = data.time;
    return solution;
}

struct gps_ublox::PollCallbacks : gps_ublox::Driver::PollCallbacks {
    Task& mTask;

    PollCallbacks(Task& task, bool rtk)
        : mTask(task), mOutputRTK(rtk) {}

    void pvt(PVT const& pvt) override {
        mTask._pose_samples.write(convertToRBS(pvt, mTask.mUTMConverter));
        mTask._gps_solution.write(convertToBaseSolution(pvt));
    }

    void satelliteInfo(SatelliteInfo const& info) override {
        mTask._satellite_info.write(convertToBaseSatelliteInfo(info));
    }

    void signalInfo(SignalInfo const& info) override {
        mTask._signal_info.write(info);
    }

    void rfInfo(RFInfo const& info) override {
        mTask._rf_info.write(info);
    }
};
void Task::processIO()
{
    PollCallbacks callbacks(*this);
    mDriver->poll(callbacks);
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
