/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <iodrivers_base/ConfigureGuard.hpp>
#include <gps_ublox/Driver.hpp>

using namespace gps_ublox;
using base::samples::RigidBodyState;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

void Task::loadConfiguration(Driver& driver) {
    auto const& rtcmOutput = _rtcm_output_messages.get();
    mOutputRTK = !rtcmOutput.empty();

    DevicePort port = _device_port.get();
    driver.setPortProtocol(port, DIRECTION_INPUT, PROTOCOL_UBX, true, false);
    driver.setPortProtocol(port, DIRECTION_INPUT, PROTOCOL_RTCM3X, true, false);
    driver.setPortProtocol(port, DIRECTION_OUTPUT, PROTOCOL_UBX, true, false);
    driver.setPortProtocol(port, DIRECTION_OUTPUT, PROTOCOL_RTCM3X, mOutputRTK, false);
    driver.setPortProtocol(port, DIRECTION_OUTPUT, PROTOCOL_NMEA, false, false);

    // Message rates
    configuration::MessageRates rates = _msg_rates.get();
    driver.setOutputRate(port, MSGOUT_MON_RF, rates.mon_rf, false);
    driver.setOutputRate(port, MSGOUT_NAV_PVT, rates.nav_pvt, false);
    driver.setOutputRate(port, MSGOUT_NAV_SIG, rates.nav_sig, false);
    driver.setOutputRate(port, MSGOUT_NAV_SAT, rates.nav_sat, false);
    driver.setOutputRate(port, MSGOUT_NAV_RELPOSNED, rates.nav_relposned, false);
    driver.setOutputRate(port, MSGOUT_RXM_RTCM, rates.rtk_info, false);
    driver.setOutputRate(port, MSGOUT_MON_COMMS, rates.mon_comms, false);

    for (auto rtcm_msg: _rtcm_output_messages.get()) {
        driver.setRTCMOutputRate(port, rtcm_msg);
    }

    // Odometer configuration
    configuration::Odometer odom_cfg = _odometer_configuration.get();
    driver.setOdometer(odom_cfg.enabled, false);
    driver.setLowSpeedCourseOverGroundFilter(odom_cfg.low_speed_course_over_ground_filter, false);
    driver.setOutputLowPassFilteredVelocity(odom_cfg.output_low_pass_filtered_velocity, false);
    driver.setOutputLowPassFilteredHeading(odom_cfg.output_low_pass_filtered_heading, false);
    driver.setOdometerProfile(odom_cfg.odometer_profile, false);
    driver.setUpperSpeedLimitForHeadingFilter(odom_cfg.upper_speed_limit_for_heading_filter, false);
    driver.setMaxPositionAccuracyForLowSpeedHeadingFilter(
        odom_cfg.max_position_accuracy_for_low_speed_heading_filter, false);
    driver.setVelocityLowPassFilterLevel(odom_cfg.velocity_low_pass_filter_level, false);
    driver.setHeadingLowPassFilterLevel(odom_cfg.heading_low_pass_filter_level, false);

    // Navigation configuration
    configuration::Navigation nav_cfg = _navigation_configuration.get();
    driver.setPositionMeasurementPeriod(
        nav_cfg.position_measurement_period.toMilliseconds(), false);
    driver.setMeasurementsPerSolutionRatio(
        nav_cfg.measurements_per_solution_ratio, false);
    driver.setMeasurementRefTime(nav_cfg.measurement_ref_time, false);
    driver.setDynamicModel(nav_cfg.dynamic_model, false);
    driver.setSpeedThreshold(
        std::round(nav_cfg.speed_threshold * 100), false);
    driver.setStaticHoldDistanceThreshold(
        nav_cfg.static_hold_distance_threshold, false);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    mUTMConverter.setParameters(_utm_parameters.get());

    std::unique_ptr<gps_ublox::Driver> driver(new Driver());
    iodrivers_base::ConfigureGuard guard(this);
    if (!_io_port.get().empty()) {
        driver->openURI(_io_port.get());
    }
    setDriver(driver.get());

    if (! TaskBase::configureHook()) {
        return false;
    }

    loadConfiguration(*driver);

    mDriver = move(driver);
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
    iodrivers_base::RawPacket rtcm;
    while (_rtcm_in.read(rtcm, false) == RTT::NewData) {
        mRTKInfo.addRX(rtcm.data.size());
        mDriver->writeRTCM(rtcm.data);
    }

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
static RigidBodyState convertToRBS(PVT const& data, gps_base::UTMConverter& utmConverter);
static RigidBodyState convertToRBS(RelPosNED const& data);

struct gps_ublox::PollCallbacks : gps_ublox::Driver::PollCallbacks {
    Task& mTask;
    bool mOutputRTK;

    PollCallbacks(Task& task, bool rtk)
        : mTask(task), mOutputRTK(rtk) {}

    void pvt(PVT const& pvt) override {
        auto rbs = convertToRBS(pvt, mTask.mUTMConverter);
        mTask._pose_samples.write(rbs);
        mTask._gps_solution.write(convertToBaseSolution(pvt));

        mTask.mTOW = pvt.time_of_week;
        mTask.mUTCAtTOW = pvt.time;
        mTask.mVelocityAtTOW = rbs.velocity;

        mTask.mRTKInfo.update(pvt);
        if (mOutputRTK) {
            mTask._rtk_info.write(mTask.mRTKInfo);
        }
    }

    void relposned(RelPosNED const& relposned) override {
        if (relposned.time_of_week != mTask.mTOW) {
            // TODO: it happens that in my tests the PVT message always arrived
            // before the RELPOSNED, which actually makes sense
            //
            // BUT I could not find anything guaranteeing this in the documentation.
            // Try to handle that case here.
            return;
        }

        auto rbs = convertToRBS(relposned);
        rbs.time = mTask.mUTCAtTOW;
        rbs.velocity = mTask.mVelocityAtTOW;

        mTask._rtk_relative_pose_samples.write(rbs);
    }

    void rtcm(uint8_t const* buffer, size_t size) override {
        iodrivers_base::RawPacket packet;
        packet.time = base::Time::now();
        packet.data.insert(packet.data.end(), buffer, buffer + size);
        mTask._rtcm_out.write(packet);

        mTask.mRTKInfo.addTX(size);
    }

    void rtcmReceivedMessage(RTCMReceivedMessage const& msg) override {
        mTask.mRTKInfo.update(msg);
    }

    void satelliteInfo(SatelliteInfo const& info) override {
        mTask._satellite_info.write(convertToBaseSatelliteInfo(info));
        mTask.mRTKInfo.update(info);
    }

    void signalInfo(SignalInfo const& info) override {
        mTask._signal_info.write(info);
    }

    void rfInfo(RFInfo const& info) override {
        mTask._rf_info.write(info);
    }

    void commsInfo(CommsInfo const& info) override {
        mTask._comms_info.write(info);
    }
};
void Task::processIO()
{
    PollCallbacks callbacks(*this, mOutputRTK);
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
    DevicePort port = _device_port.get();
    mDriver->setOutputRate(port, MSGOUT_MON_RF, 0, false);
    mDriver->setOutputRate(port, MSGOUT_NAV_PVT, 0, false);
    mDriver->setOutputRate(port, MSGOUT_NAV_SIG, 0, false);
    mDriver->setOutputRate(port, MSGOUT_NAV_SAT, 0, false);
    mDriver->setOutputRate(port, MSGOUT_NAV_RELPOSNED, 0, false);
    mDriver->setOutputRate(port, MSGOUT_RXM_RTCM, 0, false);
    mDriver->setOutputRate(port, MSGOUT_MON_COMMS, 0, false);

    TaskBase::cleanupHook();
}

static RigidBodyState convertToRBS(PVT const& data, gps_base::UTMConverter& utmConverter) {
    RigidBodyState rbs;
    rbs.time = data.time;
    rbs.velocity =
        Eigen::Vector3d(data.vel_ned.x(), -data.vel_ned.y(), -data.vel_ned.z());

    auto geodetic = convertToBaseSolution(data);
    RigidBodyState nwu = utmConverter.convertToNWU(geodetic);
    rbs.position = nwu.position;
    rbs.cov_position = nwu.cov_position;
    return rbs;
}

static RigidBodyState convertToRBS(RelPosNED const& data) {
    RigidBodyState rbs;
    if (!(data.flags & RelPosNED::FLAGS_RELATIVE_POSITION_VALID)) {
        return rbs;
    }

    rbs.position = Eigen::Vector3d(
        data.relative_position_NED.x(),
        -data.relative_position_NED.y(),
        -data.relative_position_NED.z()
    );
    rbs.cov_position(0, 0) = data.accuracy_NED.x() * data.accuracy_NED.x();
    rbs.cov_position(1, 1) = data.accuracy_NED.y() * data.accuracy_NED.y();
    rbs.cov_position(2, 2) = data.accuracy_NED.z() * data.accuracy_NED.z();
    rbs.velocity = rbs.velocity;
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
        sat.PRN = (static_cast<uint32_t>(data.gnss_id) << 8) + data.satellite_id;
        sat.SNR = (double)data.signal_strength;

        rock_sat_info.knownSatellites.push_back(sat);
    }
    return rock_sat_info;
}
static gps_base::Solution convertToBaseSolution(const PVT &data)
{
    gps_base::Solution solution;

    double corrAge = data.age_of_differential_corrections.toSeconds();
    if (corrAge < 120) {
        solution.ageOfDifferentialCorrections = corrAge;
    }
    else {
        solution.ageOfDifferentialCorrections = base::unknown<double>();
    }

    solution.latitude = data.latitude.getDeg();
    solution.longitude = data.longitude.getDeg();

    float gps2ellipsoid = data.height;
    float gps2msl = data.height_above_mean_sea_level;
    float msl2ellipsoid = gps2ellipsoid - gps2msl;
    solution.geoidalSeparation = msl2ellipsoid;
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
        case PVT::GNSS_PLUS_DEAD_RECKONING: {
            if (data.fix_flags & PVT::FIX_RTK_FIXED) {
                solution.positionType = gps_base::RTK_FIXED;
            }
            else if (data.fix_flags & PVT::FIX_RTK_FLOAT) {
                solution.positionType = gps_base::RTK_FLOAT;
            }
            else {
                solution.positionType = gps_base::AUTONOMOUS;
            }
            break;
        }
        default:
            solution.positionType = gps_base::INVALID;
            break;
    }
    solution.time = data.time;
    return solution;
}
