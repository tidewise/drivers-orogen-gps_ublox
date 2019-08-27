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
    bool persist = _persist_configuration.get();

    // Enabled ports
    configuration::Ports enabled_ports = _enabled_ports.get();
    mDriver->setPortEnabled(Driver::PORT_I2C, enabled_ports.i2c, persist);
    mDriver->setPortEnabled(Driver::PORT_UART1, enabled_ports.uart1, persist);
    mDriver->setPortEnabled(Driver::PORT_UART2, enabled_ports.uart2, persist);
    mDriver->setPortEnabled(Driver::PORT_USB, enabled_ports.usb, persist);
    mDriver->setPortEnabled(Driver::PORT_SPI, enabled_ports.spi, persist);

    // Input protocols
    configuration::PortProtocols input_protocols = _input_protocols.get();
    mDriver->setPortProtocol(Driver::PORT_I2C, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_NMEA, input_protocols.i2c.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_I2C, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_UBX, input_protocols.i2c.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_I2C, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_RTCM3X, input_protocols.i2c.rtcm3x, persist);
    mDriver->setPortProtocol(Driver::PORT_UART1, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_NMEA, input_protocols.uart1.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_UART1, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_UBX, input_protocols.uart1.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_UART1, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_RTCM3X, input_protocols.uart1.rtcm3x, persist);
    mDriver->setPortProtocol(Driver::PORT_UART2, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_NMEA, input_protocols.uart2.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_UART2, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_UBX, input_protocols.uart2.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_UART2, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_RTCM3X, input_protocols.uart2.rtcm3x, persist);
    mDriver->setPortProtocol(Driver::PORT_USB, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_NMEA, input_protocols.usb.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_USB, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_UBX, input_protocols.usb.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_USB, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_RTCM3X, input_protocols.usb.rtcm3x, persist);
    mDriver->setPortProtocol(Driver::PORT_SPI, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_NMEA, input_protocols.spi.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_SPI, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_UBX, input_protocols.spi.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_SPI, Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_RTCM3X, input_protocols.spi.rtcm3x, persist);

    // Output protocols
    configuration::PortProtocols output_protocols = _output_protocols.get();
    mDriver->setPortProtocol(Driver::PORT_I2C, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_NMEA, output_protocols.i2c.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_I2C, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_UBX, output_protocols.i2c.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_I2C, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_RTCM3X, output_protocols.i2c.rtcm3x, persist);
    mDriver->setPortProtocol(Driver::PORT_UART1, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_NMEA, output_protocols.uart1.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_UART1, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_UBX, output_protocols.uart1.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_UART1, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_RTCM3X, output_protocols.uart1.rtcm3x, persist);
    mDriver->setPortProtocol(Driver::PORT_UART2, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_NMEA, output_protocols.uart2.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_UART2, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_UBX, output_protocols.uart2.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_UART2, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_RTCM3X, output_protocols.uart2.rtcm3x, persist);
    mDriver->setPortProtocol(Driver::PORT_USB, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_NMEA, output_protocols.usb.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_USB, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_UBX, output_protocols.usb.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_USB, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_RTCM3X, output_protocols.usb.rtcm3x, persist);
    mDriver->setPortProtocol(Driver::PORT_SPI, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_NMEA, output_protocols.spi.nmea, persist);
    mDriver->setPortProtocol(Driver::PORT_SPI, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_UBX, output_protocols.spi.ubx, persist);
    mDriver->setPortProtocol(Driver::PORT_SPI, Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_RTCM3X, output_protocols.spi.rtcm3x, persist);

    // Message rates
    configuration::MessageRates rates = _msg_rates.get();
    mDriver->setOutputRate(Driver::PORT_I2C, Driver::MSGOUT_MON_RF, rates.i2c.mon_rf, persist);
    mDriver->setOutputRate(Driver::PORT_I2C, Driver::MSGOUT_NAV_PVT, rates.i2c.nav_pvt, persist);
    mDriver->setOutputRate(Driver::PORT_I2C, Driver::MSGOUT_NAV_SIG, rates.i2c.nav_sig, persist);
    mDriver->setOutputRate(Driver::PORT_UART1, Driver::MSGOUT_MON_RF, rates.uart1.mon_rf, persist);
    mDriver->setOutputRate(Driver::PORT_UART1, Driver::MSGOUT_NAV_PVT, rates.uart1.nav_pvt, persist);
    mDriver->setOutputRate(Driver::PORT_UART1, Driver::MSGOUT_NAV_SIG, rates.uart1.nav_sig, persist);
    mDriver->setOutputRate(Driver::PORT_UART2, Driver::MSGOUT_MON_RF, rates.uart2.mon_rf, persist);
    mDriver->setOutputRate(Driver::PORT_UART2, Driver::MSGOUT_NAV_PVT, rates.uart2.nav_pvt, persist);
    mDriver->setOutputRate(Driver::PORT_UART2, Driver::MSGOUT_NAV_SIG, rates.uart2.nav_sig, persist);
    mDriver->setOutputRate(Driver::PORT_USB, Driver::MSGOUT_MON_RF, rates.usb.mon_rf, persist);
    mDriver->setOutputRate(Driver::PORT_USB, Driver::MSGOUT_NAV_PVT, rates.usb.nav_pvt, persist);
    mDriver->setOutputRate(Driver::PORT_USB, Driver::MSGOUT_NAV_SIG, rates.usb.nav_sig, persist);
    mDriver->setOutputRate(Driver::PORT_SPI, Driver::MSGOUT_MON_RF, rates.spi.mon_rf, persist);
    mDriver->setOutputRate(Driver::PORT_SPI, Driver::MSGOUT_NAV_PVT, rates.spi.nav_pvt, persist);
    mDriver->setOutputRate(Driver::PORT_SPI, Driver::MSGOUT_NAV_SIG, rates.spi.nav_sig, persist);

    // Odometer configuration
    configuration::Odometer odom_cfg = _odometer_configuration.get();
    mDriver->setOdometer(odom_cfg.enabled, persist);
    mDriver->setLowSpeedCourseOverGroundFilter(odom_cfg.low_speed_course_over_ground_filter, persist);
    mDriver->setOutputLowPassFilteredVelocity(odom_cfg.output_low_pass_filtered_velocity, persist);
    mDriver->setOutputLowPassFilteredHeading(odom_cfg.output_low_pass_filtered_heading, persist);
    mDriver->setOdometerProfile(odom_cfg.odometer_profile, persist);
    mDriver->setUpperSpeedLimitForHeadingFilter(odom_cfg.upper_speed_limit_for_heading_filter, persist);
    mDriver->setMaxPositionAccuracyForLowSpeedHeadingFilter(
        odom_cfg.max_position_accuracy_for_low_speed_heading_filter, persist);
    mDriver->setVelocityLowPassFilterLevel(odom_cfg.velocity_low_pass_filter_level, persist);
    mDriver->setHeadingLowPassFilterLevel(odom_cfg.heading_low_pass_filter_level, persist);

    // Navigation configuration
    configuration::Navigation nav_cfg = _navigation_configuration.get();
    mDriver->setPositionMeasurementPeriod(nav_cfg.position_measurement_period, persist);
    mDriver->setMeasurementsPerSolutionRatio(nav_cfg.measurements_per_solution_ratio, persist);
    mDriver->setTimeSystem(nav_cfg.time_system, persist);
    mDriver->setDynamicModel(nav_cfg.dynamic_model, persist);
    mDriver->setSpeedThreshold(nav_cfg.speed_threshold, persist);
    mDriver->setStaticHoldDistanceThreshold(nav_cfg.static_hold_distance_threshold, persist);
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
void Task::processIO()
{
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
