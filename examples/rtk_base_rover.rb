using_task_library "gps_ublox"

Syskit.extend_model OroGen.gps_ublox.Task do
    argument :io_port

    def update_properties
        super

        properties.io_port = io_port
        properties.rtcm_output_messages = [1074, 1084, 1094, 1124, 1230, 4072]
        properties.msg_rates do |r|
            r.rtk_info = 1
            r
        end
    end
end

module RTK
    class BaseRover < Syskit::Composition
        add(OroGen.gps_ublox.Task, as: "reference")
            .with_arguments(io_port: ENV["REFERENCE_IO"])
            .prefer_deployed_tasks("reference")
        add(OroGen.gps_ublox.Task, as: "rover")
            .with_arguments(io_port: ENV["ROVER_IO"])
            .prefer_deployed_tasks("rover")

        reference_child.rtcm_out_port.connect_to \
            rover_child.rtcm_in_port, type: :buffer, size: 100
    end
end

Syskit.conf.use_deployment OroGen.gps_ublox.Task => "reference"
Syskit.conf.use_deployment OroGen.gps_ublox.Task => "rover"

Robot.controller do
    Roby.plan.add_mission_task RTK::BaseRover
end