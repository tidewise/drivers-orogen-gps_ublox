using_task_library "gps_ublox"

module RTK
    class BaseRover < Syskit::Composition
        add(OroGen.gps_ublox.Task, as: "reference")
            .deployed_as("ublox_reference_station")
            .with_conf("default", "reference")
        add(OroGen.gps_ublox.Task, as: "rover")
            .deployed_as("ublox_rover")
            .with_conf("default", "rover")

        reference_child.rtcm_out_port.connect_to \
            rover_child.rtcm_in_port, type: :buffer, size: 100
    end
end

Robot.controller do
    Roby.plan.add_mission_task RTK::BaseRover
end
