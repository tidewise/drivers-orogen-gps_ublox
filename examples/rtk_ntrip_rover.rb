using_task_library "gps_ntrip"
using_task_library "gps_ublox"

module RTK
    class BaseRover < Syskit::Composition
        add(OroGen.gps_ntrip.Task, as: "reference")
            .deployed_as("ntrip_client")
        add(OroGen.gps_ublox.Task, as: "rover")
            .deployed_as("ublox_rover")
            .with_conf("default", "rover")

        reference_child.rtcm_out_port.connect_to \
            rover_child.rtcm_in_port, type: :buffer, size: 100
        rover_child.gps_solution_port.connect_to \
            reference_child.gps_solution_port
    end
end

Robot.controller do
    Roby.plan.add_mission_task RTK::BaseRover
end
