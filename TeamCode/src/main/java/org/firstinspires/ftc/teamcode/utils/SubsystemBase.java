package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.configs.RobotHardwareGroup;

public interface SubsystemBase {
    // This stuff is magically available for each subsystem
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    GraphManager panelsGraph = PanelsGraph.INSTANCE.getManager();

    void init(RobotHardwareGroup robotHardware);

    void update();
}
