package org.firstinspires.ftc.teamcode.utils;


import androidx.annotation.Nullable;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SimpleTelemetry {
    private static final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private @Nullable Telemetry telemetry;

    public SimpleTelemetry() {
    }

    public void registerFtcTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void update() {
        panelsTelemetry.update();
        if (telemetry != null) telemetry.update();

    }

    public void addData(String key, double value) {
        panelsTelemetry.addData(key, value);
        if (telemetry != null) telemetry.addData(key, value);
    }

    public void addData(String key, Object value) {
        panelsTelemetry.addData(key, value);
        if (telemetry != null) telemetry.addData(key, value);
    }
}
