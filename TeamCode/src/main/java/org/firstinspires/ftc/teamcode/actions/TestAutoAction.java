package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class TestAutoAction implements Action {
    private boolean initialized = false;

    private int cycleCounter;

    public boolean run(@NonNull TelemetryPacket packet) {
        if (!this.initialized) {
            this.initializeAction();
        }

        packet.put("TestAuto: cyclesLeft", this.cycleCounter);

        this.cycleCounter--;

        return this.cycleCounter <= 0;
    }

    private void initializeAction() {
        this.cycleCounter = 5;
        this.initialized = true;
    }
}
