package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;

public class TestAutoAction implements Action {
    private boolean initialized = false;
    private Telemetry mTelemetry;

    private int cycleCounter;

    public TestAutoAction(Telemetry robotTelemetry) {
        super();
        mTelemetry = robotTelemetry;
    }

    public boolean run(@NonNull TelemetryPacket packet) {
        if (!this.initialized) {
            this.initializeAction();
        }

        packet.put("TestAuto: cyclesLeft", this.cycleCounter);
        mTelemetry.addData("TestAuto: cyclesLeft", this.cycleCounter);

        this.cycleCounter--;

        // Return of Action#run should say whether to run another cycle or not
        return !isFinished();
    }

    private void initializeAction() {
        this.cycleCounter = ConfigConstants.AutoTesting.cyclesToCount;
        this.initialized = true;
    }

    public boolean isFinished() {
        return this.cycleCounter < 0;
    }
}
