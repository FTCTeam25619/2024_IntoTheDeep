package org.firstinspires.ftc.teamcode.commands;

import android.os.SystemClock;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Climb;

public class HoldClimb extends CommandBase {
    private Climb mClimb;
    private Telemetry mTelemetry;
    private int holdDurationMS;
    private long startTimeMS;
    private long endTimeMS;

    /**
     *
     * @param subsystem  Climb type subsystem
     * @param timeOutMS  Duration to hold climb hooks in milliseconds
     * @param telemetry  Telemetry type robot telemetry object
     *
     * Command holds position on Climb hooks wherever they currently are for a specific time
     */
    public HoldClimb(Climb subsystem, int timeOutMS, Telemetry telemetry) {
        mClimb = subsystem;
        mTelemetry = telemetry;
        holdDurationMS = timeOutMS;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        mClimb.enablePIDHold(true);
        startTimeMS = SystemClock.elapsedRealtime();
        endTimeMS = startTimeMS + holdDurationMS;
        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            mTelemetry.addData("Climb/HoldClimb: start (ms)", startTimeMS);
            mTelemetry.addData("Climb/HoldClimb: end (ms)", endTimeMS);
            mTelemetry.addData("Climb/HoldClimb: remaining (ms)", endTimeMS - SystemClock.elapsedRealtime());
        }
    }

    @Override
    public void execute() {
        // Nothing more needed
        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            mTelemetry.addData("Climb/HoldClimb: start (ms)", startTimeMS);
            mTelemetry.addData("Climb/HoldClimb: end (ms)", endTimeMS);
            mTelemetry.addData("Climb/HoldClimb: remaining (ms)", endTimeMS - SystemClock.elapsedRealtime());
        }
    }

    @Override
    public void end(boolean interrupted) {
        mClimb.enablePIDHold(false);
        if (!interrupted) {
            mClimb.stopMotors();
        }
    }

    @Override
    public boolean isFinished() {
        // Will hold position until interrupted or time out occurs
        return SystemClock.elapsedRealtime() >= endTimeMS;
    }
}
