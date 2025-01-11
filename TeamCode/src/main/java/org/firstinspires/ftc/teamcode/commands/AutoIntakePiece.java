package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class AutoIntakePiece  extends CommandBase {
    private final Intake mSubsystem;
    private final Telemetry mTelemetry;

    private int countAfterPiece;

    private enum State {
        STOPPED,
        INTAKE,
        PIECE_FOUND,
        OUTTAKE;
    }

    private State currentState;

    public AutoIntakePiece(Intake subsystem, Telemetry telemetry) {
        this.mSubsystem = subsystem;
        this.mTelemetry = telemetry;

        countAfterPiece = 0;
        currentState = State.INTAKE;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        currentState = State.INTAKE;
        countAfterPiece = 0;
    }

    @Override
    public void execute() {
        switch (currentState) {
            case STOPPED:
                countAfterPiece = 0;
                mSubsystem.stopIntake();
                break;
            case INTAKE:
                mSubsystem.intakePiece();
                countAfterPiece = 0;
                if (mSubsystem.seeingPiece()) { currentState = State.PIECE_FOUND; }
                break;
            case PIECE_FOUND:
                countAfterPiece++;
                if (countAfterPiece <= ConfigConstants.IntakeTiming.cyclesReverse) {
                    mSubsystem.outtakePiece();
                } else {
                    currentState = State.STOPPED;
                }
                break;
            case OUTTAKE:
                mSubsystem.outtakePiece();
                break;
        }

        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            mTelemetry.addData("IntakePiece: State INTAKE", currentState == State.INTAKE);
            mTelemetry.addData("IntakePiece: State STOPPED", currentState == State.STOPPED);
            mTelemetry.addData("IntakePiece: State OUTTAKE", currentState == State.OUTTAKE);
            mTelemetry.addData("IntakePiece: State PIECE_FOUND", currentState == State.PIECE_FOUND);
            mTelemetry.addData("IntakePiece: count", countAfterPiece);
            mTelemetry.addData("IntakePiece: done?", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mSubsystem.stopIntake();
        mTelemetry.addData("IntakePiece: done?", true);
    }

    @Override
    public boolean isFinished() {
        return currentState == State.STOPPED;
    }
}
