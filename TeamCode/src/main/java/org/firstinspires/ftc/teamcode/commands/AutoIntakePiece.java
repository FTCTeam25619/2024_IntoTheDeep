package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class AutoIntakePiece extends CommandBase {
    private final Intake mSubsystem;
    private final Telemetry mTelemetry;
    private final Constants.OpModes.AllianceColor allianceColor;

    private int countAfterPiece;
    private State currentState;
    private final TransferPiece transferPiece;

    private enum State {
        STOPPED,
        INTAKE,
        PIECE_FOUND,
        OUTTAKE,
        EJECTING
    }

    public AutoIntakePiece(Intake subsystem, Telemetry telemetry, Constants.OpModes.AllianceColor color, TransferPiece transferPiece) {
        this.mSubsystem = subsystem;
        this.mTelemetry = telemetry;
        this.allianceColor = color;
        this.transferPiece = transferPiece; // Store reference to TransferPiece
        countAfterPiece = 0;
        currentState = State.INTAKE;

        addRequirements(subsystem); // Ensure proper dependency management
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
                // Schedule TransferPiece after STOPPED state
                transferPiece.schedule();
                break;

            case INTAKE:
                mSubsystem.intakePiece();
                countAfterPiece = 0;
                if (mSubsystem.seeingPiece()) {
                    currentState = State.PIECE_FOUND;
                }
                break;

            case PIECE_FOUND:
                if (mSubsystem.seeingAllianceSpecificPiece(allianceColor) ||
                        mSubsystem.seeingAllianceSpecificPiece(Constants.OpModes.AllianceColor.YELLOW)) {
                    currentState = State.STOPPED;
                } else {
                    countAfterPiece = 0;
                    currentState = State.EJECTING;
                }
                if (!mSubsystem.seeingPiece()) {
                    currentState = State.INTAKE;
                }
                break;

            case OUTTAKE:
                mSubsystem.outtakePiece();
                break;

            case EJECTING:
                mSubsystem.outtakePiece();
                countAfterPiece++;
                if (countAfterPiece > 250) {
                    currentState = State.INTAKE;
                }
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
