package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class AutoIntakePiece extends CommandBase {
    private final Intake mSubsystem;
    private final Telemetry mTelemetry;
    private final AllianceColor allianceColor;  // Alliance color selected via opmode

    // A counter used in the EJECT state to run outtake for a set duration.
    private int countAfterPiece;

    // Define a state machine with three states.
    private enum State {
        INTAKE,    // Actively intake a game piece.
        ACCEPT,    // Correct (acceptable) piece detected: stop intake.
        EJECT,      // Wrong piece detected: eject for a set time.
        STOP        // Stop the intake.
    }

    private State currentState;

    /**
     * Enum representing the two possible alliance colors.
     */
    public enum AllianceColor {
        RED,
        BLUE
    }

    /**
     * Constructor now accepts an AllianceColor parameter.
     *
     * @param subsystem      The intake subsystem.
     * @param telemetry      Telemetry for debug output.
     * @param allianceColor  The alliance color for the current opmode.
     */
    public AutoIntakePiece(Intake subsystem, Telemetry telemetry, AllianceColor allianceColor) {
        this.mSubsystem = subsystem;
        this.mTelemetry = telemetry;
        this.allianceColor = allianceColor;
        this.countAfterPiece = 0;
        this.currentState = State.INTAKE;
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
            case INTAKE:
                // Actively run the intake.
                mSubsystem.intakePiece();
                // When a piece is detected, check its color.
                if (mSubsystem.seeingPiece()) {
                    boolean pieceAcceptable = false;
                    if (mSubsystem.isYellowPiece()) {
                        pieceAcceptable = true;
                    } else if (allianceColor == AllianceColor.RED && mSubsystem.isRedPiece()) {
                        pieceAcceptable = true;
                    } else if (allianceColor == AllianceColor.BLUE && mSubsystem.isBluePiece()) {
                        pieceAcceptable = true;
                    }
                    // Transition to the appropriate state.
                    if (pieceAcceptable) {
                        currentState = State.ACCEPT;
                    } else {
                        currentState = State.EJECT;
                        countAfterPiece = 0; // Reset the eject timer.
                    }
                }
                break;

            case ACCEPT:
                // Correct piece acquired: stop the intake.
                mSubsystem.stopIntake();
                // Remain in this state until the piece is removed.
                if (!mSubsystem.seeingPiece()) {
                    // Once the piece is no longer detected, resume intaking.
                    currentState = State.INTAKE;
                }
                break;

            case EJECT:
                // Wrong piece: outtake (eject) the piece.
                mSubsystem.outtakePiece();
                countAfterPiece++;
                // After a set number of cycles, assume the piece has been ejected.
                if (countAfterPiece >= ConfigConstants.IntakeTiming.cyclesEject) {
                    // Resume intaking regardless of sensor reading.
                    currentState = State.INTAKE;
                }
                break;
            case STOP:
                // Stop the intake.
                mSubsystem.stopIntake();
                countAfterPiece = 0;
        }

        // Telemetry for debugging.
        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            mTelemetry.addData("AutoIntake State - INTAKE", currentState == State.INTAKE);
            mTelemetry.addData("AutoIntake State - ACCEPT", currentState == State.ACCEPT);
            mTelemetry.addData("AutoIntake State - EJECT", currentState == State.EJECT);
            mTelemetry.addData("Cycle Count", countAfterPiece);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mSubsystem.stopIntake();
        mTelemetry.addData("AutoIntakePiece: done?", true);
    }

    @Override
    public boolean isFinished() {
        // This command is designed to run continuously until canceled.
        return false;
    }
}
