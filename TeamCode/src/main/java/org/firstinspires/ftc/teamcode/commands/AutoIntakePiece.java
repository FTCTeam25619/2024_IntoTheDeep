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

    private int countAfterPiece;

    // Define states.
    public enum State {
        INTAKE,    // Actively intake a game piece.
        ACCEPT,    // Correct piece detected: stop intake.
        EJECT,      // Wrong piece detected: eject for a set time.
        STOP       // Stop intake.
    }

    private State currentState;

    /**
     * Enum representing the two possible alliance colors.
     */
    public enum AllianceColor {
        RED,
        BLUE
    }

    // Callback to be run when a correct piece is acquired.
    private Runnable onPieceAccepted;
    // To ensure we only call the callback once per accepted piece.
    private boolean callbackCalled = false;

    /**
     * Constructor.
     */
    public AutoIntakePiece(Intake subsystem, Telemetry telemetry, AllianceColor allianceColor) {
        this.mSubsystem = subsystem;
        this.mTelemetry = telemetry;
        this.allianceColor = allianceColor;
        this.countAfterPiece = 0;
        this.currentState = State.STOP;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        currentState = State.STOP;
        countAfterPiece = 0;
        callbackCalled = false;
    }

    @Override
    public void execute() {
        switch (currentState) {
            case INTAKE:
                mSubsystem.intakePiece();
                if (mSubsystem.seeingPiece()) {
                    boolean pieceAcceptable = false;
                    if (mSubsystem.isYellowPiece()) {
                        pieceAcceptable = true;
                    } else if (allianceColor == AllianceColor.RED && mSubsystem.isRedPiece()) {
                        pieceAcceptable = true;
                    } else if (allianceColor == AllianceColor.BLUE && mSubsystem.isBluePiece()) {
                        pieceAcceptable = true;
                    }
                    // Transition based on the piece color.
                    if (pieceAcceptable) {
                        currentState = State.ACCEPT;
                    } else {
                        currentState = State.EJECT;
                        countAfterPiece = 0;
                    }
                }
                break;

            case ACCEPT:
                if (onPieceAccepted != null && !callbackCalled) {
                    callbackCalled = true;
                    onPieceAccepted.run(); // This schedules handoffPiece.
                }
                // Stay in ACCEPT until the piece is removed.
                if (!mSubsystem.seeingPiece()) {
                    // Reset for the next piece.
                    mSubsystem.stopIntake();
                    currentState = State.STOP;
                    callbackCalled = false;
                }
                break;


            case EJECT:
                // Wrong piece: outtake (eject) the piece.
                mSubsystem.outtakePiece();
                countAfterPiece++;
                if (countAfterPiece >= ConfigConstants.IntakeTiming.cyclesEject) {
                    currentState = State.INTAKE;
                }
                break;
            case STOP:
                mSubsystem.stopIntake();
                break;
            default:
                break;
        }

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
        // Run continuously until canceled.
        return false;
    }

    // Public setter for the callback.
    public void setOnPieceAccepted(Runnable callback) {
        this.onPieceAccepted = callback;
    }

    // Optionally, a public getter for the current state if needed.
    public State getCurrentState() {
        return currentState;
    }
    public void setState(State newState) {
        this.currentState = newState;
        this.countAfterPiece = 0; // Optionally reset your timer
    }

}


