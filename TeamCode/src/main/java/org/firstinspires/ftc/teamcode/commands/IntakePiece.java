package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakePiece  extends CommandBase {
    private final GamepadEx mController2;
    private final Intake mSubsystem;
    private final Telemetry mTelemetry;

    private boolean pieceSeen;
    private int countAfterPiece;

    public IntakePiece(Intake subsystem, GamepadEx controller2, Telemetry telemetry) {
        this.mSubsystem = subsystem;
        this.mController2 = controller2;
        this.mTelemetry = telemetry;

        pieceSeen = false;
        countAfterPiece = 0;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double leftTrigger = this.mController2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = this.mController2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        boolean cmdIntake = rightTrigger > 0.0;
        boolean cmdOuttake = leftTrigger > 0.0;
        pieceSeen = pieceSeen || mSubsystem.seeingPiece();
        if (!cmdIntake) {
            pieceSeen = false;
            countAfterPiece = 0;
        }
        boolean stopping = cmdIntake && pieceSeen;
        boolean intaking = cmdIntake && !stopping;
        boolean outtaking = cmdOuttake && !cmdIntake;
        if (intaking) {
            mSubsystem.intakePiece();
            countAfterPiece = 0;
        } else if (stopping) {
            mSubsystem.stopIntake();
            if (countAfterPiece >= ConfigConstants.IntakeTiming.cycleWaitAfterPiece) {
                pieceSeen = false;
                countAfterPiece = 0;
            } else {
                countAfterPiece++;
                if (countAfterPiece <= ConfigConstants.IntakeTiming.cyclesReverse) {
                    mSubsystem.outtakePiece();
                }
            }
        } else if (outtaking) {
            mSubsystem.outtakePiece();
        } else {
            mSubsystem.stopIntake();
        }
        mTelemetry.addData("IntakePiece: intaking", intaking);
        mTelemetry.addData("IntakePiece: stopping", stopping);
        mTelemetry.addData("IntakePiece: outtaking", outtaking);
        mTelemetry.addData("IntakePiece: Seen?", pieceSeen);
        mTelemetry.addData("IntakePiece: count", countAfterPiece);
        mTelemetry.addData("IntakePiece: done?", false);
    }

    @Override
    public void end(boolean interrupted) {
        mSubsystem.stopIntake();
        mTelemetry.addData("IntakePiece: done?", true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
