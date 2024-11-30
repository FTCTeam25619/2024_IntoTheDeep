package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakePiece  extends CommandBase {
    private final GamepadEx mController2;
    private final Intake mSubsystem;
    private final Telemetry mTelemetry;

    public IntakePiece(Intake subsystem, GamepadEx controller2, Telemetry telemetry) {
        this.mSubsystem = subsystem;
        this.mController2 = controller2;
        this.mTelemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double leftTrigger = this.mController2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = this.mController2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        boolean intaking = rightTrigger > 0.0;
        boolean outtaking = leftTrigger > 0.0 && !intaking;
        if (intaking) {
            mSubsystem.intakePiece();
        } else if (outtaking) {
            mSubsystem.outtakePiece();
        } else {
            mSubsystem.stopIntake();
        }
        mTelemetry.addData("IntakePiece: intaking", intaking);
        mTelemetry.addData("IntakePiece: outtaking", outtaking);
    }

    @Override
    public void end(boolean interrupted) {
        mSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
