package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants.OpModes.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakePiece  extends CommandBase {
    private final GamepadEx mController2;
    private final Intake mSubsystem;
    private final Telemetry mTelemetry;
    private final AllianceColor allianceColor;

    private boolean pieceSeen;

    public IntakePiece(Intake subsystem, GamepadEx controller2, AllianceColor allianceColor, Telemetry telemetry) {
        this.mSubsystem = subsystem;
        this.mController2 = controller2;
        this.mTelemetry = telemetry;
        this.allianceColor = allianceColor;

        pieceSeen = false;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double leftTrigger = this.mController2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = this.mController2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        boolean cmdIntake = rightTrigger > 0.0;
        boolean cmdOuttake = leftTrigger > 0.0;
        pieceSeen = pieceSeen || mSubsystem.seeingPiece(allianceColor);
        if (!cmdIntake) {
            pieceSeen = false;
        }
        boolean stopping = cmdIntake && pieceSeen;
        boolean intaking = cmdIntake && !stopping;
        boolean outtaking = cmdOuttake && !cmdIntake;
        if (intaking) {
            mSubsystem.intakePiece();
        } else if (stopping) {
            mSubsystem.stopIntake();
            pieceSeen = false;
        } else if (outtaking) {
            mSubsystem.outtakePiece();
        } else {
            mSubsystem.stopIntake();
        }
        mTelemetry.addData("IntakePiece: intaking", intaking);
        mTelemetry.addData("IntakePiece: stopping", stopping);
        mTelemetry.addData("IntakePiece: outtaking", outtaking);
        mTelemetry.addData("IntakePiece: Seen?", pieceSeen);
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
