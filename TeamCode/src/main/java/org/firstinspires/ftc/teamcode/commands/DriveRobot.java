package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.DriveHelpers;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveRobot extends CommandBase {
    private final GamepadEx mController1;
    private final Drivetrain mSubsystem;
    private final Telemetry mTelemetry;

    public DriveRobot(Drivetrain subsystem, GamepadEx controller1, Telemetry robotTelemetry) {
        mSubsystem = subsystem;
        mController1 = controller1;
        mTelemetry = robotTelemetry;

        addRequirements(mSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double rawY = mController1.getLeftY();
        double rawX = mController1.getLeftX();
        double rawTurn = mController1.getRightX();
//        mTelemetry.addData("Y input raw", rawY);
//        mTelemetry.addData("X input raw", rawX);
//        mTelemetry.addData("Turn input raw", rawTurn);

        double x = rawY;
        double y = rawX;
        double turn = -rawTurn;
//        mTelemetry.addData("Y input signed", y);
//        mTelemetry.addData("X input signed", x);
//        mTelemetry.addData("Turn input signed", turn);

        double power = Math.hypot(y, x);
        double theta = -Math.atan2(y, x);
        power = DriveHelpers.smoothJoystick(
                DriveHelpers.deadBandJoystick(
                        power,
                        Constants.DriveControl.POWER_DEADZONE_THRESHOLD_RAW));
        turn = DriveHelpers.smoothJoystick(
                DriveHelpers.deadBandJoystick(
                        turn,
                        Constants.DriveControl.TURN_DEADZONE_THRESHOLD_RAW));

        if (Constants.RobotModes.SLOW_DRIVE_MODE) {
            power /= Constants.DriveControl.SLOW_DRIVE_MODE_FACTOR;
            turn /= Constants.DriveControl.SLOW_DRIVE_MODE_FACTOR;
        }

        mTelemetry.addData("Drive power final", power);
        mTelemetry.addData("Drive angle final", theta);
        mTelemetry.addData("Drive turn  final", turn);

        if (Constants.RobotModes.FIELD_CENTRIC_DRIVE) {
            mSubsystem.driveRobotFieldCentric(power, theta, turn);
        } else {
            mSubsystem.driveRobot(power, theta, turn);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mSubsystem.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
