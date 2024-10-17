package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveRobot extends CommandBase {
    private final GamepadEx mController1;
    private final Drivetrain mSubsystem;

    public DriveRobot(Drivetrain subsystem, GamepadEx controller1) {
        mSubsystem = subsystem;
        mController1 = controller1;

        addRequirements(mSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double x = mController1.getLeftY();
        double y = mController1.getLeftX();
        double turn = mController1.getRightX();

        double power = Math.hypot(y,x);
        double theta = Math.atan2(y,x);
        mSubsystem.driveRobot(power, theta, turn);
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
