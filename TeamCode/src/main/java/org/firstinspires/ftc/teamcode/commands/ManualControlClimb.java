package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.subsystems.Climb;


public class ManualControlClimb  extends CommandBase {
    private Climb mClimb;
    private GamepadEx controller;
    private final double movementThreshold;

    public ManualControlClimb(Climb subsystem, GamepadEx controller, double threshold) {
        this.mClimb = subsystem;
        this.controller = controller;
        this.movementThreshold = threshold;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (controller.getRightY() > movementThreshold) {
            mClimb.setMotorPower(ConfigConstants.ManualMovement.climbUpMotorPower);
        } else if (controller.getRightY() < -movementThreshold) {
            mClimb.setMotorPower(ConfigConstants.ManualMovement.climbDownMotorPower);
        } else {
            mClimb.stopMotors();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
