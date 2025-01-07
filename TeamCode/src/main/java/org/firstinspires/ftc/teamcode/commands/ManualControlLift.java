package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


public class ManualControlLift  extends CommandBase {
    private Lift mLift;
    private GamepadEx controller;
    private final double movementThreshold;

    public ManualControlLift(Lift subsystem, GamepadEx controller, double threshold) {
        this.mLift = subsystem;
        this.controller = controller;
        this.movementThreshold = threshold;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (controller.getLeftY() > movementThreshold) {
            mLift.setMotorPower(ConfigConstants.ManualMovement.liftUpMotorPower);
        } else if (controller.getLeftY() < -movementThreshold) {
            mLift.setMotorPower(ConfigConstants.ManualMovement.liftDownMotorPower);
        } else {
            mLift.stopMotors();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
