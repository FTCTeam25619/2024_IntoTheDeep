package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class MoveLiftDown extends CommandBase {
    private Lift mLift;

    private final double motorPower = -0.5;

    public MoveLiftDown(Lift subsystem) {
        mLift = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        mLift.setMotorPower(motorPower);
    }

    @Override
    public void end(boolean interrupted) {
        mLift.stopMotors();
    }

    @Override
    public boolean isFinished() {
//        return mLift.getPositionCM() <= mLift.minPositionCM;
        return false;
    }
}
