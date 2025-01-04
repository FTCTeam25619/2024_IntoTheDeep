package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class MoveLiftToHeight extends CommandBase {
    private Lift mLift;
    private double mHeight;

    /**
     *
     * @param subsystem  Lift type subsystem
     * @param height    Target height in CM
     *
     *  Command moves lift ot target height and holds position
     */
    public MoveLiftToHeight(Lift subsystem, double height) {
        mLift = subsystem;
        mHeight = height;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        mLift.setHeight(mHeight);
    }

    @Override
    public void execute() {
        // Nothing more needed
    }

    @Override
    public void end(boolean interrupted) {
        mLift.disablePID();
    }

    @Override
    public boolean isFinished() {
        // Will hold position until interrupted
        // TODO:  Consider adding a timer to limit how long we hold power on motor
        return false;
    }

}
