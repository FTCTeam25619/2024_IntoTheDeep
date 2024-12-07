package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;

public class AwaitGamePiece extends CommandBase {
    private final Depositor depositor;

    public AwaitGamePiece(Depositor subsystem) {
        depositor = subsystem;
    }

    @Override
    public void execute() {
        depositor.awaitPiece();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return depositor.seeingPiece();
    }
}
