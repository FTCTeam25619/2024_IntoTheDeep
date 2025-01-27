package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class TransferPiece extends CommandBase {
    private final SequentialCommandGroup commandGroup;

    public TransferPiece(Intake intake, Depositor depositor, RobotState robotState) {
        addRequirements(intake, depositor);

        // Initialize the command sequence
        commandGroup = new SequentialCommandGroup(
                new InstantCommand(() -> intake.pivotToPosition(Constants.Intake.PivotSetPosition.UP)),
                new WaitCommand(ConfigConstants.IntakeTiming.handoffWaitBeforeSlideMove),
                new InstantCommand(() -> intake.slideToPosition(Constants.Intake.SlideSetPosition.IN)),
                new WaitCommand(ConfigConstants.IntakeTiming.handoffWaitForMateMS),
                new InstantCommand(() -> intake.intakePiece()),
                new AwaitGamePiece(depositor),
                new InstantCommand(() -> intake.stopIntake()),
                new InstantCommand(() -> robotState.setSlowDriveMode(false))
        );
    }

    @Override
    public void initialize() {
        // Start the command group
        commandGroup.schedule();
    }

    @Override
    public void execute() {
        // No need to call anything here; the group handles execution.
    }

    @Override
    public void end(boolean interrupted) {
        // End the group if interrupted
        commandGroup.cancel();
    }

    @Override
    public boolean isFinished() {
        // Check if the group has finished
        return commandGroup.isFinished();
    }
}