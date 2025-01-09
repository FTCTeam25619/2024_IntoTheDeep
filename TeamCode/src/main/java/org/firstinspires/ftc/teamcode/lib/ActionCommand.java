package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

public class ActionCommand<A extends Action> implements Command {
    protected A mAction;
    protected Set<Subsystem> sRequirements;
    protected Telemetry mTelemetry;
    private boolean finished = false;

    public ActionCommand(A action, Set<Subsystem> requirements, Telemetry robotTelemetry) {
        this.mAction = action;
        this.mTelemetry = robotTelemetry;
        this.sRequirements = requirements;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return this.sRequirements;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        mAction.preview(packet.fieldOverlay());
        finished = !mAction.run(packet);
        mTelemetry.addData("ActionCommand finished", finished);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
