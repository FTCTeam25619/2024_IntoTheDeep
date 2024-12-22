package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.actions.TestAutoAction;
import org.firstinspires.ftc.teamcode.lib.ActionCommand;

import java.util.Set;

public class TestAuto extends ActionCommand<TestAutoAction> {
    public TestAuto(TestAutoAction action, Set<Subsystem> requirements) {
        super(action, requirements);
    }
}
