package org.firstinspires.ftc.teamcode.lib.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class TriggerButton extends Trigger {
    private final GamepadEx controller;
    private final GamepadKeys.Trigger button;
    private final double threshold;

    public TriggerButton(GamepadEx controller, GamepadKeys.Trigger button, double threshold) {
        this.controller = controller;
        this.button = button;
        this.threshold = Math.max(Math.min(threshold, 1.0), 0.0);
    }

    @Override
    public boolean get() {
        return controller.getTrigger(button) > threshold;
    }
}
