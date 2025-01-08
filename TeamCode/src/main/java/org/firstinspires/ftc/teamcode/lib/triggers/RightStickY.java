package org.firstinspires.ftc.teamcode.lib.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class RightStickY extends Trigger {
    private final GamepadEx controller;
    private final double threshold;
    private final boolean up;

    public RightStickY(GamepadEx controller, double threshold) {
        this.controller = controller;
        this.threshold = Math.max(Math.min(threshold, 1.0), -1.0);
        this.up = this.threshold > 0.0;
    }

    @Override
    public boolean get() {
        if (up) {
            return controller.getRightY() > threshold;
        } else {
            return controller.getRightY() < threshold;
        }
    }
}
