package org.firstinspires.ftc.teamcode.lib.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.function.DoubleSupplier;

public class TriggerAxis extends Trigger implements DoubleSupplier {
    GamepadEx mGamepad;
    GamepadKeys.Trigger mTrigger;
    double dThreshold;
    public TriggerAxis(GamepadEx gamepad, GamepadKeys.Trigger trigger, double threshold)  {
        mGamepad = gamepad;
        mTrigger = trigger;
        dThreshold = threshold;
    }

    @Override
    public boolean get() {
        return (mGamepad.getTrigger(mTrigger) > dThreshold);
    }
    public double getAsDouble() {
        return mGamepad.getTrigger(mTrigger);
    }
}

