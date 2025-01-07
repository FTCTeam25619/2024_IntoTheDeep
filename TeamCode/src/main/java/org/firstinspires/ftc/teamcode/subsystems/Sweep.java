package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;

public class Sweep  extends SubsystemBase {
    private final Servo sweeper;

    private final Telemetry mTelemetry;

    public Sweep(HardwareMap hardwareMap, Telemetry telemetry) {
        sweeper = hardwareMap.get(Servo.class, Constants.HardwareMapping.sweepServo);

        mTelemetry = telemetry;
    }

    public void sweepToPosition(Constants.Depositor.SweepSetPosition setPosition) {
        sweeper.setPosition(setPosition.position);
    }

    public void sweepToTestPosition() {
        sweeper.setPosition(ConfigConstants.TestPositions.sweepTest);
    }
}
