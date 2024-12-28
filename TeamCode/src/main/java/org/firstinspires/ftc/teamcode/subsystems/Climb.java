package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Climb extends SubsystemBase {
    private Motor leftMotor;
    private Motor rightMotor;
    private AnalogInput absoluteEncoder;

    private Sensors mSensors;
    private Telemetry mTelemetry;

    // TODO: This is currently volts -- should actually be cm.
    public double minPositionCM = 0.5;
    public double maxPositionCM = 3.0;

    public Climb(HardwareMap hardwareMap, Sensors sensors, Telemetry telemetry){
        leftMotor = new Motor(hardwareMap, Constants.HardwareMapping.climbLeftMotor);
        rightMotor = new Motor(hardwareMap, Constants.HardwareMapping.climbRightMotor);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        mSensors = sensors;
        mTelemetry = telemetry;
    }

    @Override
    public void periodic() {
        mTelemetry.addData("Climb: L Enc", mSensors.climbLeftEncoderPosition);
        mTelemetry.addData("Climb: R Enc", mSensors.climbRightEncoderPosition);
    }

    public void setMotorPower(double power){
        if (power > 1.0){ power = 1.0;}
        if (power < -1.0){ power = -1.0;}

        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void stopMotors() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }
}
