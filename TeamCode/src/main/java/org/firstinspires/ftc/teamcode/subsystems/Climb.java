package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;

public class Climb extends SubsystemBase {
    private Motor leftMotor;
    private Motor rightMotor;

    private Sensors mSensors;
    private Telemetry mTelemetry;

    private PIDController leftPIDController;
    private PIDController rightPIDController;
    private double leftPower;
    private double rightPower;
    private boolean enabledPID = false;
    private int leftPIDTarget;
    private int rightPIDTarget;

    public Climb(HardwareMap hardwareMap, Sensors sensors, Telemetry telemetry){
        leftMotor = new Motor(hardwareMap, Constants.HardwareMapping.climbLeftMotor);
        rightMotor = new Motor(hardwareMap, Constants.HardwareMapping.climbRightMotor);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        leftPIDController = new PIDController(
                ConfigConstants.Climb.kPLeft,
                ConfigConstants.Climb.kILeft,
                ConfigConstants.Climb.kDLeft
        );
        rightPIDController = new PIDController(
                ConfigConstants.Climb.kPRight,
                ConfigConstants.Climb.kIRight,
                ConfigConstants.Climb.kDRight
        );

        mSensors = sensors;
        mTelemetry = telemetry;
    }

    @Override
    public void periodic() {
        mTelemetry.addData("Climb: L Enc", mSensors.climbLeftEncoderPosition);
        mTelemetry.addData("Climb: R Enc", mSensors.climbRightEncoderPosition);

        if (enabledPID) {
            double leftPower = leftPIDController.calculate() + ConfigConstants.Climb.kFLeft;
            double rightPower = rightPIDController.calculate() + ConfigConstants.Climb.kFRight;
            // TODO: Do we need to limit these for safety beyond -1 to 1? If so, how?
            if (leftPower > 1.0) { leftPower = 1.0; }
            if (leftPower < -1.0) { leftPower = -1.0; }
            if (rightPower > 1.0) { rightPower = 1.0; }
            if (rightPower < -1.0) { rightPower = -1.0; }
        }

        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    public void setMotorPower(double power){
        if (power > 1.0){ power = 1.0;}
        if (power < -1.0){ power = -1.0;}

        leftPower = power;
        rightPower = power;
    }

    public void stopMotors() {
        leftPower = 0.0;
        rightPower = 0.0;
        enabledPID = false;
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    public void enablePIDHold(boolean enable) {
        if (enable) {
            setPIDHoldTargets();
            resetPIDControllers();
            leftPIDController.setSetPoint(leftPIDTarget);
            rightPIDController.setSetPoint(rightPIDTarget);
        }
        enabledPID = enable;
    }

    private void setPIDHoldTargets() {
        leftPIDTarget = mSensors.climbLeftEncoderPosition;
        rightPIDTarget = mSensors.climbRightEncoderPosition;
    }

    private void resetPIDControllers() {
        leftPIDController.reset();
        rightPIDController.reset();
    }
}
