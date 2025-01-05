package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Bitmap;

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

    private PIDController holdPIDController;
    private double leftPower;
    private double rightPower;
    private boolean enabledPID = false;
    private int holdPIDTarget;

    public Climb(HardwareMap hardwareMap, Sensors sensors, Telemetry telemetry){
        leftMotor = new Motor(hardwareMap, Constants.HardwareMapping.climbLeftMotor);
        rightMotor = new Motor(hardwareMap, Constants.HardwareMapping.climbRightMotor);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        holdPIDController = new PIDController(
                ConfigConstants.Climb.kP,
                ConfigConstants.Climb.kI,
                ConfigConstants.Climb.kD
        );
        holdPIDController.setTolerance(ConfigConstants.Climb.pidTolerance);

        mSensors = sensors;
        mTelemetry = telemetry;
    }

    @Override
    public void periodic() {
        mTelemetry.addData("Climb: L Enc", mSensors.climbLeftEncoderPosition);
        mTelemetry.addData("Climb: R Enc", mSensors.climbRightEncoderPosition);
        mTelemetry.addData("Climb: PID Target (L motor)", holdPIDTarget);
        mTelemetry.addData("Climb: PID Hold", enabledPID);

        if (enabledPID) {
            // PID hold: recalculate power via PID controllers based on left motor encoder
            leftPower = holdPIDController.calculate() + ConfigConstants.Climb.kF;
            if (leftPower > ConfigConstants.Climb.maxHoldMotorPower) {
                leftPower = ConfigConstants.Climb.maxHoldMotorPower;
            }
            if (leftPower < -ConfigConstants.Climb.maxHoldMotorPower) {
                leftPower = -ConfigConstants.Climb.maxHoldMotorPower;
            }
            rightPower = leftPower;
        } else {
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
        enabledPID = false;
        leftPower = 0.0;
        rightPower = 0.0;
    }

    public void enablePIDHold(boolean enable) {
        if (enable) {
            setPIDHoldTarget();
            resetPIDController();
            holdPIDController.setSetPoint(holdPIDTarget);
        }
        enabledPID = enable;
    }

    private void setPIDHoldTarget() {
        holdPIDTarget = mSensors.climbLeftEncoderPosition;
    }

    private void resetPIDController() {
        holdPIDController.reset();
    }
}
