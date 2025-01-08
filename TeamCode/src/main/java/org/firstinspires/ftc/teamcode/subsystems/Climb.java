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
    private int countPIDTargets = 0;
    private int countPIDResets = 0;
    private double calculatedPIDpower = 0.0;
    private int bufferMultiplier = 1;

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
        mTelemetry.addData("Climb: PID Targets set", countPIDTargets);
        mTelemetry.addData("Climb: PID Controller resets", countPIDResets);

        if (enabledPID) {
            // PID hold: recalculate power via PID controllers based on left motor encoder
            calculatedPIDpower = holdPIDController.calculate(mSensors.climbLeftEncoderPosition);
            leftPower = calculatedPIDpower + ConfigConstants.Climb.kF;
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

        if (leftPower > 0.0) { bufferMultiplier = 1; } else { bufferMultiplier = -1; }

        if ((leftPower > -0.05 && leftPower < 0.05) || (rightPower > -0.05 && rightPower < 0.05)) {
            leftPower = 0.0;
            rightPower = 0.0;
        }

        mTelemetry.addData("Climb: PID power", calculatedPIDpower);
        mTelemetry.addData("Climb: leftPower", leftPower);
        mTelemetry.addData("Climb: rightPower", rightPower);

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
        countPIDTargets++;
        holdPIDTarget = mSensors.climbLeftEncoderPosition +
                bufferMultiplier * ConfigConstants.Climb.movementBufferCounts;
    }

    private void resetPIDController() {
        countPIDResets++;
        holdPIDController.reset();
    }
}
