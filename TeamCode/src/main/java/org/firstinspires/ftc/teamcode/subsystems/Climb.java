package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;

public class Climb extends SubsystemBase {
    private final Motor leftMotor;
    private final Motor rightMotor;

    private Sensors mSensors;
    private Telemetry mTelemetry;
    private final PIDController holdPIDController;
    private final PIDController syncPIDController;

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    private boolean enabledPID = false;
    private int holdPIDTarget;
    private int countPIDTargets = 0;
    private int countPIDResets = 0;
    private double calculatedPIDpower = 0.0;
    private int bufferMultiplier = 1;
    private double syncCorrection = 0.0;

    public double targetPower = 1.0; // Target power for motors
    private static final double DEAD_BAND = 0.05; // Dead band to prevent small oscillations

    private int leftStartPosition = 0;
    private int rightStartPosition = 0;
    private int leftTravel = 0;
    private int rightTravel = 0;
    private State currentState;
    public enum State {
        HOLD,
        CLIMB,
        STOP
    }

    public Climb(HardwareMap hardwareMap, Sensors sensors, Telemetry telemetry) {
        // Initialize motors
        leftMotor = new Motor(hardwareMap, Constants.HardwareMapping.climbLeftMotor);
        rightMotor = new Motor(hardwareMap, Constants.HardwareMapping.climbRightMotor);

        leftMotor.setInverted(false); // Left motor is not inverted
        rightMotor.setInverted(true); // Right motor is inverted

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        // PID controller for synchronization
        syncPIDController = new PIDController(ConfigConstants.Climb.SYNC_KP, ConfigConstants.Climb.SYNC_KI, ConfigConstants.Climb.SYNC_KD);
        // PID controller for hold position
        holdPIDController = new PIDController(ConfigConstants.Climb.kP, ConfigConstants.Climb.kI, ConfigConstants.Climb.kD);

        mSensors = sensors;
        mTelemetry = telemetry;


        // Capture initial encoder positions for travel calculations
        leftStartPosition = leftMotor.getCurrentPosition();
        rightStartPosition = rightMotor.getCurrentPosition();

        currentState = State.STOP;
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case STOP:
                stopMotors();
                break;
            case CLIMB:
                // Calculate encoder travel for both motors
                leftTravel = leftMotor.getCurrentPosition() - leftStartPosition;
                rightTravel = rightMotor.getCurrentPosition() - rightStartPosition;

                // Synchronization correction based on encoder difference
                syncCorrection = syncPIDController.calculate(rightTravel, leftTravel);

                // Calculate motor power with synchronization adjustment
                leftPower = targetPower - syncCorrection;  // Subtract syncCorrection from left motor
                rightPower = targetPower + syncCorrection; // Add syncCorrection to right motor

                // Clamp motor power to valid range
                leftPower = clampPower(leftPower);
                rightPower = clampPower(rightPower);

                // Apply dead band
                leftPower = applyDeadBand(leftPower);
                rightPower = applyDeadBand(rightPower);

                // Set motor powers
                leftMotor.set(leftPower);
                rightMotor.set(rightPower);
                break;
            case HOLD:
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
                break;
        }

        // Telemetry for debugging
        mTelemetry.addData("Left Encoder Travel", leftTravel);
        mTelemetry.addData("Right Encoder Travel", rightTravel);
        mTelemetry.addData("Left Motor Power", leftPower);
        mTelemetry.addData("Right Motor Power", rightPower);
        mTelemetry.addData("Sync Correction", syncCorrection);
    }

    public void setMotorPower(double power){
        if (power > 1.0){ power = 1.0;}
        if (power < -1.0){ power = -1.0;}

        leftPower = power;
        rightPower = power;
    }
    public void stopMotors() {
        // Stop both motors and hold position
        leftMotor.stopMotor();
        rightMotor.stopMotor();
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

    private double clampPower(double power) {
        return Math.max(-targetPower, Math.min(targetPower, power));
    }

    private double applyDeadBand(double power) {
        return Math.abs(power) < DEAD_BAND ? 0.0 : power;
    }

    public void setClimbState(State state) {
        currentState = state;
    }
    public void setTargetPower(double power){
        targetPower = power;
    }
}
