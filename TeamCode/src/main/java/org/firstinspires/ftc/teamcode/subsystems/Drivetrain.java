package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Constants.HardwareMapping;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.lib.DriveHelpers;

public class Drivetrain extends SubsystemBase {
    private Motor frontLeftWheel;
    private Motor frontRightWheel;
    private Motor backLeftWheel;
    private Motor backRightWheel;

    private MecanumDrive mecanumDrive;

    private Telemetry mTelemetry;
    private RobotState mRobotState;
    private Sensors mSensors;

    public Drivetrain(HardwareMap hardwareMap, Sensors sensors, RobotState robotState, Telemetry telemetry) {
        frontLeftWheel = new Motor(hardwareMap, HardwareMapping.frontLeftWheel);
        frontRightWheel = new Motor(hardwareMap, HardwareMapping.frontRightWheel);
        backLeftWheel = new Motor(hardwareMap, HardwareMapping.backLeftWheel);
        backRightWheel = new Motor(hardwareMap, HardwareMapping.backRightWheel);

        frontLeftWheel.setInverted(true);
        frontRightWheel.setInverted(false);
        backRightWheel.setInverted(false);
        backLeftWheel.setInverted(true);

        frontLeftWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeftWheel.setRunMode(Motor.RunMode.RawPower);
        frontRightWheel.setRunMode(Motor.RunMode.RawPower);
        backLeftWheel.setRunMode(Motor.RunMode.RawPower);
        backRightWheel.setRunMode(Motor.RunMode.RawPower);

        mSensors = sensors;
        mRobotState = robotState;
        mTelemetry = telemetry;

        // Create the FTCLib MecanumDrive object that actually drives the wheel powers
        // Only used for FTCLib drive mode.
        boolean autoInvertRight = false;
        mecanumDrive = new MecanumDrive(autoInvertRight, frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel);
    }

    @Override
    public void periodic() {
        mTelemetry.addData("RobotState: SLOW MODE", mRobotState.slowDriveMode);
        mTelemetry.addData("RobotState: FIELD CENTRIC", mRobotState.robotDriveMode.fieldCentric);
        mTelemetry.addData("RobotState: FTCLIB DRIVE CODE", mRobotState.robotDriveMode.ftcLibDriveControl);

        mTelemetry.addData("Dt: Odom Elapsed ns", SystemClock.elapsedRealtimeNanos() - mSensors.odomTimestampNanos);
        mTelemetry.addData("Dt: Odom Read TS ns", mSensors.odomTimestampNanos);
        mTelemetry.addData("Dt: L Odom Pos m", mSensors.odomLeftDistanceMeters);
        mTelemetry.addData("Dt: R Odom Pos m", mSensors.odomRightDistanceMeters);
        mTelemetry.addData("Dt: P Odom Pos m", mSensors.odomPerpDistanceMeters);
        mTelemetry.addData("Dt: Gyro Heading", mSensors.gyroHeadingDegrees);
    }

    public double getGyroHeadingRad() {
        return mSensors.getGyroHeadingRad();
    }

    public void stopDrive(){
        frontLeftWheel.set(0.0);
        frontRightWheel.set(0.0);
        backLeftWheel.set(0.0);
        backRightWheel.set(0.0);
    }


    /*
     * Method to drive the robot open loop field centric based on a directional
     * vector and turn input. Expected inputs are calculated power and angle
     * theta from joystick raw input and turn as joystick raw input.
     * Theta angle is counter-rotated against the gyro heading to compute the
     * correct drive angle.
     * Deadband and smoothing is expected to be already applied.
     */
    public void driveRobotFieldCentric(double power, double theta, double turn) {
        if (mRobotState.robotDriveMode.ftcLibDriveControl) {
            double strafeSpeed = power * Math.sin(theta);
            double forwardSpeed = power * Math.cos(theta);
            double gyroHeading = mSensors.getGyroHeadingDeg();
            this.mecanumDrive.driveFieldCentric(-strafeSpeed, forwardSpeed, -turn, gyroHeading);
        } else {
            double adjustedTheta = theta - mSensors.getGyroHeadingRad();
            driveRobot(power, adjustedTheta, turn);
        }
    }

    /*
     * Method to drive the robot open loop based on a directional vector and turn input.
     * Expected inputs are calculated power and angle theta from joystick raw input
     * and turn as joystick raw input. Deadband and smoothing is expected to be already
     * applied. This is robot-centric drive only.
     */
    public void driveRobot(double power, double theta, double turn){
        if (mRobotState.robotDriveMode.ftcLibDriveControl) {
            double strafeSpeed = power * Math.sin(theta);
            double forwardSpeed = power * Math.cos(theta);
            this.mecanumDrive.driveRobotCentric(-strafeSpeed, forwardSpeed, -turn);
        } else {
            // Rotate theta + pi/4 because + pi/2 for coord rotation, - pi/4 for roller angle
            double sinTheta = Math.sin(-theta + Math.PI/4);
            double cosTheta = Math.cos(-theta + Math.PI/4);

            mTelemetry.addData("sin(theta)", sinTheta);
            mTelemetry.addData("cos(theta)", cosTheta);

            // Rescale power to account for max sqrt(2)/2 on pi/4 at 0 angle.
            power = power * Math.sqrt(2.0);

            double leftFront = power * sinTheta - turn;
            double rightFront = power * cosTheta + turn;
            double leftRear = power * cosTheta - turn;
            double rightRear = power * sinTheta + turn;

            mTelemetry.addData("LF power (prescale)", leftFront);
            mTelemetry.addData("RF power (prescale)", rightFront);
            mTelemetry.addData("LR power (prescale)", leftRear);
            mTelemetry.addData("RR power (prescale)", rightRear);

            double scaling = DriveHelpers.computeWheelScaling(leftFront, rightFront, leftRear, rightRear);

            mTelemetry.addData("Scaling factor", scaling);

            if (scaling > 1.0){
                leftFront /= scaling;
                rightFront /= scaling;
                leftRear /= scaling;
                rightRear /= scaling;
            }

            mTelemetry.addData("LF power (final)", leftFront);
            mTelemetry.addData("RF power (final)", rightFront);
            mTelemetry.addData("LR power (final)", leftRear);
            mTelemetry.addData("RR power (final)", rightRear);

            frontLeftWheel.set(leftFront);
            frontRightWheel.set(rightFront);
            backLeftWheel.set(leftRear);
            backRightWheel.set(rightRear);
        }
    }
}

