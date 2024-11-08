package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.OdometryData;

public class Drivetrain extends SubsystemBase {
    private Motor frontLeftWheel;
    private Motor frontRightWheel;
    private Motor backLeftWheel;
    private Motor backRightWheel;

    private Telemetry mTelemetry;
    private Sensors mSensors;

    public Drivetrain(HardwareMap hardwareMap, Sensors sensors, Telemetry telemetry) {
        frontLeftWheel = new Motor(hardwareMap, Constants.HardwareMapping.frontLeftWheel);
        frontRightWheel = new Motor(hardwareMap, Constants.HardwareMapping.frontRightWheel);
        backLeftWheel = new Motor(hardwareMap, Constants.HardwareMapping.backLeftWheel);
        backRightWheel = new Motor(hardwareMap, Constants.HardwareMapping.backRightWheel);

        frontLeftWheel.setInverted(true);
        frontRightWheel.setInverted(true);
        backRightWheel.setInverted(true);
        backLeftWheel.setInverted(false);

        frontLeftWheel.setRunMode(Motor.RunMode.RawPower);
        frontRightWheel.setRunMode(Motor.RunMode.RawPower);
        backLeftWheel.setRunMode(Motor.RunMode.RawPower);
        backRightWheel.setRunMode(Motor.RunMode.RawPower);

        mSensors = sensors;
        mTelemetry = telemetry;
    }

    @Override
    public void periodic() {
        OdometryData odomData = mSensors.getOdometryData();
        mTelemetry.addData("L Odom Pos", odomData.leftOdometerPosition);
        mTelemetry.addData("R Odom Pos", odomData.rightOdometerPosition);
        mTelemetry.addData("P Odom Pos", odomData.perpendicularOdometerPosition);
        mTelemetry.addData("L Odom Vel", odomData.leftOdometerVelocity);
        mTelemetry.addData("R Odom Vel", odomData.rightOdometerVelocity);
        mTelemetry.addData("P Odom Vel", odomData.perpendicularOdometerVelocity);
        mTelemetry.addData("Gyro Heading", odomData.gyroHeading.getDegrees());
    }

    public void stopDrive(){
        frontLeftWheel.set(0.0);
        frontRightWheel.set(0.0);
        backLeftWheel.set(0.0);
        backRightWheel.set(0.0);
    }

    public void driveRobot(double power, double theta, double turn){
        double sinTheta = Math.sin(theta - Math.PI/4);
        double cosTheta = Math.cos(theta - Math.PI/4);
        double max = Math.max( Math.abs(sinTheta), Math.abs(cosTheta));
        sinTheta = sinTheta/max;
        cosTheta = cosTheta/max;

        double leftFront = power * cosTheta + turn;
        double rightFront = power * sinTheta - turn;
        double leftRear = power * sinTheta + turn;
        double rightRear = power * cosTheta - turn;
        double scaling = power + Math.abs(turn);

        if (scaling > 1){
            leftFront /= scaling;
            rightFront /= scaling;
            leftRear /= scaling;
            rightFront /= scaling;

        }

        frontLeftWheel.set(leftFront);
        frontRightWheel.set(rightFront);
        backLeftWheel.set(leftRear);
        backRightWheel.set(rightRear);
    }
}

