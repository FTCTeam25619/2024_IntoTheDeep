package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.RevIMU;

public class Sensors extends SubsystemBase {
    /*
     * This uses our implementation of the newly updated but not let released version
     * of the FTCLib RevIMU class.
     * See org.ftcinspires.ftc.teamcode.lib.RevIMU
     */
    private final RevIMU gyro;
    /*
     * OctoQuad FTC code in this class is adapted from these examples:
     * - https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorOctoQuad.java
     * - https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorOctoQuadAdv.java
     */
    private final OctoQuad octoQuad;

    private final Motor odomLeftMotor;
    private final Motor odomRightMotor;
    private final Motor odomPerpMotor;
    private final Motor.Encoder odomLeft;
    private final Motor.Encoder odomRight;
    private final Motor.Encoder odomPerp;
    private final Telemetry mTelemetry;

    // These variables are here to cache values each cycle
    // allowing for a single (mostly) synchronized read from
    // the sensors but multiple retrieval
    public double odomLeftDistanceMeters;
    public double odomRightDistanceMeters;
    public double odomPerpDistanceMeters;
    public double gyroHeadingDegrees;
    public long odomTimestampNanos;
    private OctoQuad.EncoderDataBlock encoderDataBlock;
    public int liftLeftEncoderPosition;
    public int liftRightEncoderPosition;
    public int climbLeftEncoderPosition;
    public int climbRightEncoderPosition;

    private boolean octoQuadReady = false;

    public Sensors(HardwareMap hardwareMap, RevHubOrientationOnRobot gyroOrientation, Telemetry telemetry) {
        this.gyro = new RevIMU(hardwareMap, Constants.HardwareMapping.imu);
        this.octoQuad = hardwareMap.get(OctoQuad.class, Constants.HardwareMapping.octoQuad);
        this.odomLeftMotor = new Motor(hardwareMap, Constants.HardwareMapping.leftOdometryPod);
        this.odomRightMotor = new Motor(hardwareMap, Constants.HardwareMapping.rightOdometryPod);
        this.odomPerpMotor = new Motor(hardwareMap, Constants.HardwareMapping.perpOdometryPod);
        this.odomLeft = this.odomLeftMotor.encoder;
        this.odomRight = this.odomRightMotor.encoder;
        this.odomPerp = this.odomPerpMotor.encoder;

        this.odomLeft.setDirection(Motor.Direction.REVERSE);
        this.odomRight.setDirection(Motor.Direction.FORWARD);
        this.odomPerp.setDirection(Motor.Direction.FORWARD);

        this.odomLeft.setDistancePerPulse(2.0 * Math.PI * Constants.DriveBase.WHEEL_RADIUS_MM / 1000.0);
        this.odomRight.setDistancePerPulse(2.0 * Math.PI * Constants.DriveBase.WHEEL_RADIUS_MM / 1000.0);
        this.odomPerp.setDistancePerPulse(2.0 * Math.PI * Constants.DriveBase.WHEEL_RADIUS_MM / 1000.0);

        this.mTelemetry = telemetry;

        this.octoQuadReady = false;
        this.gyro.init(gyroOrientation);
        resetGyro();
        this.initializeOctoQuad();
    }

    @Override
    public void periodic() {
        readOctoQuadSensors();
        readOdometrySensors();
        mTelemetry.addData("Sensors: L Lift Enc", liftLeftEncoderPosition);
        mTelemetry.addData("Sensors: R Lift Enc", liftRightEncoderPosition);
        mTelemetry.addData("Sensors: L Climb Enc", climbLeftEncoderPosition);
        mTelemetry.addData("Sensors: R Climb Enc", climbRightEncoderPosition);
        mTelemetry.addData("Sensors: L Odom Pos m", odomLeftDistanceMeters);
        mTelemetry.addData("Sensors: R Odom Pos m", odomRightDistanceMeters);
        mTelemetry.addData("Sensors: P Odom Pos m", odomPerpDistanceMeters);
        mTelemetry.addData("Sensors: Gyro Heading deg", gyroHeadingDegrees);
        mTelemetry.addData("Sensors: Odom TS ns", odomTimestampNanos);
    }

    public void initializeOctoQuad() {
        // Reverse the count-direction of any encoder that is incorrect.
        // e.g. if you push the robot forward and the left encoder counts down, then reverse it so it counts up.
        this.octoQuad.setSingleEncoderDirection(
                Constants.OctoQuad.OctoQuadChannel.LiftLeftEncoder.channelId,
                OctoQuad.EncoderDirection.FORWARD);
        this.octoQuad.setSingleEncoderDirection(
                Constants.OctoQuad.OctoQuadChannel.LiftRightEncoder.channelId,
                OctoQuad.EncoderDirection.FORWARD);
        this.octoQuad.setSingleEncoderDirection(
                Constants.OctoQuad.OctoQuadChannel.ClimbLeftEncoder.channelId,
                OctoQuad.EncoderDirection.FORWARD);
        this.octoQuad.setSingleEncoderDirection(
                Constants.OctoQuad.OctoQuadChannel.ClimbRightEncoder.channelId,
                OctoQuad.EncoderDirection.FORWARD);

        // Set sample rates for velocity reads from OctoQuad channels
        this.octoQuad.setSingleVelocitySampleInterval(
                Constants.OctoQuad.OctoQuadChannel.LiftLeftEncoder.channelId,
                Constants.Sensors.ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS);
        this.octoQuad.setSingleVelocitySampleInterval(
                Constants.OctoQuad.OctoQuadChannel.LiftRightEncoder.channelId,
                Constants.Sensors.ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS);
        this.octoQuad.setSingleVelocitySampleInterval(
                Constants.OctoQuad.OctoQuadChannel.ClimbLeftEncoder.channelId,
                Constants.Sensors.ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS);
        this.octoQuad.setSingleVelocitySampleInterval(
                Constants.OctoQuad.OctoQuadChannel.ClimbRightEncoder.channelId,
                Constants.Sensors.ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS);

        // Any changes that are made should be saved in FLASH just in case there is a sensor power glitch.
        this.octoQuad.saveParametersToFlash();
        this.encoderDataBlock = new OctoQuad.EncoderDataBlock();
        this.octoQuadReady = true;
    }

    /*
     * Takes a reading of all OctoQuad encoder data channels and stores the data in
     * the encoderDataBlock instance variable
     */
    public void readOctoQuadSensors() {
        if (this.octoQuadReady) {
            this.octoQuad.readAllEncoderData(this.encoderDataBlock);
            this.liftLeftEncoderPosition = this.getOctoQuadValue(Constants.OctoQuad.OctoQuadChannel.LiftLeftEncoder);
            this.liftRightEncoderPosition = this.getOctoQuadValue(Constants.OctoQuad.OctoQuadChannel.LiftRightEncoder);
            this.climbLeftEncoderPosition = this.getOctoQuadValue(Constants.OctoQuad.OctoQuadChannel.ClimbLeftEncoder);
            this.climbRightEncoderPosition = this.getOctoQuadValue(Constants.OctoQuad.OctoQuadChannel.ClimbRightEncoder);
        }
    }

    public void readOdometrySensors() {
        this.odomTimestampNanos = SystemClock.elapsedRealtimeNanos();
        this.odomLeftDistanceMeters = this.odomLeft.getDistance();
        this.odomRightDistanceMeters = this.odomRight.getDistance();
        this.odomPerpDistanceMeters = this.odomPerp.getDistance();
        this.gyroHeadingDegrees = this.gyro.getHeading(AngleUnit.DEGREES);
    }

    private int getOctoQuadValue(Constants.OctoQuad.OctoQuadChannel channel) {
        return this.encoderDataBlock.positions[channel.channelId];
    }

    /*
     * Gets the current heading of the gyro in degrees.
     * This heading is offset from the last gyro reset.
     */
    public double getGyroHeadingDeg() {
        return this.gyro.getHeading();
    }

    /*
     * Gets the current heading of the gyro in radians.
     * This heading is offset from the last gyro reset.
     */
    public double getGyroHeadingRad() {
        return this.gyro.getHeading(AngleUnit.RADIANS);
    }

    /*
     * Gets the current heading of the gyro as a Rotation2d.
     * This heading is offset from the last gyro reset.
     */
    public Rotation2d getGyroRotation2d() {
        return this.gyro.getRotation2d();
    }

    /*
     * Resets the offset of the gyro such that getGyroHeadingDeg will return 0.
     */
    public void resetGyro() {
        this.gyro.reset();
    }

    /*
     * Inverts the reading of the gyro.
     */
    public void invertGyro() {
        this.gyro.invertGyro();
    }

    /*
     * Gets the raw absolute reading from the gyro in degrees, without offset.
     */
    public double getGyroAbsoluteHeadingDeg() {
        return this.gyro.getAbsoluteHeading();
    }
}
