package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.OdometryData;
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
    private final Telemetry mTelemetry;

    private boolean octoQuadReady = false;
    private OctoQuad.EncoderDataBlock encoderDataBlock;

    public Sensors(HardwareMap hardwareMap, RevHubOrientationOnRobot gyroOrientation, Telemetry telemetry) {
        this.gyro = new RevIMU(hardwareMap);
        this.octoQuad = hardwareMap.get(OctoQuad.class, Constants.HardwareMapping.octoQuad);
        this.mTelemetry = telemetry;

        this.octoQuadReady = false;
        this.gyro.init(gyroOrientation);
        resetGyro();
        this.initializeOctoQuad();
    }

    @Override
    public void periodic() {
        OdometryData odomData = this.getOdometryData(true);
        mTelemetry.addData("Sensors: L Lift Enc", getLiftLeftEncoderCount());
        mTelemetry.addData("Sensors: R Lift Enc", getLiftRightEncoderCount());
        mTelemetry.addData("Sensors: L Odom Pos", odomData.leftOdometerPosition);
        mTelemetry.addData("Sensors: R Odom Pos", odomData.rightOdometerPosition);
        mTelemetry.addData("Sensors: P Odom Pos", odomData.perpendicularOdometerPosition);
        mTelemetry.addData("Sensors: L Odom Vel", odomData.leftOdometerVelocity);
        mTelemetry.addData("Sensors: R Odom Vel", odomData.rightOdometerVelocity);
        mTelemetry.addData("Sensors: P Odom Vel", odomData.perpendicularOdometerVelocity);
        mTelemetry.addData("Sensors: Gyro Heading", odomData.gyroHeading.getDegrees());
    }

    public void initializeOctoQuad() {
        // Reverse the count-direction of any encoder that is incorrect.
        // e.g. if you push the robot forward and the left encoder counts down, then reverse it so it counts up.
        this.octoQuad.setSingleEncoderDirection(
                Constants.OctoQuad.OctoQuadChannel.OdometryLeft.channelId,
                OctoQuad.EncoderDirection.REVERSE);
        this.octoQuad.setSingleEncoderDirection(
                Constants.OctoQuad.OctoQuadChannel.OdometryRight.channelId,
                OctoQuad.EncoderDirection.FORWARD);
        this.octoQuad.setSingleEncoderDirection(
                Constants.OctoQuad.OctoQuadChannel.OdometryPerp.channelId,
                OctoQuad.EncoderDirection.FORWARD);

        // Set sample rates for velocity reads from OctoQuad channels
        this.octoQuad.setSingleVelocitySampleInterval(
                Constants.OctoQuad.OctoQuadChannel.OdometryLeft.channelId,
                Constants.SensorRates.ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS);
        this.octoQuad.setSingleVelocitySampleInterval(
                Constants.OctoQuad.OctoQuadChannel.OdometryRight.channelId,
                Constants.SensorRates.ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS);
        this.octoQuad.setSingleVelocitySampleInterval(
                Constants.OctoQuad.OctoQuadChannel.OdometryPerp.channelId,
                Constants.SensorRates.ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS);

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
        }
    }

    private double getOctoQuadValue(Constants.OctoQuad.OctoQuadChannel channel) {
        return this.encoderDataBlock.positions[channel.channelId];
    }

    public double getLiftLeftEncoderCount() {
        return this.getOctoQuadValue(Constants.OctoQuad.OctoQuadChannel.LiftLeftEncoder);
    }

    public double getLiftRightEncoderCount() {
        return this.getOctoQuadValue(Constants.OctoQuad.OctoQuadChannel.LiftRightEncoder);
    }

    /*
     * Returns current odometry data as an OdometryData object (without updating)
     */
    public OdometryData getOdometryData() {
        return this.getOdometryData(false);
    }

    /*
     * Returns current odometry data as an OdometryData object, refreshing
     * that data from sensors when `refresh` is `true`
     */
    public OdometryData getOdometryData(boolean refresh) {
        if (refresh) { readOctoQuadSensors(); }
        return new OdometryData(this.encoderDataBlock, this.gyro.getRotation2d());
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
