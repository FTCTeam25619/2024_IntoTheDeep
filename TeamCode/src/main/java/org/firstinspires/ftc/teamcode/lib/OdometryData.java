package org.firstinspires.ftc.teamcode.lib;

import static java.lang.Integer.valueOf;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.EncoderDataBlock;

import org.firstinspires.ftc.teamcode.Constants.OctoQuad.OctoQuadChannel;

public final class OdometryData {
    public double leftOdometerPosition;
    public double rightOdometerPosition;
    public double perpendicularOdometerPosition;
    public double leftOdometerVelocity;
    public double rightOdometerVelocity;
    public double perpendicularOdometerVelocity;
    public Rotation2d gyroHeading;

    public OdometryData(EncoderDataBlock encoderData, Rotation2d gyroHeading) {
        this.leftOdometerPosition = DriveHelpers.odometryRawToMeters(
                encoderData.positions[OctoQuadChannel.OdometryLeft.channelId]);
        this.rightOdometerPosition = DriveHelpers.odometryRawToMeters(
                encoderData.positions[OctoQuadChannel.OdometryRight.channelId]);
        this.perpendicularOdometerPosition = DriveHelpers.odometryRawToMeters(
                encoderData.positions[OctoQuadChannel.OdometryPerp.channelId]);
        this.leftOdometerVelocity = DriveHelpers.odometryRawToMeters(
                encoderData.velocities[OctoQuadChannel.OdometryLeft.channelId]);
        this.rightOdometerVelocity = DriveHelpers.odometryRawToMeters(
                encoderData.velocities[OctoQuadChannel.OdometryRight.channelId]);
        this.perpendicularOdometerVelocity = DriveHelpers.odometryRawToMeters(
                encoderData.velocities[OctoQuadChannel.OdometryPerp.channelId]);
        this.gyroHeading = gyroHeading;
    }
}
