package org.firstinspires.ftc.teamcode.lib;

import static java.lang.Integer.valueOf;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.EncoderDataBlock;

import org.firstinspires.ftc.teamcode.Constants.OctoQuad.OctoQuadChannel;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

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
                encoderData.positions[OctoQuadChannel.OdometryLeft.channel]);
        this.rightOdometerPosition = DriveHelpers.odometryRawToMeters(
                encoderData.positions[OctoQuadChannel.OdometryRight.channel]);
        this.perpendicularOdometerPosition = DriveHelpers.odometryRawToMeters(
                encoderData.positions[OctoQuadChannel.OdometryPerp.channel]);
        this.leftOdometerVelocity = DriveHelpers.odometryRawToMeters(
                encoderData.velocities[OctoQuadChannel.OdometryLeft.channel]);
        this.rightOdometerVelocity = DriveHelpers.odometryRawToMeters(
                encoderData.velocities[OctoQuadChannel.OdometryRight.channel]);
        this.perpendicularOdometerVelocity = DriveHelpers.odometryRawToMeters(
                encoderData.velocities[OctoQuadChannel.OdometryPerp.channel]);
        this.gyroHeading = gyroHeading;
    }
}
