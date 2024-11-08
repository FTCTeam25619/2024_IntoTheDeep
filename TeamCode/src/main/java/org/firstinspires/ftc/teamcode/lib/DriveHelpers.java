package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Constants;

public final class DriveHelpers {
    public static ChassisSpeeds getDesiredChassisSpeeds(double x, double y, double turn, Rotation2d currentGyro) {
        DriveSpec spec = new DriveSpec(x, y, turn);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                spec.xScaledMPS, spec.yScaledMPS, spec.turnRPS, currentGyro);
    }

    public static double odometryRawToMeters(double ticks) {
        return ticks * 2.0 * Math.PI * Constants.DriveBase.WHEEL_RADIUS_MM / 1000.0;
    }
}
