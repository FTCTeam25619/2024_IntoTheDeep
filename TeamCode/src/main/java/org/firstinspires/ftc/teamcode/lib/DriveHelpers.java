package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Constants;

public final class DriveHelpers {
    public static ChassisSpeeds getDesiredChassisSpeeds(double x, double y, double turn, Rotation2d currentGyro) {
        return new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    public static double odometryRawToMeters(double ticks) {
        return ticks * 2.0 * Math.PI * Constants.DriveBase.WHEEL_RADIUS_MM / 1000.0;
    }

    public static double smoothJoystick(double rawInput) {
        double clamped = rawInput;
        if (Math.abs(rawInput) > 1.0) {
            clamped = Math.signum(rawInput);
        }
        return clamped * clamped * clamped;
    }

    public static double deadBandJoystick(double rawInput, double bandSize) {
        if (Math.abs(rawInput) <= bandSize) {
            return 0.0;
        }
        return rawInput;
    }

    public static double computeWheelScaling(double leftFront, double rightFront, double leftRear, double rightRear) {
        return Math.max(
                Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                Math.max(Math.abs(leftRear), Math.abs(rightRear))
        );
    }
}
