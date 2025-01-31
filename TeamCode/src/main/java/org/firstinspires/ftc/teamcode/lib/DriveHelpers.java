package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Constants;

public final class DriveHelpers {
    public static ChassisSpeeds getDesiredChassisSpeeds(double x, double y, double turn, Rotation2d currentGyro) {
        return new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    public static double smoothJoystick(double rawInput) {
        double clamped = rawInput;
        if (Math.abs(rawInput) > 1.0) {
            clamped = Math.signum(rawInput);
        }
        return clamped * clamped * clamped;
    }

    public static double smoothJoystickExponential(double rawInput, double k) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput)); // Clamp to [-1, 1]
        return Math.signum(clamped) * Math.pow(Math.abs(clamped), k);
    }

    public static double smoothJoystickPolynomial(double rawInput, double a, double b) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput)); // Clamp to [-1, 1]
        return a * Math.pow(clamped, 3) + b * clamped;
    }

    public static double deadBandJoystick(double rawInput, double bandSize) {
        if (Math.abs(rawInput) <= bandSize) {
            return 0.0; // Input within deadband range
        }

        // Scale the input to compensate for the deadband's removal
        double scaledInput = (Math.abs(rawInput) - bandSize) / (1.0 - bandSize);
        return Math.signum(rawInput) * scaledInput;
    }

    public static double computeWheelScaling(double leftFront, double rightFront, double leftRear, double rightRear) {
        return Math.max(
                Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                Math.max(Math.abs(leftRear), Math.abs(rightRear))
        );
    }
}
