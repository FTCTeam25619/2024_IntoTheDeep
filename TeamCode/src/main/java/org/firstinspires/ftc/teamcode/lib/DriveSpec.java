package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.teamcode.Constants;

public final class DriveSpec {
    public final double xRawAxis;
    public final double yRawAxis;

    public final double turnRawAxis;

    public final double powerRaw;

    public final double powerMPS;
    public final double angleRad;
    public final double turnRPS;


    public final double xScaledMPS;
    public final double yScaledMPS;
    public final double turnScaledRPS;

    public static final double driveSpeedScaling =
            Constants.DriveControl.THEORETICAL_MAX_DRIVE_SPEED_MPS *
            Constants.DriveControl.ACHIEVABLE_MAX_DRIVE_SPEED_FACTOR;
    public static final double turnScaling = Math.PI;

    public DriveSpec(double xAxis, double yAxis, double turnAxis) {
        this.xRawAxis = xAxis;
        this.yRawAxis = yAxis;
        this.turnRawAxis = turnAxis;

        this.powerRaw = applyDrivePowerSmoothing(computePowerRaw(this.xRawAxis, this.yRawAxis));
        this.powerMPS = applyDrivePowerDeadZone(computeScaledPowerMPS(this.powerRaw));
        this.angleRad = computeAngleRad(this.xRawAxis, this.yRawAxis);
        this.turnRPS = applyTurnSpeedDeadZone(computeTurnRPS(this.turnRawAxis));

        this.xScaledMPS = Math.cos(this.angleRad - Math.PI/4) * this.powerMPS;
        this.yScaledMPS = Math.sin(this.angleRad - Math.PI/4) * this.powerMPS;
        this.turnScaledRPS = applyTurnSpeedLimiting(this.turnRPS);
    }

    public static double computePowerRaw(double x, double y) {
        return Math.hypot(y, x);
    }

    public static double computeAngleRad(double x, double y) {
        return Math.atan2(y, x);
    }

    public static double computeScaledPowerMPS(double powerRaw) {
        return powerRaw * driveSpeedScaling;
    }

    public static double computeTurnRPS(double turnRaw) {
        return turnRaw * turnScaling;
    }

    public static double applyDrivePowerSmoothing(double powerRaw) {
        return powerRaw * powerRaw * powerRaw;
    }

    public static double applyDrivePowerDeadZone(double powerMPS) {
        if (powerMPS < Constants.DriveControl.POWER_DEADZONE_THRESHOLD_MPS) {
            return 0.0;
        }
        return powerMPS;
    }

    public static double applyTurnSpeedDeadZone(double turnRPS) {
        if (turnRPS < Constants.DriveControl.TURN_DEADZONE_THRESHOLD_RPS) {
            return 0.0;
        }
        return turnRPS;
    }

    public static double applyTurnSpeedLimiting(double turnRPS) {
        if (Math.abs(turnRPS) > Constants.DriveControl.TURN_SPEED_LIMIT_RPS) {
            return Math.signum(turnRPS) * Constants.DriveControl.TURN_SPEED_LIMIT_RPS;
        }
        return turnRPS;
    }
}
