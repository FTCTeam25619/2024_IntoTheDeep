package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.lib.DriveHelpers;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import org.firstinspires.ftc.teamcode.subsystems.Sensors;

public class DriveRobot extends CommandBase {
    private final GamepadEx mController1;
    private final Drivetrain mSubsystem;
    private final Telemetry mTelemetry;
    private final RobotState mRobotState;
    private final Sensors mSensors;

    private static final double BUCKET_HEADING_DEGREES = 45.0; // Bucket heading in degrees
    private static final double SUBMERSIBLE_HEADING_DEGREES = 90.0; // Submersible heading in degrees
    private double targetHeadingDegrees = 0.0;
    private boolean isOrientationLockActive = false;

    public DriveRobot(Drivetrain subsystem, GamepadEx controller1, RobotState robotState, Telemetry robotTelemetry, Sensors sensors) {
        mSubsystem = subsystem;
        mController1 = controller1;
        mRobotState = robotState;
        mTelemetry = robotTelemetry;
        mSensors = sensors;

        addRequirements(mSubsystem, mSensors);
    }

    @Override
    public void initialize() {
        isOrientationLockActive = false;
    }

    @Override
    public void execute() {
        double rawY = mController1.getLeftY();
        double rawX = mController1.getLeftX();
        double rawTurn = mController1.getRightX();

        // Transform raw inputs to coordinate system:
        double x = rawY;
        double y = rawX;
        double turn = -rawTurn;

        // Prepare drive inputs
        double power = Math.hypot(y, x);
        double theta = -Math.atan2(y, x);
        power = prepareDriveInputs(power,
                ConfigConstants.DriveControl.POWER_DEADZONE_THRESHOLD_RAW,
                ConfigConstants.DriveControl.SLOW_DRIVE_MODE_POWER_FACTOR);
        turn = prepareDriveInputs(turn,
                ConfigConstants.DriveControl.TURN_DEADZONE_THRESHOLD_RAW,
                ConfigConstants.DriveControl.SLOW_DRIVE_MODE_TURN_FACTOR);

        // Handle orientation lock logic
        handleOrientationLock();

        // Apply orientation lock correction if active
        if (isOrientationLockActive) {
            turn = calculateOrientationCorrection(targetHeadingDegrees);
        }

        // Drive the robot
        if (mRobotState.robotDriveMode.fieldCentric) {
            mSubsystem.driveRobotFieldCentric(power, theta, turn);
        } else {
            mSubsystem.driveRobot(power, theta, turn);
        }

        // Debug telemetry
        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            double currentHeading = mSensors.getGyroHeadingDeg();
            double headingError = isOrientationLockActive ? (targetHeadingDegrees - currentHeading) : 0.0;
            mTelemetry.addData("Y input raw", rawY);
            mTelemetry.addData("X input raw", rawX);
            mTelemetry.addData("Turn input raw", rawTurn);
            mTelemetry.addData("Preset Heading", targetHeadingDegrees);
            mTelemetry.addData("Current Heading", currentHeading);
            mTelemetry.addData("Heading Error", headingError);
            mTelemetry.addData("Orientation Lock Active", isOrientationLockActive);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mSubsystem.stopDrive();
        isOrientationLockActive = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double prepareDriveInputs(double input, double deadBandSize, double slowModeFactor) {
        double prepared = DriveHelpers.deadBandJoystick(input, deadBandSize);
        prepared = DriveHelpers.smoothJoystick(prepared);
        if (mRobotState.slowDriveMode) {
            prepared /= slowModeFactor;
        }
        return prepared;
    }

    private void handleOrientationLock() {
        boolean bucketOrientationLockButtonPressed = mController1.getButton(GamepadKeys.Button.A);
        boolean submersibleOrientationLockButtonPressed = mController1.getButton(GamepadKeys.Button.B);
        if (bucketOrientationLockButtonPressed) {
            isOrientationLockActive = true;
            targetHeadingDegrees = BUCKET_HEADING_DEGREES;
        } else if (submersibleOrientationLockButtonPressed) {
            isOrientationLockActive = true;
            targetHeadingDegrees = SUBMERSIBLE_HEADING_DEGREES;
        } else {
            isOrientationLockActive = false;
            targetHeadingDegrees = 0.0; // Reset target heading when not active
        }
    }

    private double calculateOrientationCorrection(double targetHeading) {
        double currentHeading = mSensors.getGyroHeadingDeg();
        double headingError = targetHeading - currentHeading;

        // Normalize the error to the range [-180, 180]
        headingError = (headingError + 180) % 360 - 180;

        // Apply proportional correction
        return headingError * ConfigConstants.DriveControl.kP_ORIENTATION;
    }
}


