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

    private double targetHeadingDegrees = 0.0;
    private boolean isOrientationLockActive = false;

    // PID state variables
    private double integral = 0.0;
    private double previousError = 0.0;
    private double previousX = 0.0;
    private double previousY = 0.0;
    private double previousTurn = 0.0;

    public DriveRobot(Drivetrain subsystem, GamepadEx controller1, RobotState robotState, Telemetry robotTelemetry, Sensors sensors) {
        mSubsystem = subsystem;
        mController1 = controller1;
        mRobotState = robotState;
        mTelemetry = robotTelemetry;
        mSensors = sensors;

        addRequirements(mSubsystem);
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

        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            mTelemetry.addData("Y input raw", rawY);
            mTelemetry.addData("X input raw", rawX);
            mTelemetry.addData("Turn input raw", rawTurn);
        }

        // Transform raw inputs to coordinate system:
        // X+ forward, Y+ left, Turn+ CCW
        double x = rawY;
        double y = rawX;
        double turn = -rawTurn;

        // Smooth over time (EMA) to prevent abrupt changes
        x = DriveHelpers.smoothInput(previousX, x, ConfigConstants.DriveControl.INPUT_SMOOTHING_ALPHA);
        y = DriveHelpers.smoothInput(previousY, y, ConfigConstants.DriveControl.INPUT_SMOOTHING_ALPHA);
        turn = DriveHelpers.smoothInput(previousTurn, turn, ConfigConstants.DriveControl.INPUT_SMOOTHING_ALPHA);

        // Update previous values for next loop iteration
        previousX = x;
        previousY = y;
        previousTurn = turn;

        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            mTelemetry.addData("Y input signed", y);
            mTelemetry.addData("X input signed", x);
            mTelemetry.addData("Turn input signed", turn);
        }


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


        //Apply orientation lock correction if active
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

    /*
     * Prepare drive inputs by doing the following:
     *   - Applying the provided joystick dead band
     *   - Smoothing the joystick input via cubing (expects inputs in [-1, 1],
     *     and will clamp the inputs to that range if outside)
     *   - Applying scaling for slow mode as needed
     */
    private double prepareDriveInputs(double input, double deadBandSize, double slowModeFactor) {
        double prepared = DriveHelpers.deadBandJoystick(input, deadBandSize);
        //prepared = DriveHelpers.smoothJoystick(prepared);
        prepared = DriveHelpers.smoothJoystickExponential(prepared, ConfigConstants.DriveControl.SMOOTHING_POWER);
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
            targetHeadingDegrees = ConfigConstants.DriveControl.BUCKET_TARGET_DEGREES;
        } else if (submersibleOrientationLockButtonPressed) {
            isOrientationLockActive = true;
            targetHeadingDegrees = ConfigConstants.DriveControl.SUBMERSIBLE_TARGET_DEGREES;
        } else {
            isOrientationLockActive = false;
        }
    }

    private double calculateOrientationCorrection(double targetHeading) {
        double currentHeading = mSensors.getGyroHeadingDeg();
        double error = targetHeading - currentHeading;

        // Normalize the error to the range [-180, 180]
        error = (error + 180) % 360 - 180;

        // Proportional term
        double proportional = error * ConfigConstants.DriveControl.kP_ORIENTATION;

        // Integral term
        integral += error;
        double integralTerm = integral * ConfigConstants.DriveControl.kI_ORIENTATION;

        // Derivative term
        double derivative = (error - previousError) * ConfigConstants.DriveControl.kD_ORIENTATION;
        previousError = error;

        // Calculate the total correction
        double correction = proportional + integralTerm + derivative;

        // Limit the correction to a reasonable range (e.g., [-1, 1])
        correction = Math.max(-1.0, Math.min(1.0, correction));

        return correction;
    }

}




